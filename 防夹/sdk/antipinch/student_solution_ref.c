/**
 * Reference Solution for SDK Verification
 * Based on the original anti_pinch_detector.c implementation,
 * adapted to the new parameter-less interface.
 */

#include "../include/seat_sdk.h"
#include <string.h>
#include <math.h>

/* ============================================================
 * Ring Buffer (private implementation)
 * ============================================================ */
typedef struct {
    float data[BASELINE_WINDOW];
    int   head;
    int   count;
} RingBuffer;

static void ringbuf_init(RingBuffer *rb)
{
    rb->head  = 0;
    rb->count = 0;
}

static void ringbuf_push(RingBuffer *rb, float value)
{
    rb->data[rb->head] = value;
    rb->head = (rb->head + 1) % BASELINE_WINDOW;
    if (rb->count < BASELINE_WINDOW) {
        rb->count++;
    }
}

static int ringbuf_len(const RingBuffer *rb)
{
    return rb->count;
}

static void ringbuf_clear(RingBuffer *rb)
{
    rb->head  = 0;
    rb->count = 0;
}

/* ============================================================
 * Baseline Calculation
 * ============================================================ */
typedef struct {
    float mean;
    float sigma;
} BaselineResult;

static BaselineResult baseline_calc(const RingBuffer *buf, float fallback)
{
    BaselineResult result;
    int n = ringbuf_len(buf);

    if (n == 0) {
        result.mean  = fallback;
        result.sigma = 0.0f;
        return result;
    }

    double sum = 0.0;
    int start = (buf->count < BASELINE_WINDOW) ? 0 : buf->head;
    for (int i = 0; i < n; i++) {
        int idx = (start + i) % BASELINE_WINDOW;
        sum += (double)buf->data[idx];
    }
    double mean = sum / (double)n;
    result.mean = (float)mean;

    if (n == 1) {
        result.sigma = 0.0f;
        return result;
    }

    double sum_sq = 0.0;
    for (int i = 0; i < n; i++) {
        int idx = (start + i) % BASELINE_WINDOW;
        double diff = (double)buf->data[idx] - mean;
        sum_sq += diff * diff;
    }
    double variance = sum_sq / (double)n;
    result.sigma = (float)sqrt(variance);

    return result;
}

static bool in_pinch_zone(Posn_u8 posn, OperMotSt_u8 oper_st)
{
    if (oper_st != OPERMOTST_LEFT_UP && oper_st != OPERMOTST_RIGHT_DOWN)
        return false;
    return posn >= PINCH_ZONE_MIN && posn <= PINCH_ZONE_MAX;
}

/* ============================================================
 * Detector State (static, persisted across steps)
 * ============================================================ */
static struct {
    RingBuffer fwd_curr_buf;
    RingBuffer fwd_hw_buf;
    RingBuffer rev_curr_buf;
    RingBuffer rev_hw_buf;

    float curr_baseline;
    float hw_baseline;
    float curr_sigma;
    float hw_sigma;

    int startup_counter;
    OperMotSt_u8 prev_oper_st;

    Posn_u8 prev_posn;
    int   stall_counter;

    float cusum_score;
    int   cusum_frames;
    bool  pinch_active;

    float last_fwd_curr_baseline;
    float last_fwd_hw_baseline;
    float last_rev_curr_baseline;
    float last_rev_hw_baseline;
    bool  has_fwd_baseline;
    bool  has_rev_baseline;

    float curr_norm;
    float curr_dev;
    float hw_dev;
    uint8_t pinch_raw;
    HallPosn_u16 hall_posn;
} g_det;

/* ============================================================
 * Student Algorithm Implementation
 * ============================================================ */

void anti_pinch_detector_init(void){
    // 初始化算法状态，比如清空滤波器、基线、计数器
    memset(&g_det, 0, sizeof(g_det));
    ringbuf_init(&g_det.fwd_curr_buf);
    ringbuf_init(&g_det.fwd_hw_buf);
    ringbuf_init(&g_det.rev_curr_buf);
    ringbuf_init(&g_det.rev_hw_buf);

    g_det.curr_baseline = 0.0f;
    g_det.hw_baseline   = 0.0f;
    g_det.curr_sigma    = 0.0f;
    g_det.hw_sigma      = 0.0f;

    g_det.startup_counter = 0;
    g_det.prev_oper_st    = OPERMOTST_NO_RUNNING;

    g_det.prev_posn      = 0xFF;
    g_det.stall_counter  = 0;

    g_det.cusum_score  = 0.0f;
    g_det.cusum_frames = 0;
    g_det.pinch_active = false;

    g_det.last_fwd_curr_baseline = 0.0f;
    g_det.last_fwd_hw_baseline   = 0.0f;
    g_det.last_rev_curr_baseline = 0.0f;
    g_det.last_rev_hw_baseline   = 0.0f;
    g_det.has_fwd_baseline       = false;
    g_det.has_rev_baseline       = false;

    g_det.curr_dev   = 0.0f;
    g_det.hw_dev     = 0.0f;
    g_det.curr_norm  = 0.0f;
    g_det.pinch_raw  = 0;
    g_det.hall_posn  = 0;
}

void anti_pinch_detector_step(void)
{
    /* Read inputs */
    MotCurr_u16    mot_curr_val;
    Posn_u8        axis_posn_val;
    HallPlsWidth_u32 hall_width_val;
    OperMotSt_u8   oper_st_val;
    MotPwrVolt_u16 batt_vol_val;
    HallPosn_u16   hall_posn_val;

    SeatBackRclnMotDD_u16MotCur(&mot_curr_val);
    BO_Atm_SeatBackRcln_ntfPosn(&axis_posn_val);
    SeatBackRclnHallDD_u32CurrHallPlsWidth(&hall_width_val);
    BO_Atm_SeatBackRcln_ntfOperSt(&oper_st_val);
    SeatBackRclnMotDD_u16MotPwrVolt(&batt_vol_val);
    BO_Atm_SeatBackRcln_ntfHallPosn(&hall_posn_val);
    g_det.hall_posn = hall_posn_val;

    /* Voltage normalization */
    float mot_curr = (float)mot_curr_val;
    float batt_vol = (float)batt_vol_val * V_ADC_FACTOR;
    float curr_norm;
    if (batt_vol > 0.1f) {
        curr_norm = mot_curr * V_REF / batt_vol;
    } else {
        curr_norm = mot_curr;
    }
    g_det.curr_norm = curr_norm;

    /* Startup delay + baseline warmup */
    bool motor_running = (oper_st_val == OPERMOTST_LEFT_UP || oper_st_val == OPERMOTST_RIGHT_DOWN);

    if (motor_running && g_det.prev_oper_st == OPERMOTST_NO_RUNNING) {
        g_det.startup_counter = STARTUP_DELAY_SAMPLES;

        if (oper_st_val == OPERMOTST_LEFT_UP && g_det.has_fwd_baseline) {
            ringbuf_clear(&g_det.fwd_curr_buf);
            ringbuf_clear(&g_det.fwd_hw_buf);
            for (int i = 0; i < BASELINE_MIN_FILL; i++) {
                ringbuf_push(&g_det.fwd_curr_buf, g_det.last_fwd_curr_baseline);
                ringbuf_push(&g_det.fwd_hw_buf,   g_det.last_fwd_hw_baseline);
            }
        } else if (oper_st_val == OPERMOTST_RIGHT_DOWN && g_det.has_rev_baseline) {
            ringbuf_clear(&g_det.rev_curr_buf);
            ringbuf_clear(&g_det.rev_hw_buf);
            for (int i = 0; i < BASELINE_MIN_FILL; i++) {
                ringbuf_push(&g_det.rev_curr_buf, g_det.last_rev_curr_baseline);
                ringbuf_push(&g_det.rev_hw_buf,   g_det.last_rev_hw_baseline);
            }
        }
    }

    if (g_det.startup_counter > 0) {
        g_det.startup_counter--;
    }

    bool in_startup = (g_det.startup_counter > 0);

    if (!motor_running && g_det.prev_oper_st != OPERMOTST_NO_RUNNING) {
        if (ringbuf_len(&g_det.fwd_curr_buf) >= BASELINE_MIN_FILL) {
            g_det.last_fwd_curr_baseline = g_det.curr_baseline;
            g_det.last_fwd_hw_baseline   = g_det.hw_baseline;
            g_det.has_fwd_baseline = true;
        }
        if (ringbuf_len(&g_det.rev_curr_buf) >= BASELINE_MIN_FILL) {
            g_det.last_rev_curr_baseline = g_det.curr_baseline;
            g_det.last_rev_hw_baseline   = g_det.hw_baseline;
            g_det.has_rev_baseline = true;
        }
    }

    /* Adaptive baseline */
    float hw_val = (float)hall_width_val;

    const RingBuffer *buf_curr = (oper_st_val == OPERMOTST_LEFT_UP) ? &g_det.fwd_curr_buf : &g_det.rev_curr_buf;
    const RingBuffer *buf_hw   = (oper_st_val == OPERMOTST_LEFT_UP) ? &g_det.fwd_hw_buf   : &g_det.rev_hw_buf;

    BaselineResult bl_curr = baseline_calc(buf_curr, curr_norm);
    BaselineResult bl_hw   = baseline_calc(buf_hw,   hw_val);

    g_det.curr_baseline = bl_curr.mean;
    g_det.curr_sigma    = bl_curr.sigma;
    g_det.hw_baseline   = bl_hw.mean;
    g_det.hw_sigma      = bl_hw.sigma;

    g_det.curr_dev = curr_norm - g_det.curr_baseline;
    g_det.hw_dev   = hw_val - g_det.hw_baseline;

    /* Position stall detection */
    if (axis_posn_val == g_det.prev_posn) {
        g_det.stall_counter++;
    } else {
        g_det.stall_counter = 0;
    }
    g_det.prev_posn = axis_posn_val;

    /* Adaptive threshold judgment */
    int pinch_raw = 0;
    bool curr_abnormal = false;
    bool hw_abnormal = false;
    bool stall_suspect = false;
    if (motor_running && !in_startup) {
        if (in_pinch_zone(axis_posn_val, oper_st_val)) {
            float curr_thresh = fmax(g_det.curr_sigma * CURR_SIGMA_FACTOR, CURR_DEV_MIN);
            float hw_thresh   = fmax(g_det.hw_sigma   * HW_SIGMA_FACTOR,   HW_DEV_MIN);

            curr_abnormal = g_det.curr_dev > curr_thresh;
            hw_abnormal   = g_det.hw_dev   > hw_thresh;
            stall_suspect = (g_det.stall_counter >= 3);
            if (curr_abnormal && (hw_abnormal || stall_suspect)) {
                pinch_raw = 1;
            }
        }
    }
    g_det.pinch_raw = (uint8_t)pinch_raw;

    /* Update baseline only with clean samples. */
    if (motor_running && !in_startup && !g_det.pinch_active &&
        !curr_abnormal && !hw_abnormal) {
        if (oper_st_val == OPERMOTST_LEFT_UP) {
            ringbuf_push(&g_det.fwd_curr_buf, curr_norm);
            ringbuf_push(&g_det.fwd_hw_buf,   hw_val);
        } else if (oper_st_val == OPERMOTST_RIGHT_DOWN) {
            ringbuf_push(&g_det.rev_curr_buf, curr_norm);
            ringbuf_push(&g_det.rev_hw_buf,   hw_val);
        }
    }

    /* CUSUM detection (replaces duration filter) */
    #define CUSUM_K 0.0f
    #define CUSUM_H 10.0f
    #define CUSUM_MIN_FRAMES 10
    if (motor_running && !in_startup && in_pinch_zone(axis_posn_val, oper_st_val)) {
        if (pinch_raw == 1) {
            float ct = fmaxf(g_det.curr_sigma * CURR_SIGMA_FACTOR, CURR_DEV_MIN);
            float ht = fmaxf(g_det.hw_sigma   * HW_SIGMA_FACTOR,   HW_DEV_MIN);
            float curr_z = (ct > 0.1f) ? (fabsf(g_det.curr_dev) / ct) : 0.0f;
            float hw_z   = (ht > 0.1f) ? (fabsf(g_det.hw_dev)   / ht) : 0.0f;
            float score  = fmaxf(0.0f, curr_z - CUSUM_K) + fmaxf(0.0f, hw_z - CUSUM_K);
            g_det.cusum_score += score;
            g_det.cusum_frames++;
        } else {
            g_det.cusum_score = fmaxf(0.0f, g_det.cusum_score - 2.0f);
            g_det.cusum_frames = 0;
        }

        if (g_det.cusum_score > CUSUM_H && g_det.cusum_frames >= CUSUM_MIN_FRAMES) {
            g_det.pinch_active = true;
        }
    } else {
        g_det.cusum_score = fmaxf(0.0f, g_det.cusum_score - 2.0f);
        g_det.cusum_frames = 0;
    }

    if (!motor_running) {
        g_det.pinch_active = false;
        g_det.cusum_score  = 0.0f;
        g_det.cusum_frames = 0;
    }

    g_det.prev_oper_st = oper_st_val;

    /* Output */
    AntiPinchSt_u8 result = g_det.pinch_active ? ANTIPINCHST_OCCURRED : ANTIPINCHST_NORMAL;
    BO_Atm_SeatBackRcln_ntfAntiPinchSt(result);
}
