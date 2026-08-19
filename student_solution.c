/**
 * @file student_solution.c
 * @brief BEV Battery Preheat Decision Algorithm �?Student Template
 *
 * Outputs:
 *   start_distance  �?optimal preheat start distance (km from charger)
 *   T_end_opt       �?predicted battery temp at charger arrival (C)
 *   SOC_end_opt     �?predicted SOC at charger arrival (%)
 *   E_heat_opt      �?total preheat energy consumption (kWh)
 *   chrg_time_s   — predicted charging time to 80% SOC (seconds)
 *
 * How to use:
 *   1. Read initial battery state and navigation data via the provided interfaces.
 *   2. Implement your algorithm between the "TODO" markers below.
 *   3. Use BattChrgPreHeatg_ntf*() to notify the system of your results.
 *   4. Compile and run:
 *        mingw32-make
 *      or
 *        gcc -Wall -O2 -std=c11 -o run.exe student_solution.c \
 *            -I./include -L./lib -lcompetition_mock -lm
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "usp_api.h"

/* ========================================================================
 *  Physical constants & parameters (from competition spec Section 3.5)
 *
 *  These are REFERENCE DATA for your algorithm. Do not modify.
 * ======================================================================== */

/* Thermal model */
#define M_BAT       400.0f    /* kg �?battery pack mass */
#define CP          1000.0f   /* J/(kg·C) �?specific heat capacity */

/* Electrical model */
#define C_NOM_AH    100.0f    /* Ah �?nominal capacity */
#define U_NOM       350.0f    /* V �?nominal voltage */

/* PTC heater */
#define P_HEAT_MAX  6000.0f   /* W �?max heater power */
#define ETA_HEAT    0.95f     /* electro-thermal efficiency */

/* Heat transfer to ambient */
#define H0          50.0f     /* W/C �?static heat transfer coeff */
#define KV          0.5f      /* W/(C·km/h) �?speed-dependent coeff */

/* Target constraints */
#define T_OPT_LOW   20.0f     /* C �?target temp lower bound */
#define T_OPT_HIGH  25.0f     /* C �?target temp upper bound */
#define SOC_TARGET  0.80f     /* fast-charge target SOC (for reference) */
#define SOC_MIN     0.10f     /* minimum safe SOC at arrival */


/* ========================================================================
 *  R_int(temperature x SOC) MAP [Ohm]
 *
 *  Rows: -20, -10, 0, 10, 20, 30, 40 C
 *  Cols: 0%, 10%, 20%, 30%, 50%, 70%, 90%, 100%
 *
 *  Use bilinear interpolation to get R_int at any (T, SOC).
 * ======================================================================== */
static const float R_INT_TEMPS[7] = {-20.0f, -10.0f, 0.0f, 10.0f, 20.0f, 30.0f, 40.0f};
static const float R_INT_SOCS[8]  = {0.0f, 10.0f, 20.0f, 30.0f, 50.0f, 70.0f, 90.0f, 100.0f};
static const float R_INT_MAP[7][8] = {
    {0.150f, 0.120f, 0.100f, 0.090f, 0.085f, 0.082f, 0.080f, 0.078f},
    {0.100f, 0.085f, 0.075f, 0.070f, 0.065f, 0.062f, 0.060f, 0.058f},
    {0.070f, 0.060f, 0.052f, 0.048f, 0.045f, 0.043f, 0.042f, 0.041f},
    {0.050f, 0.042f, 0.038f, 0.035f, 0.033f, 0.032f, 0.031f, 0.030f},
    {0.038f, 0.032f, 0.029f, 0.027f, 0.025f, 0.024f, 0.023f, 0.023f},
    {0.032f, 0.028f, 0.025f, 0.023f, 0.022f, 0.021f, 0.020f, 0.020f},
    {0.028f, 0.025f, 0.022f, 0.021f, 0.020f, 0.019f, 0.019f, 0.018f},
};

/* ========================================================================
 *  U_oc(temperature x SOC) MAP [V, pack voltage]
 *
 *  You can either call EMS_HVBatt_getVolt() for current operating point,
 *  or use bilinear interpolation on this MAP for algorithm calculations
 *  (e.g., I = P/V). You need to implement the interpolation yourself.
 * ======================================================================== */

static const float U_OC_TEMPS[7] = {-20.0f, -10.0f, 0.0f, 10.0f, 20.0f, 30.0f, 40.0f};
static const float U_OC_SOCS[8]  = {0.0f, 10.0f, 20.0f, 30.0f, 50.0f, 70.0f, 90.0f, 100.0f};
static const float UOC_MAP[7][8] = {
    {295.0f, 320.0f, 340.0f, 352.0f, 365.0f, 375.0f, 382.0f, 410.0f},
    {298.0f, 323.0f, 343.0f, 355.0f, 368.0f, 378.0f, 385.0f, 413.0f},
    {300.0f, 325.0f, 345.0f, 357.0f, 370.0f, 380.0f, 387.0f, 415.0f},
    {302.0f, 327.0f, 347.0f, 359.0f, 372.0f, 382.0f, 389.0f, 417.0f},
    {303.0f, 328.0f, 348.0f, 360.0f, 373.0f, 383.0f, 390.0f, 418.0f},
    {304.0f, 329.0f, 349.0f, 361.0f, 374.0f, 384.0f, 391.0f, 419.0f},
    {305.0f, 330.0f, 350.0f, 362.0f, 375.0f, 385.0f, 392.0f, 420.0f},
};


/* ========================================================================
 *  P_charge(temperature x SOC) MAP [kW, max charging power]
 *
 *  Rows: -20, -10, 0, 10, 23, 30, 40 C
 *  Cols: 0%, 10%, 20%, 30%, 50%, 70%, 90%, 100%
 * ======================================================================== */
static const float P_CHARGE_TEMPS[7] = {-20.0f, -10.0f, 0.0f, 10.0f, 23.0f, 30.0f, 40.0f};
static const float P_CHARGE_SOCS[8]  = {0.0f, 10.0f, 20.0f, 30.0f, 50.0f, 70.0f, 90.0f, 100.0f};
static const float P_CHARGE_MAP[7][8] = {
    {10.0f, 12.0f, 15.0f, 18.0f, 20.0f, 18.0f, 15.0f, 10.0f},
    {20.0f, 25.0f, 30.0f, 35.0f, 38.0f, 35.0f, 28.0f, 20.0f},
    {40.0f, 50.0f, 60.0f, 65.0f, 68.0f, 65.0f, 55.0f, 40.0f},
    {60.0f, 70.0f, 80.0f, 85.0f, 88.0f, 85.0f, 70.0f, 50.0f},
    {75.0f, 85.0f, 90.0f, 92.0f, 90.0f, 88.0f, 75.0f, 55.0f},
    {70.0f, 80.0f, 88.0f, 90.0f, 88.0f, 85.0f, 70.0f, 50.0f},
    {50.0f, 60.0f, 70.0f, 72.0f, 70.0f, 65.0f, 55.0f, 40.0f},
};

/* ========================================================================
 *  Navigation segment data
 * ======================================================================== */
#define MAX_SEGS  16

typedef struct {
    float s_km;       /* segment distance, km */
    float v_kmh;      /* average speed, km/h */
    float P_drive_kW; /* drive power, kW (positive = discharge) */
    float T_env_C;    /* ambient temperature, C */
} NavSeg;

static NavSeg   g_segs[MAX_SEGS];
static int      g_n_segs = 0;

/* Load navigation data from VehPwrPred_getPwrPred() interface */
static void load_nav_segments(void)
{
    PwrPredList_stru pred;
    VehPwrPred_getPwrPred(&pred);
    g_n_segs = (pred.num < MAX_SEGS) ? pred.num : MAX_SEGS;

    for (int i = 0; i < g_n_segs; i++) {
        g_segs[i].s_km       = pred.pwrPredList[i].length;
        g_segs[i].v_kmh      = pred.pwrPredList[i].avgSpd;
        g_segs[i].P_drive_kW = pred.pwrPredList[i].drvPwr;
        g_segs[i].T_env_C    = pred.pwrPredList[i].ambTemp;
    }
}

/* ========================================================================
 *  TODO: Implement your algorithm functions below
 *
 * ======================================================================== */
/* --- Your algorithm code starts here --- */
/* 时间边界搜索：1 s 负责快速定位，0.2 s 负责最终边界和候选评分。 */
#define FAST_DT_S          1.0f
#define FINAL_DT_S         0.2f
#define OUTER_UPDATE_S     1.0f
#define COARSE_STEP_S     60.0f
#define FINE_STEP_S        5.0f
#define REFINE_COST_MARGIN 0.01f
#define MAX_PARETO_REFINE  3
#define SCORE_EPS          1.0e-6f

typedef struct { float temp_C; float soc; } BatteryState;
typedef struct { float temp_C; float soc; float heat_kWh; } TimeRouteResult;

typedef struct {
    float dt_s;
    float total_time_s;
    float route_km;
    int steps;
    BatteryState *no_heat_state;
    int *segment_at_step;
    float *step_duration_s;
} RouteBaseline;

typedef struct { RouteBaseline fast; RouteBaseline final; } RouteContext;

typedef enum {
    TIME_LEVEL_COARSE = 0,
    TIME_LEVEL_FINE = 1,
    TIME_LEVEL_FINAL = 2
} TimeSearchLevel;

enum {
    CAND_NEAR_BEST    = 1u << 0,
    CAND_MIN_CHARGE   = 1u << 1,
    CAND_MAX_CHARGE   = 1u << 2,
    CAND_LOCAL_CHARGE = 1u << 3,
    CAND_PARETO       = 1u << 4,
    CAND_LOCAL_SCORE  = 1u << 5,
    CAND_ENDPOINT     = 1u << 6
};

typedef struct {
    int start_step;              /* 0.2 s 启动步编号：候选唯一键 */
    float start_distance_km;
    float heat_time_s;
    float temp_C;
    float soc_pct;
    float heat_kWh;
    float charge_s;
    float cost;
    int feasible;
    TimeSearchLevel level;
    int refined_to_next;
    unsigned int flags;
} TimeCandidate;

typedef struct {
    TimeCandidate *data;
    int *index_by_step;
    int count;
    int capacity;
    int step_count;
} TimePool;

typedef struct {
    float min_energy;
    float max_energy;
    float min_charge;
    float max_charge;
    int valid;
} TimeScale;

typedef struct {
    int valid;
    int earliest_start_step;
    int latest_start_step;
    float t20_s;
    float t25_s;
    float t_soc_s;
    float t_min_s;
    float t_max_s;
    float min_heat_kWh;
    float max_heat_kWh;
} FeasibleTimeBounds;

typedef struct {
    int feasible;
    int best_start_step;
    float best_start_distance_km;
    float T_end_C;
    float SOC_end_pct;
    float E_heat_kWh;
    float charge_time_s;
    float min_charge_s;
    float max_charge_s;
    float min_heat_kWh;
    float max_heat_kWh;
    int evaluated_final_candidates;
} PreheatOptimizationResult;

typedef struct { int start_step; int index; } TimeCandidateRef;
typedef enum { BOUNDARY_T20, BOUNDARY_T25, BOUNDARY_SOC } BoundaryKind;

/* 原二维 MAP、轴点和双线性插值算法保持不变。 */
static float interp_map(const float temp_axis[7], const float soc_axis[8],
                        const float map[7][8], float temp_C, float soc_pct)
{
    const float temp = fminf(temp_axis[6], fmaxf(temp_axis[0], temp_C));
    const float soc = fminf(soc_axis[7], fmaxf(soc_axis[0], soc_pct));
    int ti;
    int si;
    float tf;
    float sf;
    float lower;
    float upper;

    /* 固定 7x8 轴用二分分支定位区间，插值点和双线性公式不变。 */
    if (temp > temp_axis[3])
        ti = (temp > temp_axis[5]) ? 5
           : (temp > temp_axis[4]) ? 4 : 3;
    else
        ti = (temp > temp_axis[1])
           ? ((temp > temp_axis[2]) ? 2 : 1) : 0;
    if (soc > soc_axis[4])
        si = (soc > soc_axis[6]) ? 6
           : (soc > soc_axis[5]) ? 5 : 4;
    else
        si = (soc > soc_axis[2])
           ? ((soc > soc_axis[3]) ? 3 : 2)
           : ((soc > soc_axis[1]) ? 1 : 0);
    tf = (temp - temp_axis[ti]) / (temp_axis[ti + 1] - temp_axis[ti]);
    sf = (soc - soc_axis[si]) / (soc_axis[si + 1] - soc_axis[si]);
    lower = map[ti][si] + sf * (map[ti][si + 1] - map[ti][si]);
    upper = map[ti + 1][si] +
            sf * (map[ti + 1][si + 1] - map[ti + 1][si]);
    return lower + tf * (upper - lower);
}

/* 三个函数只是语义包装；不增加插值点，也不做函数拟合。 */
static float battery_resistance(float temp_C, float soc)
{
    return interp_map(R_INT_TEMPS, R_INT_SOCS, R_INT_MAP,
                      temp_C, soc * 100.0f);
}

static float open_circuit_voltage(float temp_C, float soc)
{
    return interp_map(U_OC_TEMPS, U_OC_SOCS, UOC_MAP,
                      temp_C, soc * 100.0f);
}

static float charge_power(float temp_C, float soc)
{
    return interp_map(P_CHARGE_TEMPS, P_CHARGE_SOCS, P_CHARGE_MAP,
                      temp_C, soc * 100.0f);
}

static float route_segment_time_s(int seg)
{
    return (g_segs[seg].v_kmh > 0.0f)
         ? 3600.0f * g_segs[seg].s_km / g_segs[seg].v_kmh
         : 0.0f;
}

static void get_route_totals(float *route_km, float *route_time_s)
{
    *route_km = 0.0f;
    *route_time_s = 0.0f;
    for (int seg = 0; seg < g_n_segs; ++seg) {
        if (g_segs[seg].v_kmh <= 0.0f) continue;
        *route_km += g_segs[seg].s_km;
        *route_time_s += route_segment_time_s(seg);
    }
}

static int route_segment_at_time(float time_s)
{
    float elapsed = 0.0f;
    int last_valid = -1;
    for (int seg = 0; seg < g_n_segs; ++seg) {
        const float duration = route_segment_time_s(seg);
        if (duration <= 0.0f) continue;
        last_valid = seg;
        if (time_s < elapsed + duration - SCORE_EPS) return seg;
        elapsed += duration;
    }
    return last_valid;
}

static float remaining_distance_at_time(float time_s, float route_km)
{
    float elapsed = fmaxf(0.0f, time_s);
    float traveled_km = 0.0f;
    if (elapsed <= 0.0f) return route_km;

    for (int seg = 0; seg < g_n_segs; ++seg) {
        const float duration = route_segment_time_s(seg);
        if (duration <= 0.0f) continue;
        if (elapsed >= duration) {
            traveled_km += g_segs[seg].s_km;
            elapsed -= duration;
        } else {
            traveled_km += g_segs[seg].v_kmh * elapsed / 3600.0f;
            break;
        }
    }
    return fmaxf(0.0f, route_km - traveled_km);
}

/* 原电模型、热模型、SOC 更新公式原样用于任意 dt。 */
static void integrate_battery_state(BatteryState *state, int seg,
                                    int heating, float dt_s)
{
    const float resistance = battery_resistance(state->temp_C, state->soc);
    const float uoc = open_circuit_voltage(state->temp_C, state->soc);
    const float power_w = g_segs[seg].P_drive_kW * 1000.0f +
                          (heating ? P_HEAT_MAX : 0.0f);
    float discriminant = uoc * uoc - 4.0f * resistance * power_w;
    float current;
    float generated_heat;
    float heat_loss;

    if (discriminant < 0.0f) discriminant = 0.0f;
    current = (fabsf(resistance) > 1.0e-8f)
            ? (uoc - sqrtf(discriminant)) / (2.0f * resistance)
            : power_w / fmaxf(uoc, 1.0f);
    generated_heat = current * current * resistance;
    heat_loss = (H0 + KV * g_segs[seg].v_kmh) *
                (state->temp_C - g_segs[seg].T_env_C);
    state->temp_C += (generated_heat +
        (heating ? ETA_HEAT * P_HEAT_MAX : 0.0f) - heat_loss) *
        dt_s / (M_BAT * CP);
    state->soc -= current * dt_s / (C_NOM_AH * 3600.0f);
}

static float baseline_step_time(const RouteBaseline *baseline, int step)
{
    return fminf(baseline->total_time_s,
                 fmaxf(0.0f, step * baseline->dt_s));
}

/* 每个外部 1 s 周期内，5 个 0.2 s 子步保持导航输入不变。 */
static int build_no_heat_baseline(RouteBaseline *baseline, float dt_s,
                                  float total_time_s, float route_km,
                                  float T_init, float SOC_init)
{
    baseline->dt_s = dt_s;
    baseline->total_time_s = total_time_s;
    baseline->route_km = route_km;
    baseline->steps = (int)ceilf(
        fmaxf(0.0f, total_time_s - SCORE_EPS) / dt_s);
    baseline->no_heat_state = (BatteryState *)malloc(
        sizeof(BatteryState) * (size_t)(baseline->steps + 1));
    baseline->segment_at_step = (int *)malloc(
        sizeof(int) * (size_t)baseline->steps);
    baseline->step_duration_s = (float *)malloc(
        sizeof(float) * (size_t)baseline->steps);
    if (!baseline->no_heat_state || !baseline->segment_at_step ||
        !baseline->step_duration_s) {
        free(baseline->no_heat_state);
        free(baseline->segment_at_step);
        free(baseline->step_duration_s);
        baseline->no_heat_state = NULL;
        baseline->segment_at_step = NULL;
        baseline->step_duration_s = NULL;
        return 0;
    }
    baseline->no_heat_state[0].temp_C = T_init;
    baseline->no_heat_state[0].soc = SOC_init;

    for (int step = 0; step < baseline->steps; ++step) {
        const float t0 = baseline_step_time(baseline, step);
        const float t1 = baseline_step_time(baseline, step + 1);
        const float sample_time =
            floorf(t0 / OUTER_UPDATE_S + 1.0e-4f) * OUTER_UPDATE_S;
        const int seg = route_segment_at_time(sample_time);
        baseline->segment_at_step[step] = seg;
        baseline->step_duration_s[step] = t1 - t0;
        baseline->no_heat_state[step + 1] =
            baseline->no_heat_state[step];
        if (seg >= 0 && t1 > t0)
            integrate_battery_state(&baseline->no_heat_state[step + 1],
                                    seg, 0, t1 - t0);
    }
    return 1;
}

static void free_route_context(RouteContext *context)
{
    free(context->fast.no_heat_state);
    free(context->fast.segment_at_step);
    free(context->fast.step_duration_s);
    free(context->final.no_heat_state);
    free(context->final.segment_at_step);
    free(context->final.step_duration_s);
    context->fast.no_heat_state = NULL;
    context->fast.segment_at_step = NULL;
    context->fast.step_duration_s = NULL;
    context->final.no_heat_state = NULL;
    context->final.segment_at_step = NULL;
    context->final.step_duration_s = NULL;
}

static int build_route_context(RouteContext *context,
                               float T_init, float SOC_init)
{
    float route_km;
    float route_time_s;
    context->fast.no_heat_state = NULL;
    context->fast.segment_at_step = NULL;
    context->fast.step_duration_s = NULL;
    context->final.no_heat_state = NULL;
    context->final.segment_at_step = NULL;
    context->final.step_duration_s = NULL;
    get_route_totals(&route_km, &route_time_s);
    if (route_km <= 0.0f || route_time_s <= 0.0f) return 0;
    if (!build_no_heat_baseline(&context->fast, FAST_DT_S,
        route_time_s, route_km, T_init, SOC_init)) return 0;
    if (!build_no_heat_baseline(&context->final, FINAL_DT_S,
        route_time_s, route_km, T_init, SOC_init)) {
        free_route_context(context);
        return 0;
    }
    return 1;
}

static TimeRouteResult simulate_heating_suffix(
    const RouteBaseline *baseline, int start_step)
{
    TimeRouteResult result;
    BatteryState state;
    int clipped = start_step;
    float start_time;

    if (clipped < 0) clipped = 0;
    if (clipped > baseline->steps) clipped = baseline->steps;
    start_time = baseline_step_time(baseline, clipped);
    state = baseline->no_heat_state[clipped];
    for (int step = clipped; step < baseline->steps; ++step) {
        const int seg = baseline->segment_at_step[step];
        const float dt_s = baseline->step_duration_s[step];
        if (seg >= 0 && dt_s > 0.0f)
            integrate_battery_state(&state, seg, 1, dt_s);
    }
    result.temp_C = state.temp_C;
    result.soc = state.soc;
    result.heat_kWh = (P_HEAT_MAX / 1000.0f) *
        (baseline->total_time_s - start_time) / 3600.0f;
    return result;
}

/* PDF 3.4 终点功率恒定近似保持不变。 */
static float estimate_charge_time(float temp_C, float start_soc)
{
    const float charge_kw = fmaxf(0.1f, charge_power(temp_C, start_soc));
    if (start_soc >= SOC_TARGET) return 0.0f;
    return (SOC_TARGET - start_soc) * C_NOM_AH * U_NOM *
           3.6f / charge_kw;
}

static int time_pool_init(TimePool *pool, int step_count)
{
    pool->capacity = step_count + 1;
    pool->step_count = step_count;
    pool->count = 0;
    pool->data = (TimeCandidate *)malloc(
        sizeof(TimeCandidate) * (size_t)pool->capacity);
    pool->index_by_step = (int *)malloc(
        sizeof(int) * (size_t)(step_count + 1));
    if (!pool->data || !pool->index_by_step) {
        free(pool->data);
        free(pool->index_by_step);
        pool->data = NULL;
        pool->index_by_step = NULL;
        return 0;
    }
    for (int step = 0; step <= step_count; ++step)
        pool->index_by_step[step] = -1;
    return 1;
}

static void time_pool_free(TimePool *pool)
{
    free(pool->data);
    free(pool->index_by_step);
    pool->data = NULL;
    pool->index_by_step = NULL;
    pool->count = 0;
}

static int time_pool_store(TimePool *pool, const TimeCandidate *candidate)
{
    const int step = candidate->start_step;
    int index;
    if (step < 0 || step > pool->step_count) return -1;
    index = pool->index_by_step[step];
    if (index >= 0) {
        if (candidate->level < pool->data[index].level) {
            pool->data[index].level = candidate->level;
            pool->data[index].refined_to_next = 0;
        }
        return index;
    }
    if (pool->count >= pool->capacity) return -1;
    index = pool->count++;
    pool->data[index] = *candidate;
    pool->index_by_step[step] = index;
    return index;
}

static TimeCandidate make_time_candidate(const RouteBaseline *baseline,
                                         int start_step,
                                         TimeSearchLevel level)
{
    TimeCandidate candidate;
    const TimeRouteResult route =
        simulate_heating_suffix(baseline, start_step);
    const float start_time = baseline_step_time(baseline, start_step);
    candidate.start_step = start_step;
    candidate.start_distance_km =
        remaining_distance_at_time(start_time, baseline->route_km);
    candidate.heat_time_s = baseline->total_time_s - start_time;
    candidate.temp_C = route.temp_C;
    candidate.soc_pct = route.soc * 100.0f;
    candidate.heat_kWh = route.heat_kWh;
    candidate.charge_s = estimate_charge_time(route.temp_C, route.soc);
    candidate.cost = INFINITY;
    candidate.feasible = route.temp_C >= T_OPT_LOW &&
                         route.temp_C <= T_OPT_HIGH &&
                         route.soc >= SOC_MIN;
    candidate.level = level;
    candidate.refined_to_next = 0;
    candidate.flags = 0u;
    return candidate;
}

static int evaluate_final_candidate(const RouteContext *context,
                                    int start_step,
                                    TimeSearchLevel level,
                                    TimePool *pool)
{
    int clipped = start_step;
    int index;
    if (clipped < 0) clipped = 0;
    if (clipped > context->final.steps) clipped = context->final.steps;
    index = pool->index_by_step[clipped];
    if (index >= 0) {
        if (level < pool->data[index].level) {
            pool->data[index].level = level;
            pool->data[index].refined_to_next = 0;
        }
        return index;
    }
    {
        const TimeCandidate candidate =
            make_time_candidate(&context->final, clipped, level);
        return time_pool_store(pool, &candidate);
    }
}

static int evaluate_fast_candidate(const RouteContext *context,
                                   int final_step, TimePool *pool)
{
    const float start_time =
        baseline_step_time(&context->final, final_step);
    int fast_step = (int)floorf(start_time / FAST_DT_S + 0.5f);
    TimeCandidate candidate;
    TimeRouteResult route;
    if (fast_step < 0) fast_step = 0;
    if (fast_step > context->fast.steps) fast_step = context->fast.steps;
    route = simulate_heating_suffix(&context->fast, fast_step);
    candidate.start_step = final_step;
    candidate.start_distance_km =
        remaining_distance_at_time(start_time, context->final.route_km);
    candidate.heat_time_s = context->final.total_time_s - start_time;
    candidate.temp_C = route.temp_C;
    candidate.soc_pct = route.soc * 100.0f;
    candidate.heat_kWh = (P_HEAT_MAX / 1000.0f) *
                         candidate.heat_time_s / 3600.0f;
    candidate.charge_s = estimate_charge_time(route.temp_C, route.soc);
    candidate.cost = INFINITY;
    candidate.feasible = route.temp_C >= T_OPT_LOW &&
                         route.temp_C <= T_OPT_HIGH &&
                         route.soc >= SOC_MIN;
    candidate.level = TIME_LEVEL_COARSE;
    candidate.refined_to_next = 0;
    candidate.flags = 0u;
    return time_pool_store(pool, &candidate);
}

static int boundary_condition(const TimeRouteResult *route,
                              BoundaryKind kind)
{
    if (kind == BOUNDARY_T20) return route->temp_C >= T_OPT_LOW;
    if (kind == BOUNDARY_T25) return route->temp_C <= T_OPT_HIGH;
    return route->soc >= SOC_MIN;
}

static int fast_boundary_condition(const RouteContext *context,
                                   int step, BoundaryKind kind)
{
    const TimeRouteResult route =
        simulate_heating_suffix(&context->fast, step);
    return boundary_condition(&route, kind);
}

/* first_true=1 搜 false->true 首点；否则搜 true->false 末点。 */
static int find_fast_boundary(const RouteContext *context,
                              BoundaryKind kind, int first_true)
{
    int low = 0;
    int high = context->fast.steps;
    if (first_true) {
        if (fast_boundary_condition(context, low, kind)) return low;
        if (!fast_boundary_condition(context, high, kind)) return -1;
        while (high - low > 1) {
            const int mid = low + (high - low) / 2;
            if (fast_boundary_condition(context, mid, kind)) high = mid;
            else low = mid;
        }
        return high;
    }
    if (fast_boundary_condition(context, high, kind)) return high;
    if (!fast_boundary_condition(context, low, kind)) return -1;
    while (high - low > 1) {
        const int mid = low + (high - low) / 2;
        if (fast_boundary_condition(context, mid, kind)) low = mid;
        else high = mid;
    }
    return low;
}

static int final_boundary_condition(const RouteContext *context,
                                    TimePool *pool, int step,
                                    BoundaryKind kind)
{
    const int index = evaluate_final_candidate(
        context, step, TIME_LEVEL_FINE, pool);
    TimeRouteResult route;
    if (index < 0) return 0;
    route.temp_C = pool->data[index].temp_C;
    route.soc = pool->data[index].soc_pct / 100.0f;
    route.heat_kWh = pool->data[index].heat_kWh;
    return boundary_condition(&route, kind);
}

/*
 * 用稀疏 1 s 探针检查二分所需的趋势。发现反转时，后续改走 0.2 s
 * 全步枚举兜底，不能继续把单调二分结果当作正式边界。
 */
static int fast_boundary_trends_are_monotonic(const RouteContext *context)
{
    int probe_step = (int)(COARSE_STEP_S / FAST_DT_S + 0.5f);
    int seen_t20_false = 0;
    int seen_t25_true = 0;
    int seen_soc_true = 0;
    if (probe_step < 1) probe_step = 1;

    for (int step = 0;;) {
        const TimeRouteResult route =
            simulate_heating_suffix(&context->fast, step);
        const int t20 = boundary_condition(&route, BOUNDARY_T20);
        const int t25 = boundary_condition(&route, BOUNDARY_T25);
        const int soc = boundary_condition(&route, BOUNDARY_SOC);

        if (!t20) seen_t20_false = 1;
        else if (seen_t20_false) return 0;
        if (t25) seen_t25_true = 1;
        else if (seen_t25_true) return 0;
        if (soc) seen_soc_true = 1;
        else if (seen_soc_true) return 0;

        if (step == context->fast.steps) break;
        step += probe_step;
        if (step > context->fast.steps) step = context->fast.steps;
    }
    return 1;
}

/* 非单调兜底：只在趋势探针触发时枚举全部 0.2 s 启动步。 */
static int find_feasible_time_bounds_exhaustive(
    const RouteContext *context, TimePool *pool,
    FeasibleTimeBounds *bounds)
{
    int earliest = -1;
    int latest = -1;
    for (int step = 0; step <= context->final.steps; ++step) {
        const int index = evaluate_final_candidate(
            context, step, TIME_LEVEL_FINAL, pool);
        if (index >= 0 && pool->data[index].feasible) {
            if (earliest < 0) earliest = step;
            latest = step;
        }
    }
    if (earliest < 0) return 0;

    bounds->earliest_start_step = earliest;
    bounds->latest_start_step = latest;
    bounds->t_min_s = context->final.total_time_s -
        baseline_step_time(&context->final, latest);
    bounds->t_max_s = context->final.total_time_s -
        baseline_step_time(&context->final, earliest);
    bounds->t20_s = bounds->t_min_s;
    bounds->t25_s = bounds->t_max_s;
    bounds->t_soc_s = bounds->t_max_s;
    bounds->min_heat_kWh = (P_HEAT_MAX / 1000.0f) *
                           bounds->t_min_s / 3600.0f;
    bounds->max_heat_kWh = (P_HEAT_MAX / 1000.0f) *
                           bounds->t_max_s / 3600.0f;
    bounds->valid = 1;
    return 1;
}

/* 用 1 s 结果作初值，指数扩展后在 0.2 s 步号上二分精修。 */
static int refine_time_boundary(const RouteContext *context,
                                TimePool *pool, int fast_step,
                                BoundaryKind kind, int first_true)
{
    const float guess_time =
        baseline_step_time(&context->fast, fast_step);
    int guess = (int)floorf(guess_time / FINAL_DT_S + 0.5f);
    int low;
    int high;
    int span = (int)(FAST_DT_S / FINAL_DT_S);
    if (guess < 0) guess = 0;
    if (guess > context->final.steps) guess = context->final.steps;
    if (span < 1) span = 1;

    if (first_true) {
        if (final_boundary_condition(context, pool, guess, kind)) {
            high = guess;
            low = (guess > span) ? guess - span : 0;
            while (low > 0 &&
                   final_boundary_condition(context, pool, low, kind)) {
                high = low;
                span *= 2;
                low = (low > span) ? low - span : 0;
            }
            if (low == 0 &&
                final_boundary_condition(context, pool, low, kind))
                return 0;
        } else {
            low = guess;
            high = (guess + span < context->final.steps)
                 ? guess + span : context->final.steps;
            while (high < context->final.steps &&
                   !final_boundary_condition(context, pool, high, kind)) {
                low = high;
                span *= 2;
                high = (high + span < context->final.steps)
                     ? high + span : context->final.steps;
            }
            if (high == context->final.steps &&
                !final_boundary_condition(context, pool, high, kind))
                return -1;
        }
        while (high - low > 1) {
            const int mid = low + (high - low) / 2;
            if (final_boundary_condition(context, pool, mid, kind)) high = mid;
            else low = mid;
        }
        return high;
    }

    if (final_boundary_condition(context, pool, guess, kind)) {
        low = guess;
        high = (guess + span < context->final.steps)
             ? guess + span : context->final.steps;
        while (high < context->final.steps &&
               final_boundary_condition(context, pool, high, kind)) {
            low = high;
            span *= 2;
            high = (high + span < context->final.steps)
                 ? high + span : context->final.steps;
        }
        if (high == context->final.steps &&
            final_boundary_condition(context, pool, high, kind))
            return context->final.steps;
    } else {
        high = guess;
        low = (guess > span) ? guess - span : 0;
        while (low > 0 &&
               !final_boundary_condition(context, pool, low, kind)) {
            high = low;
            span *= 2;
            low = (low > span) ? low - span : 0;
        }
        if (low == 0 &&
            !final_boundary_condition(context, pool, low, kind))
            return -1;
    }
    while (high - low > 1) {
        const int mid = low + (high - low) / 2;
        if (final_boundary_condition(context, pool, mid, kind)) low = mid;
        else high = mid;
    }
    return low;
}

static int find_feasible_time_bounds(const RouteContext *context,
                                     TimePool *pool,
                                     FeasibleTimeBounds *bounds)
{
    const int fast_t20 = find_fast_boundary(context, BOUNDARY_T20, 0);
    const int fast_t25 = find_fast_boundary(context, BOUNDARY_T25, 1);
    const int fast_soc = find_fast_boundary(context, BOUNDARY_SOC, 1);
    int step_t20;
    int step_t25;
    int step_soc;
    int earliest;
    int latest;

    bounds->valid = 0;
    if (!fast_boundary_trends_are_monotonic(context))
        return find_feasible_time_bounds_exhaustive(context, pool, bounds);
    /*
     * 1 s 结果只提供初值，不能否决 0.2 s 模型。若快速模型在极窄
     * 区间上没有找到边界，就从相应路线端点启动精确边界搜索。
     */
    step_t20 = refine_time_boundary(
        context, pool, fast_t20 >= 0 ? fast_t20 : 0,
        BOUNDARY_T20, 0);
    step_t25 = refine_time_boundary(
        context, pool,
        fast_t25 >= 0 ? fast_t25 : context->fast.steps,
        BOUNDARY_T25, 1);
    step_soc = refine_time_boundary(
        context, pool,
        fast_soc >= 0 ? fast_soc : context->fast.steps,
        BOUNDARY_SOC, 1);
    if (step_t20 < 0 || step_t25 < 0 || step_soc < 0) return 0;

    earliest = (step_t25 > step_soc) ? step_t25 : step_soc;
    latest = step_t20;
    /* 四个归一化边界必须落在通过全部硬约束的 0.2 s 候选上。 */
    while (earliest <= latest) {
        const int index = evaluate_final_candidate(
            context, earliest, TIME_LEVEL_FINE, pool);
        if (index >= 0 && pool->data[index].feasible) break;
        ++earliest;
    }
    while (latest >= earliest) {
        const int index = evaluate_final_candidate(
            context, latest, TIME_LEVEL_FINE, pool);
        if (index >= 0 && pool->data[index].feasible) break;
        --latest;
    }
    if (earliest > latest) return 0;

    bounds->earliest_start_step = earliest;
    bounds->latest_start_step = latest;
    bounds->t20_s = context->final.total_time_s -
        baseline_step_time(&context->final, step_t20);
    bounds->t25_s = context->final.total_time_s -
        baseline_step_time(&context->final, step_t25);
    bounds->t_soc_s = context->final.total_time_s -
        baseline_step_time(&context->final, step_soc);
    bounds->t_min_s = context->final.total_time_s -
        baseline_step_time(&context->final, latest);
    bounds->t_max_s = context->final.total_time_s -
        baseline_step_time(&context->final, earliest);
    bounds->min_heat_kWh = (P_HEAT_MAX / 1000.0f) *
                           bounds->t_min_s / 3600.0f;
    bounds->max_heat_kWh = (P_HEAT_MAX / 1000.0f) *
                           bounds->t_max_s / 3600.0f;
    bounds->valid = bounds->t_min_s <= bounds->t_max_s + SCORE_EPS;
    return bounds->valid;
}

static int compare_time_candidate_ref(const void *lhs, const void *rhs)
{
    const TimeCandidateRef *a = (const TimeCandidateRef *)lhs;
    const TimeCandidateRef *b = (const TimeCandidateRef *)rhs;
    return (a->start_step > b->start_step) -
           (a->start_step < b->start_step);
}

static int time_candidate_in_bounds(const TimeCandidate *candidate,
                                    const FeasibleTimeBounds *bounds)
{
    return candidate->feasible &&
           candidate->start_step >= bounds->earliest_start_step &&
           candidate->start_step <= bounds->latest_start_step;
}

/* 能耗边界直接取 t_min/t_max；候选池只统计充电时间边界。 */
static TimeScale score_time_pool(TimePool *pool,
                                 const FeasibleTimeBounds *bounds)
{
    TimeScale scale = {
        bounds->min_heat_kWh, bounds->max_heat_kWh,
        INFINITY, -INFINITY, 0
    };
    for (int i = 0; i < pool->count; ++i) {
        TimeCandidate *candidate = &pool->data[i];
        candidate->cost = INFINITY;
        candidate->flags = 0u;
        if (!time_candidate_in_bounds(candidate, bounds)) continue;
        scale.min_charge = fminf(scale.min_charge, candidate->charge_s);
        scale.max_charge = fmaxf(scale.max_charge, candidate->charge_s);
        scale.valid = 1;
    }
    if (!scale.valid) return scale;

    for (int i = 0; i < pool->count; ++i) {
        TimeCandidate *candidate = &pool->data[i];
        float energy_cost;
        float charge_cost;
        if (!time_candidate_in_bounds(candidate, bounds)) continue;
        energy_cost = (scale.max_energy - scale.min_energy > SCORE_EPS)
            ? (candidate->heat_kWh - scale.min_energy) /
              (scale.max_energy - scale.min_energy) : 0.0f;
        charge_cost = (scale.max_charge - scale.min_charge > SCORE_EPS)
            ? (candidate->charge_s - scale.min_charge) /
              (scale.max_charge - scale.min_charge) : 0.0f;
        candidate->cost = 0.4f * energy_cost + 0.6f * charge_cost;
    }
    return scale;
}

/* 排序后同时标记全局极值、局部极值、Pareto、端点和近优 score。 */
static int refresh_time_flags(TimePool *pool,
                              const FeasibleTimeBounds *bounds,
                              TimeScale *out_scale)
{
    TimeCandidateRef *refs;
    TimeScale scale = score_time_pool(pool, bounds);
    int count = 0;
    int best_index = -1;
    float best_cost = INFINITY;
    float pareto_best_charge = INFINITY;
    int pareto_count = 0;

    *out_scale = scale;
    if (!scale.valid) return -1;
    for (int i = 0; i < pool->count; ++i) {
        TimeCandidate *candidate = &pool->data[i];
        if (!time_candidate_in_bounds(candidate, bounds)) continue;
        if (candidate->cost < best_cost - SCORE_EPS ||
            (fabsf(candidate->cost - best_cost) <= SCORE_EPS &&
             (best_index < 0 || candidate->heat_kWh <
              pool->data[best_index].heat_kWh))) {
            best_cost = candidate->cost;
            best_index = i;
        }
        ++count;
    }

    refs = (TimeCandidateRef *)malloc(
        sizeof(TimeCandidateRef) * (size_t)count);
    if (!refs) return best_index;
    count = 0;
    for (int i = 0; i < pool->count; ++i) {
        TimeCandidate *candidate = &pool->data[i];
        if (!time_candidate_in_bounds(candidate, bounds)) continue;
        if (candidate->cost <= best_cost + REFINE_COST_MARGIN)
            candidate->flags |= CAND_NEAR_BEST;
        if (candidate->charge_s <= scale.min_charge + SCORE_EPS)
            candidate->flags |= CAND_MIN_CHARGE;
        if (candidate->charge_s >= scale.max_charge - SCORE_EPS)
            candidate->flags |= CAND_MAX_CHARGE;
        if (candidate->start_step == bounds->earliest_start_step ||
            candidate->start_step == bounds->latest_start_step)
            candidate->flags |= CAND_ENDPOINT;
        refs[count].start_step = candidate->start_step;
        refs[count].index = i;
        ++count;
    }
    qsort(refs, (size_t)count, sizeof(TimeCandidateRef),
          compare_time_candidate_ref);

    /* 从最少能耗端反向扫描：充电时间刷新下界的点即非支配点。 */
    for (int i = count - 1; i >= 0; --i) {
        TimeCandidate *candidate = &pool->data[refs[i].index];
        if (candidate->charge_s < pareto_best_charge - SCORE_EPS) {
            candidate->flags |= CAND_PARETO;
            pareto_best_charge = candidate->charge_s;
            ++pareto_count;
        }
    }
    /*
     * 平滑权衡段上几乎每个点都可能属于 Pareto 前沿。逐点展开会退化为
     * 全区间穷举，因此均匀保留有限个前沿代表点；端点、极值、近优点和
     * 局部最优仍由各自独立标志完整保护。
    */
    if (pareto_count > MAX_PARETO_REFINE) {
        int ordinal = 0;
        for (int i = count - 1; i >= 0; --i) {
            TimeCandidate *candidate = &pool->data[refs[i].index];
            int keep = 0;
            if ((candidate->flags & CAND_PARETO) == 0u) continue;
            for (int slot = 0; slot < MAX_PARETO_REFINE; ++slot) {
                const int target = (slot * (pareto_count - 1) +
                                   (MAX_PARETO_REFINE - 1) / 2) /
                                   (MAX_PARETO_REFINE - 1);
                if (ordinal == target) {
                    keep = 1;
                    break;
                }
            }
            if (!keep) candidate->flags &= ~CAND_PARETO;
            ++ordinal;
        }
    }
    for (int i = 1; i + 1 < count; ++i) {
        TimeCandidate *candidate = &pool->data[refs[i].index];
        const TimeCandidate *left = &pool->data[refs[i - 1].index];
        const TimeCandidate *right = &pool->data[refs[i + 1].index];
        if ((candidate->charge_s <= left->charge_s + SCORE_EPS &&
             candidate->charge_s <= right->charge_s + SCORE_EPS) ||
            (candidate->charge_s >= left->charge_s - SCORE_EPS &&
             candidate->charge_s >= right->charge_s - SCORE_EPS))
            candidate->flags |= CAND_LOCAL_CHARGE;
        if (candidate->cost <= left->cost + SCORE_EPS &&
            candidate->cost <= right->cost + SCORE_EPS)
            candidate->flags |= CAND_LOCAL_SCORE;
    }
    free(refs);
    return best_index;
}

static void add_precise_range(const RouteContext *context, TimePool *pool,
                              int low, int high, int step_size,
                              TimeSearchLevel level)
{
    if (low > high) return;
    if (step_size < 1) step_size = 1;
    for (int step = low; step <= high; step += step_size)
        evaluate_final_candidate(context, step, level, pool);
    evaluate_final_candidate(context, high, level, pool);
}

static PreheatOptimizationResult empty_preheat_result(void)
{
    PreheatOptimizationResult result = {
        0, -1, NAN, NAN, NAN, NAN, NAN,
        NAN, NAN, NAN, NAN, 0
    };
    return result;
}

static PreheatOptimizationResult optimize_preheat(float T_init,
                                                  float SOC_init)
{
    PreheatOptimizationResult result = empty_preheat_result();
    RouteContext context;
    TimePool final_pool = {0};
    TimePool coarse_pool = {0};
    FeasibleTimeBounds bounds;
    TimeScale final_scale;
    TimeScale coarse_scale;
    int best_index;
    int coarse_step = (int)(COARSE_STEP_S / FINAL_DT_S + 0.5f);
    int fine_step = (int)(FINE_STEP_S / FINAL_DT_S + 0.5f);

    if (!build_route_context(&context, T_init, SOC_init)) return result;
    if (!time_pool_init(&final_pool, context.final.steps) ||
        !time_pool_init(&coarse_pool, context.final.steps)) {
        time_pool_free(&final_pool);
        time_pool_free(&coarse_pool);
        free_route_context(&context);
        return result;
    }
    if (!find_feasible_time_bounds(&context, &final_pool, &bounds))
        goto cleanup;

    evaluate_final_candidate(&context, bounds.earliest_start_step,
                             TIME_LEVEL_FINE, &final_pool);
    evaluate_final_candidate(&context, bounds.latest_start_step,
                             TIME_LEVEL_FINE, &final_pool);
    if (coarse_step < 1) coarse_step = 1;
    if (fine_step < 1) fine_step = 1;
    for (int step = bounds.earliest_start_step;
         step <= bounds.latest_start_step; step += coarse_step)
        evaluate_fast_candidate(&context, step, &coarse_pool);
    evaluate_fast_candidate(&context, bounds.latest_start_step, &coarse_pool);

    best_index = refresh_time_flags(&coarse_pool, &bounds, &coarse_scale);
    if (best_index < 0) {
        /* 1 s 近似可能抹掉极窄区间，此时粗点全部进入补搜。 */
        for (int i = 0; i < coarse_pool.count; ++i)
            coarse_pool.data[i].flags |= CAND_ENDPOINT;
    }

    /*
     * 1 s 粗搜只决定精确锚点的加入顺序。全部粗搜锚点随后都用 0.2 s
     * 重算并进入统一池，避免未入选粗点永远失去重新晋级机会。
     */
    for (int pass = 0; pass < 2; ++pass) {
        for (int i = 0; i < coarse_pool.count; ++i) {
            const TimeCandidate *coarse = &coarse_pool.data[i];
            const int initially_selected = coarse->flags != 0u;
            if ((pass == 0) != initially_selected) continue;
            evaluate_final_candidate(&context, coarse->start_step,
                                     TIME_LEVEL_COARSE, &final_pool);
        }
    }

    /*
     * 每轮只晋升一级：粗搜锚点先展开 5 s 细搜，细搜候选才能展开
     * 0.2 s 精搜。每次扩展后回到循环顶部统一重评分，旧候选可重新晋级。
     */
    for (;;) {
        int refine_index = -1;
        best_index = refresh_time_flags(&final_pool, &bounds, &final_scale);
        for (int i = 0; i < final_pool.count; ++i) {
            TimeCandidate *candidate = &final_pool.data[i];
            if ((candidate->level == TIME_LEVEL_COARSE ||
                 candidate->level == TIME_LEVEL_FINE) &&
                !candidate->refined_to_next && candidate->flags != 0u) {
                refine_index = i;
                break;
            }
        }
        if (refine_index < 0) break;
        {
            TimeCandidate *candidate = &final_pool.data[refine_index];
            const int center_step = candidate->start_step;
            const TimeSearchLevel level = candidate->level;
            const int radius = (level == TIME_LEVEL_COARSE)
                             ? (2 * coarse_step + 2) / 3 : fine_step;
            const int sample_step = (level == TIME_LEVEL_COARSE)
                                  ? fine_step : 1;
            const TimeSearchLevel next_level =
                (level == TIME_LEVEL_COARSE)
                ? TIME_LEVEL_FINE : TIME_LEVEL_FINAL;
            int low = center_step - radius;
            int high = center_step + radius;
            candidate->refined_to_next = 1;
            if (low < bounds.earliest_start_step) low = bounds.earliest_start_step;
            if (high > bounds.latest_start_step) high = bounds.latest_start_step;
            add_precise_range(&context, &final_pool, low, high,
                              sample_step, next_level);
            if (level == TIME_LEVEL_COARSE) {
                candidate = &final_pool.data[refine_index];
                candidate->level = TIME_LEVEL_FINE;
                candidate->refined_to_next = 0;
            }
        }
    }

    best_index = refresh_time_flags(&final_pool, &bounds, &final_scale);
    if (best_index >= 0 && final_scale.valid) {
        const TimeCandidate *best = &final_pool.data[best_index];
        result.feasible = 1;
        result.best_start_step = best->start_step;
        result.best_start_distance_km = best->start_distance_km;
        result.T_end_C = best->temp_C;
        result.SOC_end_pct = best->soc_pct;
        result.E_heat_kWh = best->heat_kWh;
        result.charge_time_s = best->charge_s;
        result.min_charge_s = final_scale.min_charge;
        result.max_charge_s = final_scale.max_charge;
        result.min_heat_kWh = bounds.min_heat_kWh;
        result.max_heat_kWh = bounds.max_heat_kWh;
        result.evaluated_final_candidates = final_pool.count;
    }

cleanup:
    time_pool_free(&coarse_pool);
    time_pool_free(&final_pool);
    free_route_context(&context);
    return result;
}

#ifdef PREHEAT_ENABLE_EXHAUSTIVE
static PreheatOptimizationResult optimize_preheat_exhaustive(
    float T_init, float SOC_init)
{
    PreheatOptimizationResult result = empty_preheat_result();
    RouteContext context;
    TimePool pool = {0};
    FeasibleTimeBounds bounds;
    TimeScale scale;
    int best_index;
    if (!build_route_context(&context, T_init, SOC_init)) return result;
    if (!time_pool_init(&pool, context.final.steps)) {
        free_route_context(&context);
        return result;
    }
    if (find_feasible_time_bounds(&context, &pool, &bounds)) {
        add_precise_range(&context, &pool,
            bounds.earliest_start_step, bounds.latest_start_step,
            1, TIME_LEVEL_FINAL);
        best_index = refresh_time_flags(&pool, &bounds, &scale);
        if (best_index >= 0 && scale.valid) {
            const TimeCandidate *best = &pool.data[best_index];
            result.feasible = 1;
            result.best_start_step = best->start_step;
            result.best_start_distance_km = best->start_distance_km;
            result.T_end_C = best->temp_C;
            result.SOC_end_pct = best->soc_pct;
            result.E_heat_kWh = best->heat_kWh;
            result.charge_time_s = best->charge_s;
            result.min_charge_s = scale.min_charge;
            result.max_charge_s = scale.max_charge;
            result.min_heat_kWh = bounds.min_heat_kWh;
            result.max_heat_kWh = bounds.max_heat_kWh;
            result.evaluated_final_candidates = pool.count;
        }
    }
    time_pool_free(&pool);
    free_route_context(&context);
    return result;
}
#endif

static float find_optimal_start_distance(
    float T_init, float SOC_init,
    float *T_end_opt, float *SOC_end_opt,
    float *E_heat_opt, float *chrg_time_s)
{
    const PreheatOptimizationResult result =
        optimize_preheat(T_init, SOC_init);
    *T_end_opt = result.T_end_C;
    *SOC_end_opt = result.SOC_end_pct;
    *E_heat_opt = result.E_heat_kWh;
    *chrg_time_s = result.charge_time_s;
    return result.best_start_distance_km;
}
/* --- Your algorithm code ends here --- */


/* ========================================================================
 *  Main entry point
 * ======================================================================== */
int main(void)
{
    /* --- Step 1: Read initial battery state --- */
    float T_init = 0.0f;
    EMS_HVBatt_getTempAvg(&T_init);

    float SOC_init = 0.0f;
    EMS_HVBatt_getTargetSOC(&SOC_init);
    SOC_init /= 100.0f;   /* convert % to 0~1 */

    float I_batt = 0.0f;
    EMS_HVBatt_getCurrent(&I_batt);

    float V_batt = 0.0f;
    EMS_HVBatt_getVolt(&V_batt);

    /* Load navigation data */
    load_nav_segments();

    /* Print initial state */
    printf("=== Initial State ===\n");
    printf("  Battery temp:     %.1f C\n", T_init);
    printf("  Battery SOC:      %.1f %%\n", SOC_init * 100.0f);
    printf("  Battery current:  %.1f A\n", I_batt);
    printf("  Battery voltage:  %.1f V\n", V_batt);
    printf("  Route segments:   %d\n", g_n_segs);

    printf("\n=== Route Segments ===\n");
    printf("  %-6s %-10s %-10s %-12s %-12s\n",
           "Seg", "Dist(km)", "Spd(km/h)", "Pdrive(kW)", "Tamb(C)");
    for (int i = 0; i < g_n_segs; i++) {
        printf("  %-6d %-10.3f %-10.3f %-12.3f %-12.3f\n",
               i + 1,
               g_segs[i].s_km,
               g_segs[i].v_kmh,
               g_segs[i].P_drive_kW,
               g_segs[i].T_env_C);
    }

    /* --- Step 2: Compute optimal preheat start distance --- */
    printf("\n=== Algorithm (implement your solution above) ===\n");

    /* TODO: Replace the placeholder values below with your algorithm's output.
     *
     * Required outputs:
     *   start_distance  — optimal preheat start distance, km from charger
     *   T_end_opt       — predicted battery temp at charger arrival, C
     *   SOC_end_opt     — predicted SOC at charger arrival, %
     *   E_heat_opt      — total preheat energy consumption, kWh
     *   chrg_time_s   — predicted charging time to 80% SOC, s
     *
     * Hard Constraints:
     *   T_end_opt in [20, 25] C
     *   SOC_end_opt >= 10 %
     *
     * Multi-Objective Optimization (DESIGN YOUR OWN STRATEGY):
     *   - Minimize E_heat_opt      (preheat energy)
     *   - Minimize chrg_time_s   (charging time to 80% SOC)
     *
     *   There is no single "correct" answer. You decide the tradeoff.
     *   Hint: start_distance controls both objectives.
     *         Preheating earlier => higher energy but potentially shorter charge time.
     *         Preheating later  => lower energy but longer charge time.
     *         Find your own optimal balance.
     *
     * Hints:
     *   1. Build a forward simulation using the physical model above.
     *   2. Locate the feasible heating-time bounds, then search only inside
     *      the corresponding 0.2 s start-step interval.
     *   3. chrg_time_s: estimate charging from SOC_end_opt to SOC_TARGET (80%),
     *      using P_CHARGE_MAP at the arrival endpoint (T_end, SOC_end).
     */
    float T_end_opt = T_init;
    float SOC_end_opt = SOC_init * 100.0f;
    float E_heat_opt = 0.0f;
    float chrg_time_s = 0.0f;
    float start_distance = find_optimal_start_distance(
        T_init, SOC_init, &T_end_opt, &SOC_end_opt, &E_heat_opt, &chrg_time_s);

    /* --- Step 3: Notify system of results --- */
    BattChrgPreHeatg_ntfPreHeatgStartDist(start_distance);
    BattChrgPreHeatg_ntfPreHeatgEndTemp(T_end_opt);
    BattChrgPreHeatg_ntfPreHeatgEndSOC(SOC_end_opt);
    BattChrgPreHeatg_ntfPreHeatgEnergy(E_heat_opt);

    /* --- Step 4: Verify results --- */
    printf("\n===== Result Verification =====\n");
    printf("  start_distance: notified=%.2f km\n", start_distance);
    printf("  T_end_opt:      notified=%.2f\n", T_end_opt);
    printf("  SOC_end_opt:    notified=%.2f\n", SOC_end_opt);
    printf("  E_heat_opt:     notified=%.4f\n", E_heat_opt);

    /* --- Step 5: Print results --- */
    printf("\n===== Results =====\n");
    printf("  start_distance = %.2f km  (begin preheat when %.2f km from charger)\n", start_distance, start_distance);
    printf("  T_end_opt      = %.2f C  [target: 20 ~ 25 C]\n", T_end_opt);
    printf("  SOC_end_opt    = %.2f %%  [min: 10 %%]\n", SOC_end_opt);
    printf("  E_heat_opt     = %.4f kWh  (%.0f kJ)\n", E_heat_opt, E_heat_opt * 3600.0f);
    printf("  chrg_time_s  = %.2f s  (to charge from %.1f%% to 80%%)\n", chrg_time_s, SOC_end_opt);

    /* --- Step 6: Constraint checks --- */
    printf("\n===== Constraint Checks =====\n");
    int pass = 1;
    if (T_end_opt >= T_OPT_LOW && T_end_opt <= T_OPT_HIGH) {
        printf("  [PASS] Temperature %.2f C in [%.0f, %.0f] C\n", T_end_opt, T_OPT_LOW, T_OPT_HIGH);
    } else {
        printf("  [FAIL] Temperature %.2f C NOT in [%.0f, %.0f] C\n", T_end_opt, T_OPT_LOW, T_OPT_HIGH);
        pass = 0;
    }
    if (SOC_end_opt >= SOC_MIN * 100.0f) {
        printf("  [PASS] SOC %.2f %% >= %.0f %%\n", SOC_end_opt, SOC_MIN * 100.0f);
    } else {
        printf("  [FAIL] SOC %.2f %% < %.0f %%\n", SOC_end_opt, SOC_MIN * 100.0f);
        pass = 0;
    }
    printf("  %s\n", pass ? "ALL CONSTRAINTS SATISFIED" : "SOME CONSTRAINTS VIOLATED");

    printf("\n===== Done =====\n");
    return 0;
}
