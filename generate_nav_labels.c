#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PREHEAT_ENABLE_EXHAUSTIVE 1
#define main student_solution_original_main
#include "student_solution.c"
#undef main

static float g_ntf_start_dist = 0.0f;
static float g_ntf_temp = 0.0f;
static float g_ntf_soc = 0.0f;
static float g_ntf_energy = 0.0f;

void EMS_HVBatt_getTempAvg(float *rty_getTempAvg) { *rty_getTempAvg = 0.0f; }
void EMS_HVBatt_getTargetSOC(float *rty_getTargetSOC) { *rty_getTargetSOC = 65.0f; }
void EMS_HVBatt_getCurrent(float *rty_getCurrent) { *rty_getCurrent = -39.7f; }
void EMS_HVBatt_getVolt(float *rty_getVolt) { *rty_getVolt = 377.5f; }
void TMS_EnvMonitor_getEnvTemp(float *rty_getEnvTemp) { *rty_getEnvTemp = -10.0f; }

void VehPwrPred_getPwrPred(PwrPredList_stru *rty_getPwrPred)
{
    rty_getPwrPred->num = (uint8_t)g_n_segs;
    for (int i = 0; i < g_n_segs; ++i) {
        rty_getPwrPred->pwrPredList[i].segId = (SegId_u16)(i + 1);
        rty_getPwrPred->pwrPredList[i].length = g_segs[i].s_km;
        rty_getPwrPred->pwrPredList[i].avgSpd = g_segs[i].v_kmh;
        rty_getPwrPred->pwrPredList[i].drvPwr = g_segs[i].P_drive_kW;
        rty_getPwrPred->pwrPredList[i].ambTemp = g_segs[i].T_env_C;
    }
}

void BattChrgPreHeatg_ntfPreHeatgStartDist(Dist_km_f32 StartDist_km)
{
    g_ntf_start_dist = StartDist_km;
}
void BattChrgPreHeatg_ntfPreHeatgEndTemp(float temp_C) { g_ntf_temp = temp_C; }
void BattChrgPreHeatg_ntfPreHeatgEndSOC(float soc_perc) { g_ntf_soc = soc_perc; }
void BattChrgPreHeatg_ntfPreHeatgEnergy(float energy_kWh)
{
    g_ntf_energy = energy_kWh;
}

void BattChrgPreHeatg_getPreHeatgStartDist(Dist_km_f32 *rty_getPreHeatgStartDist)
{
    *rty_getPreHeatgStartDist = g_ntf_start_dist;
}
void BattChrgPreHeatg_getPreHeatgEndTemp(float *rty_getPreHeatgEndTemp)
{
    *rty_getPreHeatgEndTemp = g_ntf_temp;
}
void BattChrgPreHeatg_getPreHeatgEndSOC(float *rty_getPreHeatgEndSOC)
{
    *rty_getPreHeatgEndSOC = g_ntf_soc;
}
void BattChrgPreHeatg_getPreHeatgEnergy(float *rty_getPreHeatgEnergy)
{
    *rty_getPreHeatgEnergy = g_ntf_energy;
}

void mock_setSimulationState(float T, float SOC, float time_s)
{
    (void)T;
    (void)SOC;
    (void)time_s;
}
void mock_resetSimulation(void) {}
void mock_advanceTime(float dt) { (void)dt; }
float mock_getTotalTime(void) { return 0.0f; }
void mock_getSegmentData(int segIdx, float *s_km, float *v_kmh,
                         float *P_kW, float *T_env)
{
    if (segIdx < 0 || segIdx >= g_n_segs) {
        *s_km = 0.0f;
        *v_kmh = 0.0f;
        *P_kW = 0.0f;
        *T_env = 0.0f;
        return;
    }
    *s_km = g_segs[segIdx].s_km;
    *v_kmh = g_segs[segIdx].v_kmh;
    *P_kW = g_segs[segIdx].P_drive_kW;
    *T_env = g_segs[segIdx].T_env_C;
}

static int parse_segment_line(const char *line, NavSeg *seg)
{
    int seg_id = 0;
    float s_km = 0.0f;
    float v_kmh = 0.0f;
    float p_kW = 0.0f;
    float t_env = 0.0f;

    if (sscanf(line, " %d %f %f %f %f",
               &seg_id, &s_km, &v_kmh, &p_kW, &t_env) != 5)
        return 0;
    if (seg_id <= 0 || s_km <= 0.0f || v_kmh <= 0.0f)
        return 0;

    seg->s_km = s_km;
    seg->v_kmh = v_kmh;
    seg->P_drive_kW = p_kW;
    seg->T_env_C = t_env;
    return 1;
}

typedef struct {
    int case_id;
    int seg_count;
    NavSeg segs[MAX_SEGS];
} SampleCase;

static unsigned int next_random(unsigned int *state)
{
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static void reservoir_store_case(SampleCase samples[], int sample_count,
                                 int seen_count, unsigned int *random_state,
                                 int case_id, const NavSeg segs[],
                                 int seg_count)
{
    int slot;

    if (seen_count <= sample_count) {
        slot = seen_count - 1;
    } else {
        const unsigned int pick = next_random(random_state) %
                                  (unsigned int)seen_count;
        if (pick >= (unsigned int)sample_count)
            return;
        slot = (int)pick;
    }

    samples[slot].case_id = case_id;
    samples[slot].seg_count = seg_count;
    for (int i = 0; i < seg_count; ++i)
        samples[slot].segs[i] = segs[i];
}

static int load_random_cases(const char *input_path, SampleCase samples[],
                             int sample_count, unsigned int seed)
{
    FILE *in = fopen(input_path, "rb");
    NavSeg current[MAX_SEGS];
    int current_count = 0;
    int seen_count = 0;
    int case_id = 0;
    char line[512];

    if (!in) {
        fprintf(stderr, "ERROR: cannot open input %s: %s\n",
                input_path, strerror(errno));
        return 0;
    }

    while (fgets(line, sizeof(line), in)) {
        NavSeg seg;
        if (parse_segment_line(line, &seg)) {
            if (current_count < MAX_SEGS)
                current[current_count++] = seg;
            continue;
        }
        if (current_count > 0) {
            ++case_id;
            ++seen_count;
            reservoir_store_case(samples, sample_count, seen_count, &seed,
                                 case_id, current, current_count);
            current_count = 0;
        }
    }
    if (current_count > 0) {
        ++case_id;
        ++seen_count;
        reservoir_store_case(samples, sample_count, seen_count, &seed,
                             case_id, current, current_count);
    }
    fclose(in);
    return (seen_count < sample_count) ? seen_count : sample_count;
}

static int close_enough(float a, float b, float tolerance)
{
    if (isnan(a) && isnan(b))
        return 1;
    return fabsf(a - b) <= tolerance;
}

static int verify_random_cases(const char *input_path,
                               int sample_count, unsigned int seed)
{
    const float T_INIT_C = 0.0f;
    const float SOC_INIT = 0.65f;
    SampleCase *samples;
    int loaded;
    int mismatches = 0;

    if (sample_count <= 0)
        return 1;
    samples = (SampleCase *)calloc((size_t)sample_count,
                                   sizeof(SampleCase));
    if (!samples) {
        fprintf(stderr, "ERROR: random sample allocation failed\n");
        return 1;
    }
    loaded = load_random_cases(input_path, samples, sample_count, seed);

    for (int sample = 0; sample < loaded; ++sample) {
        PreheatOptimizationResult optimized;
        PreheatOptimizationResult exhaustive;

        g_n_segs = samples[sample].seg_count;
        for (int seg = 0; seg < g_n_segs; ++seg)
            g_segs[seg] = samples[sample].segs[seg];
        optimized = optimize_preheat(T_INIT_C, SOC_INIT);
        exhaustive = optimize_preheat_exhaustive(T_INIT_C, SOC_INIT);

        if (optimized.feasible != exhaustive.feasible ||
            optimized.best_start_step != exhaustive.best_start_step ||
            !close_enough(optimized.min_charge_s,
                          exhaustive.min_charge_s, 1.0e-3f) ||
            !close_enough(optimized.max_charge_s,
                          exhaustive.max_charge_s, 1.0e-3f) ||
            !close_enough(optimized.min_heat_kWh,
                          exhaustive.min_heat_kWh, 1.0e-6f) ||
            !close_enough(optimized.max_heat_kWh,
                          exhaustive.max_heat_kWh, 1.0e-6f)) {
            ++mismatches;
            fprintf(stderr,
                    "MISMATCH case=%d opt_step=%d full_step=%d "
                    "opt_charge=[%.6f,%.6f] full_charge=[%.6f,%.6f]\n",
                    samples[sample].case_id,
                    optimized.best_start_step,
                    exhaustive.best_start_step,
                    optimized.min_charge_s,
                    optimized.max_charge_s,
                    exhaustive.min_charge_s,
                    exhaustive.max_charge_s);
        } else {
            fprintf(stderr, "PASS exhaustive case=%d step=%d\n",
                    samples[sample].case_id,
                    optimized.best_start_step);
        }
    }

    free(samples);
    g_n_segs = 0;
    fprintf(stderr, "verify-random: samples=%d mismatches=%d seed=%u\n",
            loaded, mismatches, seed);
    return mismatches == 0 ? 0 : 1;
}

static int flush_case(FILE *out, int case_id, int start_case, int max_cases,
                      int *written_count)
{
    const float T_INIT_C = 0.0f;
    const float SOC_INIT = 0.65f;
    PreheatOptimizationResult label;

    if (g_n_segs <= 0)
        return 1;
    if (case_id < start_case) {
        g_n_segs = 0;
        return 1;
    }
    if (max_cases > 0 && *written_count >= max_cases)
        return 0;

    label = optimize_preheat(T_INIT_C, SOC_INIT);
    if (fprintf(out, "%d,%.3f,%.6f,%.6f,%.6f,%.6f\n",
                case_id,
                label.best_start_distance_km,
                label.min_charge_s,
                label.max_charge_s,
                label.min_heat_kWh,
                label.max_heat_kWh) < 0) {
        fprintf(stderr, "ERROR: failed writing case %d: %s\n",
                case_id, strerror(errno));
        g_n_segs = 0;
        return -1;
    }

    ++(*written_count);
    if ((*written_count % 1000) == 0)
        fprintf(stderr, "processed %d cases\n", *written_count);

    g_n_segs = 0;
    return 1;
}

int main(int argc, char **argv)
{
    const char *input_path = "nav_train_100000.txt";
    const char *output_path = "nav_train_100000_labels.csv";
    int max_cases = 0;
    int start_case = 1;
    FILE *in = NULL;
    FILE *out = NULL;
    char line[512];
    int case_id = 0;
    int written_count = 0;
    int write_failed = 0;

    if (argc >= 2 && strcmp(argv[1], "--verify-random") == 0) {
        const char *verify_input =
            (argc >= 3) ? argv[2] : input_path;
        const int verify_count =
            (argc >= 4) ? atoi(argv[3]) : 10;
        const unsigned int verify_seed =
            (argc >= 5) ? (unsigned int)strtoul(argv[4], NULL, 10) : 20260819u;
        return verify_random_cases(verify_input, verify_count, verify_seed);
    }

    if (argc >= 2)
        input_path = argv[1];
    if (argc >= 3)
        output_path = argv[2];
    if (argc >= 4)
        max_cases = atoi(argv[3]);
    if (argc >= 5)
        start_case = atoi(argv[4]);
    if (start_case <= 0)
        start_case = 1;

    in = fopen(input_path, "rb");
    if (!in) {
        fprintf(stderr, "ERROR: cannot open input %s: %s\n",
                input_path, strerror(errno));
        return 1;
    }

    out = fopen(output_path, "wb");
    if (!out) {
        fprintf(stderr, "ERROR: cannot open output %s: %s\n",
                output_path, strerror(errno));
        fclose(in);
        return 1;
    }

    if (fprintf(out,
        "case_id,best_start_distance_km,min_charge_s,max_charge_s,"
        "min_heat_kWh,max_heat_kWh\n") < 0) {
        fprintf(stderr, "ERROR: failed writing CSV header: %s\n",
                strerror(errno));
        fclose(out);
        fclose(in);
        return 1;
    }

    while (fgets(line, sizeof(line), in)) {
        NavSeg seg;

        if (parse_segment_line(line, &seg)) {
            if (g_n_segs < MAX_SEGS)
                g_segs[g_n_segs++] = seg;
            continue;
        }

        if (g_n_segs > 0) {
            int status;
            ++case_id;
            status = flush_case(out, case_id, start_case, max_cases,
                                &written_count);
            if (status < 0) write_failed = 1;
            if (status <= 0)
                break;
        }
    }

    if (g_n_segs > 0) {
        int status;
        ++case_id;
        status = flush_case(out, case_id, start_case, max_cases,
                            &written_count);
        if (status < 0) write_failed = 1;
    }

    if (fclose(out) != 0) {
        fprintf(stderr, "ERROR: failed closing output %s: %s\n",
                output_path, strerror(errno));
        write_failed = 1;
    }
    fclose(in);

    if (write_failed) return 1;
    fprintf(stderr, "done: %d cases -> %s\n", written_count, output_path);
    return 0;
}
