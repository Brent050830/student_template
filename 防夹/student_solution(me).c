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

typedef struct {
    float temp_C;
    float soc;
    float heat_kWh;
} RouteResult;

static float interp_map(const float temp_axis[7], const float soc_axis[8],
                        const float map[7][8], float temp_C, float soc_pct)
{
    const float temp = fminf(temp_axis[6], fmaxf(temp_axis[0], temp_C));
    const float soc = fminf(soc_axis[7], fmaxf(soc_axis[0], soc_pct));
    int ti = 0;
    int si = 0;

    while (ti < 5 && temp > temp_axis[ti + 1]) ++ti;
    while (si < 6 && soc > soc_axis[si + 1]) ++si;

    const float tf = (temp - temp_axis[ti]) /
                     (temp_axis[ti + 1] - temp_axis[ti]);
    const float sf = (soc - soc_axis[si]) /
                     (soc_axis[si + 1] - soc_axis[si]);
    const float lower = map[ti][si] + sf * (map[ti][si + 1] - map[ti][si]);
    const float upper = map[ti + 1][si] +
                        sf * (map[ti + 1][si + 1] - map[ti + 1][si]);

    return lower + tf * (upper - lower);
}

static RouteResult simulate_route(float T_init, float SOC_init,
                                  float route_km, float start_distance)
{
    RouteResult result = {T_init, SOC_init, 0.0f};
    float remaining_km = route_km;
    float heat_seconds = 0.0f;

    for (int seg = 0; seg < g_n_segs; ++seg) {
        const float speed = g_segs[seg].v_kmh;
        if (speed <= 0.0f)
            continue;

        const float seg_seconds = 3600.0f * g_segs[seg].s_km / speed;
        float elapsed = 0.0f;
        while (elapsed < seg_seconds) {
            const float dt = fminf(1.0f, seg_seconds - elapsed);
            const int heating = remaining_km <= start_distance + 1.0e-6f;
            const float soc_pct = result.soc * 100.0f;
            const float resistance = interp_map(R_INT_TEMPS, R_INT_SOCS,
                                                R_INT_MAP, result.temp_C, soc_pct);
            const float uoc = interp_map(U_OC_TEMPS, U_OC_SOCS,
                                         UOC_MAP, result.temp_C, soc_pct);
            const float power_w = g_segs[seg].P_drive_kW * 1000.0f +
                                  (heating ? P_HEAT_MAX : 0.0f);
            float discriminant = uoc * uoc - 4.0f * resistance * power_w;
            if (discriminant < 0.0f) discriminant = 0.0f;

            const float current = (fabsf(resistance) > 1.0e-8f)
                                ? (uoc - sqrtf(discriminant)) / (2.0f * resistance)
                                : power_w / fmaxf(uoc, 1.0f);
            const float generated_heat = current * current * resistance;
            const float heat_loss = (H0 + KV * speed) *
                                    (result.temp_C - g_segs[seg].T_env_C);

            result.temp_C += (generated_heat +
                              (heating ? ETA_HEAT * P_HEAT_MAX : 0.0f) - heat_loss) *
                             dt / (M_BAT * CP);
            result.soc -= current * dt / (C_NOM_AH * 3600.0f);
            if (heating) heat_seconds += dt;
            remaining_km = fmaxf(0.0f, remaining_km - speed * dt / 3600.0f);
            elapsed += dt;
        }
    }

    result.heat_kWh = P_HEAT_MAX * heat_seconds / 3600000.0f;
    return result;
}

static float estimate_charge_time(float temp_C, float start_soc)
{
    const float charge_kw = fmaxf(0.1f,
        interp_map(P_CHARGE_TEMPS, P_CHARGE_SOCS, P_CHARGE_MAP,
                   temp_C, start_soc * 100.0f));

    if (start_soc >= SOC_TARGET)
        return 0.0f;

    return (SOC_TARGET - start_soc) * C_NOM_AH * U_NOM * 3.6f / charge_kw;
}

typedef enum {
    SEARCH_LEVEL_COARSE = 0,
    SEARCH_LEVEL_FINE   = 1,
    SEARCH_LEVEL_FINAL  = 2
} SearchLevel;

typedef enum {
    ADD_FULL      = -1,
    ADD_DUPLICATE = 0,
    ADD_NEW       = 1
} AddResult;

typedef struct {
    float dist_km;
    float temp_C;
    float soc_pct;
    float heat_kWh;
    float charge_s;
    float cost;
    int feasible;
    SearchLevel level;
    int refined_to_next;
    int pareto;
    int local_min;
} SearchCandidate;

typedef struct {
    float min_energy;
    float max_energy;
    float min_charge;
    float max_charge;
    int valid;
} SearchScale;

#define REFINE_COST_MARGIN 0.03f
#define MAX_COVERED_INTERVALS 64
#define MAX_SEARCH_CANDIDATES 20000

typedef struct {
    SearchCandidate data[MAX_SEARCH_CANDIDATES];
    int count;
} SearchPool;

typedef struct {
    float low_km;
    float high_km;
} SearchInterval;

typedef struct {
    float charge_s;
    float heat_kWh;
    int index;
} ParetoItem;

typedef struct {
    float dist_km;
    int index;
} CandidateRef;

#define DIST_KEY_EPS_KM 1.0e-6f

static SearchCandidate evaluate_start_distance(float T_init, float SOC_init,
                                               float route_km, float dist_km)
{
    SearchCandidate cand;
    const RouteResult route = simulate_route(T_init, SOC_init, route_km, dist_km);

    cand.dist_km = dist_km;
    cand.temp_C = route.temp_C;
    cand.soc_pct = route.soc * 100.0f;
    cand.heat_kWh = route.heat_kWh;
    cand.charge_s = estimate_charge_time(route.temp_C, route.soc);
    cand.feasible = route.temp_C >= T_OPT_LOW &&
                    route.temp_C <= T_OPT_HIGH &&
                    route.soc >= SOC_MIN;
    cand.cost = INFINITY;
    cand.level = SEARCH_LEVEL_COARSE;
    cand.refined_to_next = 0;
    cand.pareto = 0;
    cand.local_min = 0;

    return cand;
}

static void search_scale_init(SearchScale *scale)
{
    scale->min_energy = INFINITY;
    scale->max_energy = -INFINITY;
    scale->min_charge = INFINITY;
    scale->max_charge = -INFINITY;
    scale->valid = 0;
}

static void search_scale_add(SearchScale *scale, const SearchCandidate *cand)
{
    if (!cand->feasible)
        return;

    scale->min_energy = fminf(scale->min_energy, cand->heat_kWh);
    scale->max_energy = fmaxf(scale->max_energy, cand->heat_kWh);
    scale->min_charge = fminf(scale->min_charge, cand->charge_s);
    scale->max_charge = fmaxf(scale->max_charge, cand->charge_s);
    scale->valid = 1;
}

static float normalized_candidate_cost(const SearchScale *scale,
                                       const SearchCandidate *cand)
{
    const float energy_cost = (scale->max_energy > scale->min_energy)
                            ? (cand->heat_kWh - scale->min_energy) /
                              (scale->max_energy - scale->min_energy)
                            : 0.0f;
    const float charge_cost = (scale->max_charge > scale->min_charge)
                            ? (cand->charge_s - scale->min_charge) /
                              (scale->max_charge - scale->min_charge)
                            : 0.0f;

    return 0.4f * energy_cost + 0.6f * charge_cost;
}

static int compare_interval_low(const void *lhs, const void *rhs)
{
    const SearchInterval *a = (const SearchInterval *)lhs;
    const SearchInterval *b = (const SearchInterval *)rhs;

    if (a->low_km < b->low_km)
        return -1;
    if (a->low_km > b->low_km)
        return 1;
    if (a->high_km < b->high_km)
        return -1;
    if (a->high_km > b->high_km)
        return 1;
    return 0;
}

static int compare_pareto_item(const void *lhs, const void *rhs)
{
    const ParetoItem *a = (const ParetoItem *)lhs;
    const ParetoItem *b = (const ParetoItem *)rhs;

    if (a->charge_s < b->charge_s)
        return -1;
    if (a->charge_s > b->charge_s)
        return 1;
    if (a->heat_kWh < b->heat_kWh)
        return -1;
    if (a->heat_kWh > b->heat_kWh)
        return 1;
    return 0;
}

static int compare_candidate_ref(const void *lhs, const void *rhs)
{
    const CandidateRef *a = (const CandidateRef *)lhs;
    const CandidateRef *b = (const CandidateRef *)rhs;

    if (a->dist_km < b->dist_km)
        return -1;
    if (a->dist_km > b->dist_km)
        return 1;
    return 0;
}

static void search_pool_init(SearchPool *pool)
{
    pool->count = 0;
}

static int search_pool_find_index(const SearchPool *pool, float dist_km)
{
    for (int i = 0; i < pool->count; ++i) {
        if (fabsf(pool->data[i].dist_km - dist_km) < DIST_KEY_EPS_KM)
            return i;
    }

    return -1;
}

static AddResult search_pool_add(SearchPool *pool, const SearchCandidate *cand)
{
    for (int i = 0; i < pool->count; ++i) {
        if (fabsf(pool->data[i].dist_km - cand->dist_km) < DIST_KEY_EPS_KM) {
            if (cand->feasible && !pool->data[i].feasible) {
                const SearchLevel level = pool->data[i].level;
                const int refined_to_next = pool->data[i].refined_to_next;
                const int pareto = pool->data[i].pareto;
                const int local_min = pool->data[i].local_min;
                pool->data[i] = *cand;
                pool->data[i].level = level;
                pool->data[i].refined_to_next = refined_to_next;
                pool->data[i].pareto = pareto;
                pool->data[i].local_min = local_min;
            }
            return ADD_DUPLICATE;
        }
    }

    if (pool->count < MAX_SEARCH_CANDIDATES) {
        pool->data[pool->count++] = *cand;
        return ADD_NEW;
    }

    return ADD_FULL;
}

static void search_scale_from_pool(SearchScale *scale, const SearchPool *pool)
{
    search_scale_init(scale);
    for (int i = 0; i < pool->count; ++i)
        search_scale_add(scale, &pool->data[i]);
}

static int find_candidate_ref(const CandidateRef refs[], int count, float dist_km)
{
    int low = 0;
    int high = count - 1;

    while (low <= high) {
        const int mid = low + (high - low) / 2;
        const float delta = refs[mid].dist_km - dist_km;

        if (fabsf(delta) < DIST_KEY_EPS_KM)
            return refs[mid].index;
        if (delta < 0.0f)
            low = mid + 1;
        else
            high = mid - 1;
    }

    return -1;
}

static void compute_pareto_flags(SearchPool *pool)
{
    const float eps = 1.0e-6f;
    int item_count = 0;
    ParetoItem *items = NULL;

    for (int i = 0; i < pool->count; ++i) {
        pool->data[i].pareto = 0;
        if (pool->data[i].feasible)
            ++item_count;
    }

    if (item_count == 0)
        return;

    items = (ParetoItem *)malloc(sizeof(ParetoItem) * (size_t)item_count);
    if (!items) {
        fprintf(stderr, "ERROR: pareto allocation failed\n");
        for (int i = 0; i < pool->count; ++i)
            pool->data[i].pareto = pool->data[i].feasible;
        return;
    }

    item_count = 0;
    for (int i = 0; i < pool->count; ++i) {
        if (!pool->data[i].feasible)
            continue;
        items[item_count].charge_s = pool->data[i].charge_s;
        items[item_count].heat_kWh = pool->data[i].heat_kWh;
        items[item_count].index = i;
        ++item_count;
    }

    qsort(items, (size_t)item_count, sizeof(ParetoItem), compare_pareto_item);

    {
        float best_energy_before_group = INFINITY;
        int start = 0;

        while (start < item_count) {
            int end = start + 1;
            float group_min_energy = items[start].heat_kWh;
            const float group_charge = items[start].charge_s;

            while (end < item_count &&
                   fabsf(items[end].charge_s - group_charge) <= eps) {
                group_min_energy = fminf(group_min_energy,
                                         items[end].heat_kWh);
                ++end;
            }

            for (int i = start; i < end; ++i) {
                const int dominated_by_previous =
                    best_energy_before_group <= items[i].heat_kWh + eps;
                const int dominated_in_group =
                    items[i].heat_kWh > group_min_energy + eps;

                pool->data[items[i].index].pareto =
                    !(dominated_by_previous || dominated_in_group);
            }

            best_energy_before_group =
                fminf(best_energy_before_group, group_min_energy);
            start = end;
        }
    }

    free(items);
}

static void score_pool_with_scale(SearchPool *pool, const SearchScale *scale)
{
    for (int i = 0; i < pool->count; ++i) {
        SearchCandidate *cand = &pool->data[i];
        cand->cost = (scale->valid && cand->feasible)
                   ? normalized_candidate_cost(scale, cand)
                   : INFINITY;
    }
}

static void score_pool_against(SearchPool *pool, const SearchPool *reference)
{
    SearchScale scale;
    CandidateRef *refs = NULL;

    search_scale_from_pool(&scale, reference);
    score_pool_with_scale(pool, &scale);

    if (reference->count <= 0)
        return;

    refs = (CandidateRef *)malloc(sizeof(CandidateRef) *
                                  (size_t)reference->count);
    if (!refs) {
        fprintf(stderr, "ERROR: candidate reference allocation failed\n");
        for (int i = 0; i < pool->count; ++i)
            pool->data[i].pareto = pool->data[i].feasible;
        return;
    }

    for (int i = 0; i < reference->count; ++i) {
        refs[i].dist_km = reference->data[i].dist_km;
        refs[i].index = i;
    }
    qsort(refs, (size_t)reference->count, sizeof(CandidateRef),
          compare_candidate_ref);

    for (int i = 0; i < pool->count; ++i) {
        const int ref_index = find_candidate_ref(refs, reference->count,
                                                 pool->data[i].dist_km);
        pool->data[i].pareto =
            (ref_index >= 0) ? reference->data[ref_index].pareto : 0;
    }

    free(refs);
}

static int candidate_is_better(const SearchCandidate *cand,
                               const SearchCandidate *best,
                               int has_best)
{
    if (!has_best)
        return 1;
    if (cand->cost < best->cost - 1.0e-6f)
        return 1;
    if (fabsf(cand->cost - best->cost) <= 1.0e-6f &&
        cand->heat_kWh < best->heat_kWh)
        return 1;
    return 0;
}

static int best_from_pool(SearchCandidate *best, SearchPool *pool)
{
    SearchScale scale;
    int has_best = 0;

    search_scale_from_pool(&scale, pool);
    if (!scale.valid)
        return 0;

    score_pool_with_scale(pool, &scale);
    compute_pareto_flags(pool);

    for (int i = 0; i < pool->count; ++i) {
        SearchCandidate *cand = &pool->data[i];
        if (!cand->feasible)
            continue;

        if (candidate_is_better(cand, best, has_best)) {
            *best = *cand;
            has_best = 1;
        }
    }

    return has_best;
}

static int add_interval(SearchInterval intervals[], int *count, int max_count,
                        float low_km, float high_km, float route_km)
{
    const float eps = 1.0e-4f;

    low_km = fmaxf(0.0f, low_km);
    high_km = fminf(route_km, high_km);
    if (high_km < low_km)
        return 0;

    for (int i = 0; i < *count; ++i) {
        if (low_km <= intervals[i].high_km + eps &&
            high_km + eps >= intervals[i].low_km) {
            intervals[i].low_km = fminf(intervals[i].low_km, low_km);
            intervals[i].high_km = fmaxf(intervals[i].high_km, high_km);

            for (int j = i + 1; j < *count;) {
                if (intervals[i].low_km <= intervals[j].high_km + eps &&
                    intervals[i].high_km + eps >= intervals[j].low_km) {
                    intervals[i].low_km = fminf(intervals[i].low_km,
                                                intervals[j].low_km);
                    intervals[i].high_km = fmaxf(intervals[i].high_km,
                                                 intervals[j].high_km);
                    for (int k = j; k < *count - 1; ++k)
                        intervals[k] = intervals[k + 1];
                    --(*count);
                } else {
                    ++j;
                }
            }
            return 1;
        }
    }

    if (*count >= max_count)
        return 0;

    intervals[*count].low_km = low_km;
    intervals[*count].high_km = high_km;
    ++(*count);
    return 1;
}

static int interval_is_covered(const SearchInterval intervals[], int count,
                               float low_km, float high_km, float route_km)
{
    const float eps = 1.0e-4f;

    low_km = fmaxf(0.0f, low_km);
    high_km = fminf(route_km, high_km);

    for (int i = 0; i < count; ++i) {
        if (intervals[i].low_km <= low_km + eps &&
            intervals[i].high_km + eps >= high_km)
            return 1;
    }

    return 0;
}

static void compute_local_min_flags(SearchPool *pool)
{
    const float eps = 1.0e-6f;
    CandidateRef *refs = NULL;
    int ref_count = 0;

    for (int i = 0; i < pool->count; ++i) {
        pool->data[i].local_min = 0;
        if (pool->data[i].feasible && pool->data[i].cost < INFINITY)
            ++ref_count;
    }

    if (ref_count <= 1)
        return;

    refs = (CandidateRef *)malloc(sizeof(CandidateRef) * (size_t)ref_count);
    if (!refs) {
        fprintf(stderr, "ERROR: local minimum allocation failed\n");
        return;
    }

    ref_count = 0;
    for (int i = 0; i < pool->count; ++i) {
        if (!pool->data[i].feasible || pool->data[i].cost >= INFINITY)
            continue;
        refs[ref_count].dist_km = pool->data[i].dist_km;
        refs[ref_count].index = i;
        ++ref_count;
    }

    qsort(refs, (size_t)ref_count, sizeof(CandidateRef),
          compare_candidate_ref);

    for (int i = 0; i < ref_count; ++i) {
        const int idx = refs[i].index;
        const SearchCandidate *cand = &pool->data[idx];
        int is_local_min = 0;

        if (i == 0) {
            const SearchCandidate *right = &pool->data[refs[i + 1].index];
            is_local_min = cand->cost < right->cost - eps;
        } else if (i == ref_count - 1) {
            const SearchCandidate *left = &pool->data[refs[i - 1].index];
            is_local_min = cand->cost < left->cost - eps;
        } else {
            const SearchCandidate *left = &pool->data[refs[i - 1].index];
            const SearchCandidate *right = &pool->data[refs[i + 1].index];
            is_local_min =
                cand->cost <= left->cost + eps &&
                cand->cost <= right->cost + eps &&
                (cand->cost < left->cost - eps ||
                 cand->cost < right->cost - eps);
        }

        pool->data[idx].local_min = is_local_min;
    }

    free(refs);
}

static int candidate_is_promising(const SearchCandidate *cand,
                                  float best_cost, float min_energy,
                                  float min_charge)
{
    if (!cand->feasible || cand->cost >= INFINITY)
        return 0;

    return cand->cost <= best_cost + REFINE_COST_MARGIN ||
           cand->heat_kWh <= min_energy + 1.0e-6f ||
           cand->charge_s <= min_charge + 1.0e-6f ||
           cand->pareto;
}

static void get_pool_extremes(const SearchPool *pool,
                              float *min_energy, float *min_charge)
{
    *min_energy = INFINITY;
    *min_charge = INFINITY;

    for (int i = 0; i < pool->count; ++i) {
        const SearchCandidate *cand = &pool->data[i];
        if (!cand->feasible)
            continue;
        *min_energy = fminf(*min_energy, cand->heat_kWh);
        *min_charge = fminf(*min_charge, cand->charge_s);
    }
}

static int candidate_requests_refine(const SearchCandidate *cand,
                                     float best_cost, float min_energy,
                                     float min_charge)
{
    return candidate_is_promising(cand, best_cost, min_energy, min_charge) ||
           cand->local_min;
}

static void mark_refined_candidates_for_covered(SearchPool *level_pool,
                                                const SearchPool *all_pool,
                                                float best_cost,
                                                float radius_km,
                                                const SearchInterval covered[],
                                                int covered_count,
                                                float route_km)
{
    float min_energy = INFINITY;
    float min_charge = INFINITY;

    get_pool_extremes(all_pool, &min_energy, &min_charge);

    for (int i = 0; i < level_pool->count; ++i) {
        SearchCandidate *cand = &level_pool->data[i];
        float low = 0.0f;
        float high = 0.0f;

        if (cand->refined_to_next)
            continue;
        if (!candidate_requests_refine(cand, best_cost, min_energy,
                                       min_charge))
            continue;

        low = fmaxf(0.0f, cand->dist_km - radius_km);
        high = fminf(route_km, cand->dist_km + radius_km);
        if (interval_is_covered(covered, covered_count, low, high, route_km))
            cand->refined_to_next = 1;
    }
}

static int build_level_refine_intervals(SearchInterval **out_intervals,
                                        SearchPool *level_pool,
                                        const SearchPool *all_pool,
                                        float best_cost, float radius_km,
                                        const SearchInterval covered[],
                                        int covered_count,
                                        float route_km)
{
    const float eps = 1.0e-4f;
    SearchInterval *raw_intervals = NULL;
    SearchInterval *merged_intervals = NULL;
    int raw_count = 0;
    int merged_count = 0;
    float min_energy = INFINITY;
    float min_charge = INFINITY;

    *out_intervals = NULL;
    if (level_pool->count <= 0)
        return 0;

    raw_intervals = (SearchInterval *)malloc(sizeof(SearchInterval) *
                                             (size_t)level_pool->count);
    merged_intervals = (SearchInterval *)malloc(sizeof(SearchInterval) *
                                                (size_t)level_pool->count);
    if (!raw_intervals || !merged_intervals) {
        fprintf(stderr, "ERROR: interval allocation failed\n");
        free(raw_intervals);
        free(merged_intervals);
        return 0;
    }

    get_pool_extremes(all_pool, &min_energy, &min_charge);

    for (int i = 0; i < level_pool->count; ++i) {
        SearchCandidate *cand = &level_pool->data[i];
        int promising = 0;
        float low = 0.0f;
        float high = 0.0f;

        if (cand->refined_to_next)
            continue;

        promising = candidate_requests_refine(cand, best_cost, min_energy,
                                              min_charge);
        if (!promising)
            continue;

        low = fmaxf(0.0f, cand->dist_km - radius_km);
        high = fminf(route_km, cand->dist_km + radius_km);
        if (interval_is_covered(covered, covered_count, low, high, route_km))
            continue;

        raw_intervals[raw_count].low_km = low;
        raw_intervals[raw_count].high_km = high;
        ++raw_count;
    }

    if (raw_count == 0) {
        free(raw_intervals);
        free(merged_intervals);
        return 0;
    }

    qsort(raw_intervals, (size_t)raw_count, sizeof(SearchInterval),
          compare_interval_low);

    for (int i = 0; i < raw_count; ++i) {
        if (merged_count == 0 ||
            raw_intervals[i].low_km >
            merged_intervals[merged_count - 1].high_km + eps) {
            merged_intervals[merged_count++] = raw_intervals[i];
        } else {
            merged_intervals[merged_count - 1].high_km =
                fmaxf(merged_intervals[merged_count - 1].high_km,
                      raw_intervals[i].high_km);
        }
    }

    free(raw_intervals);
    *out_intervals = merged_intervals;
    return merged_count;
}

static void remember_fallback(SearchCandidate *fallback, int *has_fallback,
                              const SearchCandidate *cand)
{
    if (cand->soc_pct >= SOC_MIN * 100.0f &&
        (!*has_fallback ||
         fabsf(cand->temp_C - T_OPT_LOW) < fabsf(fallback->temp_C - T_OPT_LOW))) {
        *fallback = *cand;
        *has_fallback = 1;
    }
}

static AddResult store_candidate(float T_init, float SOC_init, float route_km,
                                  float dist_km, SearchPool *layer_pool,
                                  SearchPool *all_pool,
                                  SearchCandidate *fallback,
                                  int *has_fallback, SearchLevel level)
{
    SearchCandidate cand;
    const float clipped_dist = fminf(fmaxf(dist_km, 0.0f), route_km);
    const int existing = search_pool_find_index(all_pool, clipped_dist);
    AddResult result = ADD_DUPLICATE;

    if (existing >= 0) {
        cand = all_pool->data[existing];
        cand.level = level;
        cand.refined_to_next = 0;
        if (layer_pool) {
            result = search_pool_add(layer_pool, &cand);
            if (result == ADD_FULL)
                fprintf(stderr, "ERROR: layer candidate pool full\n");
        }
        return result;
    }

    cand = evaluate_start_distance(T_init, SOC_init, route_km, clipped_dist);
    cand.level = level;
    cand.refined_to_next = 0;

    result = search_pool_add(all_pool, &cand);
    if (result == ADD_FULL) {
        fprintf(stderr, "ERROR: all candidate pool full\n");
        return ADD_FULL;
    }

    if (layer_pool) {
        const AddResult layer_result = search_pool_add(layer_pool, &cand);
        if (layer_result == ADD_FULL) {
            fprintf(stderr, "ERROR: layer candidate pool full\n");
            return ADD_FULL;
        }
    }

    remember_fallback(fallback, has_fallback, &cand);
    return result;
}

static AddResult sample_intervals(float T_init, float SOC_init, float route_km,
                                  const SearchInterval intervals[],
                                  int interval_count, float step_km,
                                  SearchPool *layer_pool, SearchPool *all_pool,
                                  SearchCandidate *fallback, int *has_fallback,
                                  SearchLevel level)
{
    int added = 0;

    for (int i = 0; i < interval_count; ++i) {
        const float low = intervals[i].low_km;
        const float high = intervals[i].high_km;

        for (float dist = low; dist <= high + 1.0e-6f; dist += step_km) {
            const AddResult result =
                store_candidate(T_init, SOC_init, route_km, dist,
                                layer_pool, all_pool, fallback,
                                has_fallback, level);
            if (result == ADD_FULL)
                return ADD_FULL;
            if (result == ADD_NEW)
                ++added;
        }

        {
            const AddResult result =
                store_candidate(T_init, SOC_init, route_km, high, layer_pool,
                                all_pool, fallback, has_fallback, level);
            if (result == ADD_FULL)
                return ADD_FULL;
            if (result == ADD_NEW)
                ++added;
        }
    }

    return added > 0 ? ADD_NEW : ADD_DUPLICATE;
}

static int mark_intervals_covered(SearchInterval covered[],
                                  int *covered_count, int max_count,
                                  const SearchInterval intervals[],
                                  int interval_count, float route_km)
{
    for (int i = 0; i < interval_count; ++i) {
        if (!add_interval(covered, covered_count, max_count,
                          intervals[i].low_km, intervals[i].high_km,
                          route_km)) {
            fprintf(stderr, "ERROR: covered interval capacity exceeded\n");
            return 0;
        }
    }

    return 1;
}

static float find_optimal_start_distance(float T_init, float SOC_init,
                                         float *T_end_opt, float *SOC_end_opt,
                                         float *E_heat_opt, float *chrg_time_s)
{
    const float COARSE_STEP_KM = 1.0f;
    const float FINE_STEP_KM = 0.05f;
    const float FINAL_STEP_KM = 0.005f;
    float route_km = 0.0f;
    static SearchPool all_pool;
    static SearchPool coarse_pool;
    static SearchPool fine_pool;
    static SearchPool final_pool;
    SearchCandidate best;
    SearchCandidate fallback;
    SearchInterval fine_covered[MAX_COVERED_INTERVALS];
    SearchInterval final_covered[MAX_COVERED_INTERVALS];
    int fine_covered_count = 0;
    int final_covered_count = 0;
    int has_fallback = 0;
    int fallback_seeded = 0;
    int search_error = 0;

    for (int seg = 0; seg < g_n_segs; ++seg)
        route_km += g_segs[seg].s_km;

    search_pool_init(&all_pool);
    search_pool_init(&coarse_pool);
    search_pool_init(&fine_pool);
    search_pool_init(&final_pool);

    /* Coarse candidates: full-route coverage. */
    for (float dist = 0.0f; dist <= route_km + 1.0e-6f; dist += COARSE_STEP_KM) {
        if (store_candidate(T_init, SOC_init, route_km, dist, &coarse_pool,
                            &all_pool, &fallback, &has_fallback,
                            SEARCH_LEVEL_COARSE) == ADD_FULL) {
            search_error = 1;
            break;
        }
    }

    if (!search_error &&
        store_candidate(T_init, SOC_init, route_km, route_km, &coarse_pool,
                        &all_pool, &fallback, &has_fallback,
                        SEARCH_LEVEL_COARSE) == ADD_FULL)
        search_error = 1;

    while (!search_error) {
        AddResult sample_result = ADD_DUPLICATE;
        int refine_count = 0;
        SearchInterval *refine_intervals = NULL;

        /* Every pass starts from a global rescore over all known candidates. */
        if (!best_from_pool(&best, &all_pool)) {
            if (!fallback_seeded && has_fallback) {
                SearchInterval fallback_interval;
                fallback_interval.low_km =
                    fmaxf(0.0f, fallback.dist_km - COARSE_STEP_KM);
                fallback_interval.high_km =
                    fminf(route_km, fallback.dist_km + COARSE_STEP_KM);

                sample_result =
                    sample_intervals(T_init, SOC_init, route_km,
                                     &fallback_interval, 1, FINE_STEP_KM,
                                     &fine_pool, &all_pool,
                                     &fallback, &has_fallback,
                                     SEARCH_LEVEL_FINE);
                fallback_seeded = 1;
                if (sample_result == ADD_FULL) {
                    search_error = 1;
                    break;
                }
                if (!mark_intervals_covered(fine_covered,
                                            &fine_covered_count,
                                            MAX_COVERED_INTERVALS,
                                            &fallback_interval, 1,
                                            route_km)) {
                    search_error = 1;
                    break;
                }
                continue;
            }

            fprintf(stderr, "ERROR: no feasible search start\n");
            break;
        }
        score_pool_against(&coarse_pool, &all_pool);
        score_pool_against(&fine_pool, &all_pool);
        compute_local_min_flags(&coarse_pool);
        compute_local_min_flags(&fine_pool);

        /* Coarse candidates must first receive their own +/-1 km fine search. */
        refine_count = build_level_refine_intervals(&refine_intervals,
                                                    &coarse_pool, &all_pool,
                                                    best.cost, COARSE_STEP_KM,
                                                    fine_covered,
                                                     fine_covered_count,
                                                     route_km);
        if (refine_count > 0) {
            sample_result =
                sample_intervals(T_init, SOC_init, route_km,
                                 refine_intervals, refine_count, FINE_STEP_KM,
                                 &fine_pool, &all_pool,
                                 &fallback, &has_fallback,
                                 SEARCH_LEVEL_FINE);
            if (sample_result == ADD_FULL) {
                free(refine_intervals);
                search_error = 1;
                break;
            }
            if (!mark_intervals_covered(fine_covered, &fine_covered_count,
                                        MAX_COVERED_INTERVALS,
                                        refine_intervals, refine_count,
                                        route_km)) {
                free(refine_intervals);
                search_error = 1;
                break;
            }
            mark_refined_candidates_for_covered(&coarse_pool, &all_pool,
                                                best.cost, COARSE_STEP_KM,
                                                fine_covered,
                                                fine_covered_count,
                                                route_km);
            free(refine_intervals);
            refine_intervals = NULL;
            if (sample_result == ADD_NEW)
                continue;
        }
        mark_refined_candidates_for_covered(&coarse_pool, &all_pool,
                                            best.cost, COARSE_STEP_KM,
                                            fine_covered,
                                            fine_covered_count, route_km);
        free(refine_intervals);
        refine_intervals = NULL;

        /* Fine candidates receive +/-0.05 km final refinement only after no
         * coarse candidate still needs its missing fine search. */
        if (!best_from_pool(&best, &all_pool))
            break;
        score_pool_against(&fine_pool, &all_pool);
        compute_local_min_flags(&fine_pool);

        refine_count = build_level_refine_intervals(&refine_intervals,
                                                    &fine_pool, &all_pool,
                                                    best.cost, FINE_STEP_KM,
                                                    final_covered,
                                                    final_covered_count,
                                                    route_km);
        if (refine_count > 0) {
            sample_result =
                sample_intervals(T_init, SOC_init, route_km,
                                 refine_intervals, refine_count, FINAL_STEP_KM,
                                 &final_pool, &all_pool,
                                 &fallback, &has_fallback,
                                 SEARCH_LEVEL_FINAL);
            if (sample_result == ADD_FULL) {
                free(refine_intervals);
                search_error = 1;
                break;
            }
            if (!mark_intervals_covered(final_covered, &final_covered_count,
                                        MAX_COVERED_INTERVALS,
                                        refine_intervals, refine_count,
                                        route_km)) {
                free(refine_intervals);
                search_error = 1;
                break;
            }
            mark_refined_candidates_for_covered(&fine_pool, &all_pool,
                                                best.cost, FINE_STEP_KM,
                                                final_covered,
                                                final_covered_count,
                                                route_km);
            free(refine_intervals);
            refine_intervals = NULL;
            if (sample_result == ADD_NEW)
                continue;
        }
        mark_refined_candidates_for_covered(&fine_pool, &all_pool,
                                            best.cost, FINE_STEP_KM,
                                            final_covered,
                                            final_covered_count, route_km);
        free(refine_intervals);

        break;
    }

    if (!best_from_pool(&best, &all_pool))
        best = has_fallback ? fallback :
               evaluate_start_distance(T_init, SOC_init, route_km, route_km);

    *T_end_opt = best.temp_C;
    *SOC_end_opt = best.soc_pct;
    *E_heat_opt = best.heat_kWh;
    *chrg_time_s = best.charge_s;
    return best.dist_km;
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
        printf("  %-6d %-10.1f %-10.1f %-12.1f %-12.1f\n",
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
     *   2. Use binary search or grid search to explore start_distance.
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
