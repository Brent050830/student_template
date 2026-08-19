/**
 * @file usp_api.h
 * @brief Vehicle interface definitions for BEV battery preheat decision algorithm
 *
 * Students call these low-level interfaces directly to obtain:
 *   - Battery state (temperature, SOC, voltage, current)
 *   - Thermal management data (ambient temperature, heating power)
 *   - Route segment predictions (distance, speed, drive power, ambient temp)
 *     via VehPwrPred_getPwrPred()
 *
 * In the real vehicle environment, these are provided by CAN bus / ECU.
 * In the simulation environment, competition_mock.c provides benchmark data.
 */

#ifndef USP_API_H
#define USP_API_H

#include <stdint.h>

typedef uint32_t uint32;

#ifdef __cplusplus
extern "C"
{
#endif

    /*============================================================================
     *  Power prediction / route segment data types (VehPwrPred service)
     *
     *  This is the UNIFIED source of navigation + power prediction data.
     *  Each segment contains: distance, speed, drive power, ambient temp.
     *  Students should call VehPwrPred_getPwrPred() to obtain route info.
     *=========================================================================== */

    typedef uint16_t SegId_u16;     /* Segment ID */
    typedef float Dist_km_f32;      /* Distance, km */
    typedef float Spd_kmDIVh_f32;   /* Speed, km/h */
    typedef float DrvPwr_kW_f32;    /* Drive power, kW */
    typedef float AmbTemp_DegC_f32; /* Ambient temperature, C */

    typedef struct
    {
        SegId_u16 segId;          /* Segment ID */
        Dist_km_f32 length;       /* Segment length, km */
        Spd_kmDIVh_f32 avgSpd;    /* Average speed, km/h */
        DrvPwr_kW_f32 drvPwr;     /* Predicted drive power, kW */
        AmbTemp_DegC_f32 ambTemp; /* Average ambient temp, C */
    } PwrPredSeg_stru;

#define PWRPRED_MAX_SEGS 200

    typedef struct
    {
        uint8_t num;                                   /* Valid data length */
        PwrPredSeg_stru pwrPredList[PWRPRED_MAX_SEGS]; /* Prediction sequence */
    } PwrPredList_stru;

    /*============================================================================
     *  Battery state interfaces (EMS_HVBatt service)
     *=========================================================================== */

    /* Average cell temperature of the battery pack, unit: C */
    void EMS_HVBatt_getTempAvg(float *rty_getTempAvg);

    /* High-voltage battery target SOC, unit: %, range: 0~100.00 */
    void EMS_HVBatt_getTargetSOC(float *rty_getTargetSOC);

    /* High-voltage battery real-time current, unit: A
     * Positive = charging, Negative = discharging */
    void EMS_HVBatt_getCurrent(float *rty_getCurrent);

    /* High-voltage battery real-time voltage, unit: V */
    void EMS_HVBatt_getVolt(float *rty_getVolt);

    /*============================================================================
     *  Thermal management interfaces (TMS services)
     *=========================================================================== */

    /* Outside ambient temperature, unit: C */
    void TMS_EnvMonitor_getEnvTemp(float *rty_getEnvTemp);

    /*============================================================================
     *  Power prediction / route segment interface (VehPwrPred service)
     *
     *  UNIFIED interface for navigation + power prediction.
     *  Each segment in pwrPredList contains:
     *    - segId   : segment ID
     *    - length  : segment distance (km)
     *    - avgSpd  : average speed (km/h)
     *    - drvPwr  : predicted drive power (kW)
     *    - ambTemp : average ambient temperature (C)
     *=========================================================================== */

    /* Get predicted drive power sequence for upcoming route segments.
     * Fills output struct with num segments and per-segment predictions. */
    void VehPwrPred_getPwrPred(PwrPredList_stru *rty_getPwrPred);

    /*============================================================================
     *  Preheat decision result interfaces (BattChrgPreHeatg service)
     *
     *  ntf*  — student algorithm calls to NOTIFY system of computed results
     *  get*  — system/grading calls to QUERY the notified results
     *=========================================================================== */

    /* Notify: optimal preheat start distance (km), counted from departure */
    void BattChrgPreHeatg_ntfPreHeatgStartDist(Dist_km_f32 StartDist_km);

    /* Notify: optimal battery temperature at charger arrival (C) */
    void BattChrgPreHeatg_ntfPreHeatgEndTemp(float temp_C);

    /* Notify: target battery SOC at charger arrival (%, 0~100) */
    void BattChrgPreHeatg_ntfPreHeatgEndSOC(float soc_perc);

    /* Notify: total preheat energy consumption (kWh) */
    void BattChrgPreHeatg_ntfPreHeatgEnergy(float energy_kWh);

    /* Query: get notified preheat start distance (km) */
    void BattChrgPreHeatg_getPreHeatgStartDist(Dist_km_f32 *rty_getPreHeatgStartDist);

    /* Query: get notified battery temperature at charger */
    void BattChrgPreHeatg_getPreHeatgEndTemp(float *rty_getPreHeatgEndTemp);

    /* Query: get notified SOC at charger */
    void BattChrgPreHeatg_getPreHeatgEndSOC(float *rty_getPreHeatgEndSOC);

    /* Query: get notified preheat energy */
    void BattChrgPreHeatg_getPreHeatgEnergy(float *rty_getPreHeatgEnergy);

    /*============================================================================
     *  Simulation control (provided by mock, for local testing only)
     *=========================================================================== */

    void mock_setSimulationState(float T, float SOC, float time_s);
    void mock_resetSimulation(void);
    void mock_advanceTime(float dt);
    float mock_getTotalTime(void);
    void mock_getSegmentData(int segIdx, float *s_km, float *v_kmh,
                             float *P_kW, float *T_env);

#ifdef __cplusplus
}
#endif

#endif /* USP_API_H */
