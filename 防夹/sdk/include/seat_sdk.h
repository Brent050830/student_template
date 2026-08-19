/**
 * Seat Back Recline Anti-Pinch Detection SDK
 * ==========================================
 *
 * This is the ONLY header file students need to read.
 * All implementation details (data loading, RTE simulation,
 * evaluation) are pre-built into static libraries.
 *
 * Student Task:
 *   Implement ONLY the two functions at the bottom of this file:
 *     - anti_pinch_detector_init()
 *     - anti_pinch_detector_step()
 *
 *   All data input/output must go through the service interfaces
 *   declared below (AUTOSAR RTE style).
 */

#ifndef SEAT_SDK_H
#define SEAT_SDK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * Parameter Type Definitions (aligned with service interface)
 * ============================================================ */
typedef uint16_t MotCurr_u16;       /* Motor current, 0~10636 */
typedef uint16_t HallPosn_u16;      /* Hall position, 0~65535 */
typedef uint8_t  OperMotSt_u8;      /* Motor running state, 0/1/2 */
typedef uint8_t  AntiPinchSt_u8;    /* Anti-pinch state, 0=Normal, 1=Occurred */
typedef uint32_t HallPlsWidth_u32;  /* Hall pulse width */
typedef uint16_t MotPwrVolt_u16;    /* Supply voltage, ADC value */
typedef uint8_t  Posn_u8;           /* Backrest angle, 0x0-0x64=0%-100%, 0xFF=Invalid */

/* -- AntiPinchSt_u8 enum values -- */
#define ANTIPINCHST_NORMAL      0   /* Normal */
#define ANTIPINCHST_OCCURRED    1   /* Anti-pinch occurred */
#define ANTIPINCHST_RESTRAINED  2   /* Anti-pinch restrained (reserved) */

/* -- OperMotSt_u8 enum values -- */
#define OPERMOTST_NO_RUNNING    0   /* Stopped */
#define OPERMOTST_LEFT_UP       1   /* Forward (left/up) */
#define OPERMOTST_RIGHT_DOWN    2   /* Reverse (right/down) */


/* ============================================================
 * Tunable Parameters
 * ============================================================ */

/* -- Voltage normalization -- */
#define V_REF               13.5f           /* Reference voltage (V) */
#define V_ADC_FACTOR        (13.5f / 13650.0f)  /* ADC -> physical value factor */

/* -- Anti-pinch detection zone (Posn_u8: 0~100 valid, 0xFF=Invalid) -- */
#define PINCH_ZONE_MIN  1
#define PINCH_ZONE_MAX  99

/* -- Adaptive baseline -- */
#define BASELINE_WINDOW     200
#define BASELINE_MIN_FILL   50

/* -- Adaptive threshold -- */
#define CURR_SIGMA_FACTOR   3
#define HW_SIGMA_FACTOR     3
#define CURR_DEV_MIN        800.0f
#define HW_DEV_MIN          800.0f

/* -- Motor startup delay -- */
#define STARTUP_DELAY_MS    500
#define STARTUP_DELAY_SAMPLES  (STARTUP_DELAY_MS / 2)

/* -- Duration filter -- */
#define MIN_DURATION_MS     20
#define MIN_DURATION_SAMPLES (MIN_DURATION_MS / 2)

/* -- Sampling period -- */
#define SAMPLE_PERIOD_MS    2


/* ============================================================
 * Vehicle Service Interfaces (AUTOSAR RTE Style)
 * ============================================================
 *
 * Input interfaces (6 services): getter functions, write to pointer param
 *   Interface name                              Parameter type      Description
 *   SeatBackRclnMotDD_u16MotCur               MotCurr_u16        Motor current
 *   BO_Atm_SeatBackRcln_ntfHallPosn           HallPosn_u16       Hall position
 *   SeatBackRclnHallDD_u32CurrHallPlsWidth    HallPlsWidth_u32   Hall pulse width
 *   BO_Atm_SeatBackRcln_ntfOperSt             OperMotSt_u8       Motor running state
 *   SeatBackRclnMotDD_u16MotPwrVolt           MotPwrVolt_u16     Supply voltage
 *   BO_Atm_SeatBackRcln_ntfPosn               Posn_u8            Backrest angle position
 *
 * Output interface (1 BO atomic service): setter function, pass value param
 *   BO_Atm_SeatBackRcln_ntfAntiPinchSt        AntiPinchSt_u8     Anti-pinch state
 */

/* -- Input interfaces: getter -- */
extern void SeatBackRclnMotDD_u16MotCur(MotCurr_u16 *abs);
extern void BO_Atm_SeatBackRcln_ntfHallPosn(HallPosn_u16 *value);
extern void SeatBackRclnHallDD_u32CurrHallPlsWidth(HallPlsWidth_u32 *value);
extern void BO_Atm_SeatBackRcln_ntfOperSt(OperMotSt_u8 *value);
extern void SeatBackRclnMotDD_u16MotPwrVolt(MotPwrVolt_u16 *value);
extern void BO_Atm_SeatBackRcln_ntfPosn(Posn_u8 *value);

/* -- Output interface: setter -- */
extern void BO_Atm_SeatBackRcln_ntfAntiPinchSt(AntiPinchSt_u8 value);


/* ============================================================
 * Student Algorithm Contract
 * ============================================================
 *
 * Implement ONLY these two functions in student_solution.c.
 * Do NOT modify this header file.
 */

/** Initialize the detector state.
 *  Called once before the first step(). */
void anti_pinch_detector_init(void);

/** Core scheduling: read input interfaces -> write output interface.
 *  Called once per sampling period (2ms) by the runtime framework.
 *
 *  Algorithm flow (each step call):
 *    1. Read input signals via getter interfaces
 *    2. Voltage normalization
 *    3. Startup delay check
 *    4. Adaptive baseline update
 *    5. Position stall detection
 *    6. Adaptive threshold judgment
 *    7. Duration filtering
 *    8. Output anti-pinch state via setter interface
 */
void anti_pinch_detector_step(void);


#ifdef __cplusplus
}
#endif

#endif /* SEAT_SDK_H */
