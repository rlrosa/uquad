#ifndef UQUAD_CONFIG_H
#define UQUAD_CONFIG_H

#include <uquad_types.h>

#define DEBUG             1 // Show debug info

#define GPS_FAKE          0 // Simulate GPS data (use zeros)

#define KALMAN_BIAS       1 // Use kalman estimation of acc bias

#define CTRL_INTEGRAL     0 // Use PI control
#define FULL_CONTROL      0 // Control 12 states

// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
// Do NOT edit the following lines!
// -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
#if FULL_CONTROL
#define STATES_CONTROLLED     STATE_COUNT
#define STATES_INT_CONTROLLED 4
#else
#define STATES_CONTROLLED     8
#define STATES_INT_CONTROLLED 2
#endif

#if !KALMAN_BIAS
#error								\
    "Kalman bias removal not implemented correctly for H()"
#endif // !KALMAN_BIAS

#if (!USE_GPS && GPS_FAKE)
#error							\
    "Cannot use fake GPS without USE_GPS enabled!"	\
    "Edit src/CMakelists.txt to enable USE_GPS"
#endif // (!USE_GPS && GPS_FAKE)

#if (!USE_GPS && FULL_CONTROL)
#error						\
    "Cannot use FULL_CONTROL without USE_GPS!"
#endif // (!USE_GPS && FULL_CONTROL)

#if (USE_GPS && !FULL_CONTROL)
#error						\
    "Without FULL_CONTROL, GPS is useless!"
#endif // (!USE_GPS && FULL_CONTROL)

#if KALMAN_BIAS
#define STATE_BIAS        3
#else
#define STATE_BIAS        0
#endif

#endif // UQUAD_CONFIG_H
