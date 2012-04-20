#ifndef UQUAD_CONFIG_H
#define UQUAD_CONFIG_H

#include <uquad_types.h>

#define DEBUG             1 // Show debug info

#define GPS_FAKE          1 // Simulate GPS data (use zeros)
#define GPS_INIT_TOUT_S   60// GPS init timeout in sec
#define GPS_INIT_TOUT_US  0 // GPS init timeout in sec
#if (!USE_GPS && GPS_FAKE)
#error							\
    "Cannot use fake GPS without USE_GPS enabled!"	\
    "Edit src/CMakelists.txt to enable USE_GPS"
#endif // (!USE_GPS && GPS_FAKE)

#define CTRL_INTEGRAL     0 // Use PI control
#define FULL_CONTROL      0 // Control 12 states

#if FULL_CONTROL
#define STATES_CONTROLLED     STATE_COUNT
#define STATES_INT_CONTROLLED 4
#else
#define STATES_CONTROLLED     8
#define STATES_INT_CONTROLLED 2
#endif


#endif // UQUAD_CONFIG_H
