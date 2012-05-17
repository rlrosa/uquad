#ifndef UQUAD_TYPES_H
#define UQUAD_TYPES_H

#ifndef UQUAD_BOOL
  #define UQUAD_BOOL int
  #ifndef true
    #define true 1
    #define false 0
  #endif //true
#endif //UQUAD_BOOL
typedef UQUAD_BOOL uquad_bool_t;

typedef enum STATE_VECTOR{
    SV_X = 0,
    SV_Y,
    SV_Z,
    SV_PSI,
    SV_PHI,
    SV_THETA,
    SV_VQX,
    SV_VQY,
    SV_VQZ,
    SV_WQX,
    SV_WQY,
    SV_WQZ,
    SV_BAX,
    SV_BAY,
    SV_BAZ
}STATE_VECTOR_T;

#define GRAVITY           9.81 	      // Aceleracion gravitatoria
#define IXX               0.0232      // Tensor de inercia del quad - según x
#define IYY               0.0232      // Tensor de inercia del quad - según y
#define IZZ               0.0437      // Tensor de inercia del quad - según z
#define IZZM              0.0000154   // Tensor de inercia de los motores - segun z
#define LENGTH            0.29	      // Largo en metros del los brazos del quad
#define PI                3.1415926
#define F_B1              4.60160135072435e-05     // Coeficiente cuadrático de la fuerza
#define F_B2              -0.00103822726273726     // Coeficiente lineal de la fuerza
#define M_D1              3.4734e-6   // Coeficiente cuadrático del torque
#define M_D2              -1.3205e-4  // Coeficiente lineal del torque
#define DRAG_A1           0.0000034734// drag = w^2*A2 + w*A1
#define DRAG_A2           -0.00013205 // drag = w^2*A2 + w*A1
#define DRIVE_A1          4.60160135072435e-05 // drive = w^2*A2 + w*A1
#define DRIVE_A2          -0.00103822726273726 // drive = w^2*A2 + w*A1
#define MASA_DEFAULT      (1.600)     // Weight of the quadcopter [kg]
#define STATE_COUNT       12          // State vector length
#define LENGTH_INPUT      4           // Input vector length

/**
 *
 * Sampling time within [TS_MIN,TS_MAX] will be used for
 * kalman filtering. If out of range, using it within the kalman filter
 * would force a violent reaction, and the stabilization time would
 * be unacceptable.
 *
 */
#define TS_JITTER          2000L // Max jitter accepted
#define TS_ERROR_WAIT      10    // Wait 10 errors before logging again
#define TS_MAX             (TS_DEFAULT_US + TS_JITTER)
#define TS_MIN             (TS_DEFAULT_US - TS_JITTER)

/**
 * check_net macros
 *
 * In a worst case scenario, check_net will take a max
 * of CHECK_NET_TO_S [s] + CHECK_NET_MSG_T_MS [ms] to
 * detect loss of connectivity.
 */
#define CHECK_NET_ACK          "OK"
#define CHECK_NET_PING         "HI"
#define CHECK_NET_KILL_RETRIES 10
#define CHECK_NET_MSG_LEN      2    // [bytes]
#define CHECK_NET_MSG_T_MS     950  // [ms] - time between pings
#define CHECK_NET_RETRY_MS     50   // [ms] - time between requests for ack
#define CHECK_NET_TO_S         1    // [s]  - timeout
#define CHECK_NET_SERVER_IP    "10.42.43.1"
#define CHECK_NET_PORT         12341234
#endif
