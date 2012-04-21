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
    SV_WQZ
}STATE_VECTOR_T;

#define GRAVITY           9.81L	      // Aceleracion gravitatoria
#define IXX               0.0232L     // Tensor de inercia del quad - según x
#define IYY               0.0232L     // Tensor de inercia del quad - según y
#define IZZ               0.0437L     // Tensor de inercia del quad - según z
#define IZZM              0.0000154L  // Tensor de inercia de los motores - segun z
#define LENGTH            0.29L	      // Largo en metros del los brazos del quad
#define PI                3.1415926L
#define F_B1              4.6016e-05L // Coeficiente cuadrático de la fuerza
#define F_B2              -0.0010L    // Coeficiente lineal de la fuerza
#define M_D1              3.4734e-6L  // Coeficiente cuadrático del torque
#define M_D2              -1.3205e-4L // Coeficiente lineal del torque
//#define MASA              1.741L    // Masa del Quad en kg
#define MOT_W_HOVER       310.00L     // rad/s
#define MASA              ((MOT_W_HOVER*MOT_W_HOVER*F_B1 \
			    + MOT_W_HOVER*F_B2)*4/GRAVITY)
#define STATE_COUNT       12
#define LENGTH_INPUT      4           // Tamaño del vector de entradass  

#endif
