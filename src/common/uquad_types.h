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

#define GRAVITY 9.81L	   // Aceleracion gravitatoria
#define IXX     0.0232L	   // Tensor de inercia del quad - según x
#define IYY     0.0232L	   // Tensor de inercia del quad - según y
#define IZZ     0.0437L	   // Tensor de inercia del quad - según z
#define IZZM    0.0000154L // Tensor de inercia de los motores - segun z
#define LENGTH  0.29L	   // Largo en metros del los brazos del quad
#define MASA    1.741L	   // Masa del Quad en kg
#define PI      3.1415926L

#endif
