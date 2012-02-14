La IMU está configurada así:

6DOF Atomic setup, version 1.0
========================================
1) View/edit active channel list
2) Change output mode, currently ASCII
3) Set Auto run mode, currently on
4) Set accelerometer sensitivity, currently 6g
5) Set output frequency, currently 100
9) Save settings and run unit

Los nombres de los archivos llevan el siguiente formato:
EjeGiro_AnguloGiro_EjeVerticalHaciaArriba.txt
Por la construcción de la caja/plano inclinado el eje de giro queda apuntando hacia el fondo de la caja siempre (para tener giros de ángulos positivos)

El dibujo en la IMU de los ejes no está bien (sale de pruebas prácticas), entonces se redefinen los ejes. Por ello el log debe interpretarse así:

1ª col: timestamp
2ª col: aceleración según y
3ª col: aceleración según x
4ª col: aceleración según z
5ª col: velocidad angular según giro en y
6ª col: velocidad angular según giro en x
7ª col: velocidad angular según giro en z

Todas las pruebas se toman durante 1 minuto para luego promediar

log-0gr-1hr.txt: 0º con eje "z" perpendicular al plano inclinado hacia arriba
