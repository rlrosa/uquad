EESchema Schematic File Version 2  date mar 27 dic 2011 12:30:01 UYST
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:special
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:imu-beagle-cache
EELAYER 25  0
EELAYER END
$Descr A4 11700 8267
encoding utf-8
Sheet 1 1
Title ""
Date "27 dec 2011"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 2950 3650
Wire Wire Line
	2200 3650 2950 3650
Wire Wire Line
	8150 3850 8150 3950
Wire Wire Line
	8150 3950 7800 3950
Wire Wire Line
	7800 3950 7800 3650
Wire Wire Line
	8150 3650 8050 3650
Wire Wire Line
	8050 3650 8050 3850
Wire Wire Line
	8050 3850 7150 3850
Wire Wire Line
	5350 5200 5900 5200
Wire Wire Line
	5900 5200 5900 3900
Wire Wire Line
	2200 3850 2950 3850
Connection ~ 2950 2950
Wire Wire Line
	2950 3850 2950 2950
Wire Wire Line
	5400 4400 3100 4400
Wire Wire Line
	2200 3550 3100 3550
Wire Wire Line
	3100 3550 3100 3250
Wire Wire Line
	3100 3250 3500 3250
Wire Wire Line
	3500 3250 3500 3400
Wire Wire Line
	3500 3400 4650 3400
Wire Wire Line
	2200 3450 3250 3450
Wire Wire Line
	3250 3450 3250 3500
Wire Wire Line
	3250 3500 4650 3500
Connection ~ 3800 2950
Wire Wire Line
	3800 2975 3800 2950
Connection ~ 6750 5300
Wire Wire Line
	7300 5300 7300 4850
Connection ~ 7600 4250
Wire Wire Line
	7600 4450 7600 3550
Connection ~ 6750 2950
Wire Wire Line
	6750 5300 6750 2950
Wire Wire Line
	5550 3600 5550 4550
Wire Wire Line
	5550 3600 5900 3600
Wire Wire Line
	7150 2500 7150 3750
Wire Wire Line
	7150 2500 4650 2500
Wire Wire Line
	4650 2500 4650 3200
Connection ~ 4200 2950
Wire Wire Line
	4650 3300 4200 3300
Wire Wire Line
	4200 3300 4200 2950
Connection ~ 5700 4250
Wire Wire Line
	5700 4250 5700 3800
Wire Wire Line
	5700 3800 5900 3800
Connection ~ 4400 3400
Wire Wire Line
	4400 3400 4400 3000
Wire Wire Line
	4400 3000 5550 3000
Wire Wire Line
	5550 3000 5550 3400
Wire Wire Line
	5550 3400 5900 3400
Wire Wire Line
	7600 4250 4400 4250
Wire Wire Line
	4400 4250 4400 3800
Wire Wire Line
	4400 3800 4650 3800
Wire Wire Line
	7400 3650 7400 2950
Wire Wire Line
	7400 3650 7800 3650
Wire Wire Line
	5900 3300 5650 3300
Wire Wire Line
	5650 3300 5650 2950
Connection ~ 5650 2950
Wire Wire Line
	7150 3850 7150 4150
Wire Wire Line
	7150 4150 4650 4150
Wire Wire Line
	4650 4150 4650 3900
Wire Wire Line
	5400 4400 5400 3500
Wire Wire Line
	5400 3500 5900 3500
Wire Wire Line
	5900 3200 5900 3100
Wire Wire Line
	5900 3100 6500 3100
Wire Wire Line
	7300 4450 7300 4250
Connection ~ 7300 4250
Wire Wire Line
	7600 4850 7600 5300
Wire Wire Line
	7600 5300 5350 5300
Connection ~ 7300 5300
Wire Wire Line
	3800 3400 3800 3375
Connection ~ 3800 3400
Wire Wire Line
	4650 3600 3350 3600
Wire Wire Line
	3350 3600 3350 3350
Wire Wire Line
	3350 3350 2200 3350
Wire Wire Line
	5550 4550 3250 4550
Wire Wire Line
	3250 4550 3250 3750
Wire Wire Line
	3250 3750 2200 3750
Wire Wire Line
	3100 4400 3100 3950
Wire Wire Line
	3100 3950 2200 3950
Wire Wire Line
	7400 2950 2950 2950
Wire Wire Line
	6500 3100 6500 5100
Wire Wire Line
	6500 5100 5350 5100
Wire Wire Line
	7600 3550 8150 3550
Wire Wire Line
	7150 3750 8150 3750
Wire Wire Line
	8150 3450 7900 3450
Wire Wire Line
	7900 3450 7900 2750
Wire Wire Line
	7900 2750 2550 2750
Wire Wire Line
	2550 2750 2550 3250
Wire Wire Line
	2550 3250 2200 3250
$Comp
L CONN_8 P2
U 1 1 4EF8F746
P 1850 3600
F 0 "P2" V 1800 3600 60  0000 C CNN
F 1 "CONN_8" V 1900 3600 60  0000 C CNN
	1    1850 3600
	-1   0    0    1   
$EndComp
Text Label 5400 5300 0    60   ~ 0
GND
Text Label 5350 5100 0    60   ~ 0
SDA_ESC
Text Label 5350 5200 0    60   ~ 0
SCL_ESC
Text Label 2200 3950 0    60   ~ 0
SDA_BEA
Text Label 2200 3750 0    60   ~ 0
SCL_BEA
Text Label 5500 3600 0    60   ~ 0
SCL_BEA
Text Label 5500 3500 0    60   ~ 0
SDA_BEA
Text Label 2200 3350 0    60   ~ 0
Rx_BEA
Text Label 4300 3600 0    60   ~ 0
Rx_BEA
Text Label 4300 3500 0    60   ~ 0
Tx_BEA
Text Label 2200 3450 0    60   ~ 0
Tx_BEA
Text Label 4650 4150 0    60   ~ 0
Tx_IMU
Text Label 7150 3850 0    60   ~ 0
Tx_IMU
Text Label 7150 3750 0    60   ~ 0
Rx_IMU
Text Label 4650 2500 0    60   ~ 0
Rx_IMU
Text Label 7400 3650 0    60   ~ 0
GND
Text Label 2200 3850 0    60   ~ 0
GND
Text Label 5650 3300 0    60   ~ 0
GND
Text Label 4450 3300 0    60   ~ 0
GND
Text Label 5650 3400 0    60   ~ 0
1.8V
Text Label 4450 3400 0    60   ~ 0
1.8V
Text Label 2200 3550 0    60   ~ 0
1.8V
Text Label 4400 3800 0    60   ~ 0
3,3V
Text Label 5700 3800 0    60   ~ 0
3.3V
Text Label 7600 3550 0    60   ~ 0
3,3V
Text Label 8150 3450 0    60   ~ 0
5V
Text Label 2200 3250 0    60   ~ 0
5V
$Comp
L C C3
U 1 1 4EF8C709
P 7600 4650
F 0 "C3" H 7650 4750 50  0000 L CNN
F 1 "1 uF" H 7650 4550 50  0000 L CNN
	1    7600 4650
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 4EF8C6F9
P 3800 3175
F 0 "C1" H 3850 3275 50  0000 L CNN
F 1 "0,1 uF" H 3850 3075 50  0000 L CNN
	1    3800 3175
	1    0    0    -1  
$EndComp
$Comp
L CONN_5 P4
U 1 1 4EF8C6AC
P 8550 3650
F 0 "P4" V 8500 3650 50  0000 C CNN
F 1 "CONN_5" V 8600 3650 50  0000 C CNN
	1    8550 3650
	1    0    0    -1  
$EndComp
$Comp
L CONN_3 K1
U 1 1 4EF8C69F
P 5000 5200
F 0 "K1" V 4950 5200 50  0000 C CNN
F 1 "CONN_3" V 5050 5200 40  0000 C CNN
	1    5000 5200
	-1   0    0    1   
$EndComp
$Comp
L CONN_8 P3
U 1 1 4EF8C66C
P 6250 3550
F 0 "P3" V 6200 3550 60  0000 C CNN
F 1 "CONN_8" V 6300 3550 60  0000 C CNN
	1    6250 3550
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 4EF26E6B
P 7300 4650
F 0 "C2" H 7350 4750 50  0000 L CNN
F 1 "0.1 uF" H 7350 4550 50  0000 L CNN
	1    7300 4650
	1    0    0    -1  
$EndComp
$Comp
L CONN_8 P1
U 1 1 4EF25B43
P 5000 3550
F 0 "P1" V 4950 3550 60  0000 C CNN
F 1 "CONN_8" V 5050 3550 60  0000 C CNN
	1    5000 3550
	1    0    0    -1  
$EndComp
$EndSCHEMATC
