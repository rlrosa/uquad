EESchema Schematic File Version 2  date dom 20 nov 2011 03:15:23 UYST
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
EELAYER 43  0
EELAYER END
$Descr A4 11700 8267
encoding utf-8
Sheet 1 1
Title ""
Date "20 nov 2011"
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Connection ~ 2500 4200
Wire Wire Line
	2500 4200 2500 5050
Wire Wire Line
	2500 5050 2600 5050
Wire Wire Line
	2400 4000 3100 4000
Wire Wire Line
	3100 4000 3100 3600
Wire Wire Line
	3100 3600 3900 3600
Connection ~ 2950 2000
Wire Wire Line
	2400 4200 2950 4200
Wire Wire Line
	2950 4200 2950 1850
Connection ~ 2950 4500
Connection ~ 2950 1850
Connection ~ 3900 3800
Connection ~ 3900 3600
Connection ~ 3900 3400
Connection ~ 3800 3400
Connection ~ 3800 3000
Connection ~ 3900 3000
Connection ~ 3900 2900
Connection ~ 3800 2900
Connection ~ 3800 2500
Connection ~ 3900 2500
Connection ~ 6300 3000
Connection ~ 6300 2600
Connection ~ 5500 2500
Connection ~ 5500 2900
Connection ~ 5500 3100
Connection ~ 5500 3400
Connection ~ 5500 3800
Connection ~ 5500 3600
Connection ~ 5700 3500
Connection ~ 5700 3900
Connection ~ 6050 3900
Connection ~ 6050 3500
Connection ~ 7150 3750
Connection ~ 7150 3650
Connection ~ 7150 3950
Connection ~ 7150 4300
Connection ~ 7150 4300
Wire Wire Line
	7150 3950 7150 4300
Wire Wire Line
	3900 3000 3800 3000
Wire Wire Line
	6050 3500 6050 2900
Wire Wire Line
	6050 2900 5500 2900
Wire Wire Line
	5700 3500 5700 3400
Wire Wire Line
	5700 3400 5500 3400
Wire Wire Line
	3900 2500 3800 2500
Wire Wire Line
	5500 3600 7150 3750
Wire Wire Line
	2950 4300 2950 4500
Wire Wire Line
	5500 3100 6300 3100
Wire Wire Line
	2950 2000 6300 2000
Wire Wire Line
	6300 2000 6300 2600
Wire Wire Line
	5500 2500 6300 2500
Connection ~ 6300 2000
Wire Wire Line
	6300 4300 6300 3000
Connection ~ 6300 4300
Connection ~ 6300 3100
Connection ~ 6300 2500
Wire Wire Line
	5500 3800 7150 3650
Wire Wire Line
	3800 2900 3900 2900
Wire Wire Line
	5700 3900 5700 4300
Connection ~ 5700 4300
Wire Wire Line
	6050 4300 6050 3900
Connection ~ 6050 4300
Wire Wire Line
	3800 3400 3900 3400
Wire Wire Line
	7150 4300 2400 4300
Connection ~ 2950 4300
Wire Wire Line
	3900 3800 3250 3800
Wire Wire Line
	3250 3800 3250 4100
Wire Wire Line
	3250 4100 2400 4100
Wire Wire Line
	2600 4300 2600 4850
Connection ~ 2600 4300
$Comp
L CONN_2 P3
U 1 1 4EC87B37
P 2950 4950
F 0 "P3" V 2900 4950 40  0000 C CNN
F 1 "CONN_2" V 3000 4950 40  0000 C CNN
	1    2950 4950
	1    0    0    -1  
$EndComp
$Comp
L CONN_4 P1
U 1 1 4EC879A4
P 2050 4150
F 0 "P1" V 2000 4150 50  0000 C CNN
F 1 "CONN_4" V 2100 4150 50  0000 C CNN
	1    2050 4150
	-1   0    0    1   
$EndComp
$Comp
L CONN_4 P2
U 1 1 4EC8789E
P 7500 3800
F 0 "P2" V 7450 3800 50  0000 C CNN
F 1 "CONN_4" V 7550 3800 50  0000 C CNN
	1    7500 3800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 4EC86B8C
P 2950 4500
F 0 "#PWR01" H 2950 4500 30  0001 C CNN
F 1 "GND" H 2950 4430 30  0001 C CNN
	1    2950 4500
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR02
U 1 1 4EC86653
P 2950 1850
F 0 "#PWR02" H 2950 1940 20  0001 C CNN
F 1 "+5V" H 2950 1940 30  0000 C CNN
	1    2950 1850
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 4EC86575
P 3800 2700
F 0 "C1" H 3850 2800 50  0000 L CNN
F 1 "C" H 3850 2600 50  0000 L CNN
	1    3800 2700
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 4EC86572
P 3800 3200
F 0 "C2" H 3850 3300 50  0000 L CNN
F 1 "C" H 3850 3100 50  0000 L CNN
	1    3800 3200
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 4EC8656E
P 6050 3700
F 0 "C4" H 6100 3800 50  0000 L CNN
F 1 "C" H 6100 3600 50  0000 L CNN
	1    6050 3700
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 4EC8656B
P 5700 3700
F 0 "C3" H 5750 3800 50  0000 L CNN
F 1 "C" H 5750 3600 50  0000 L CNN
	1    5700 3700
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 4EC86566
P 6300 2800
F 0 "C5" H 6350 2900 50  0000 L CNN
F 1 "C" H 6350 2700 50  0000 L CNN
	1    6300 2800
	1    0    0    -1  
$EndComp
$Comp
L MAX232 U1
U 1 1 4EC86471
P 4700 3200
F 0 "U1" H 4700 4050 70  0000 C CNN
F 1 "MAX232" H 4700 2350 70  0000 C CNN
	1    4700 3200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
