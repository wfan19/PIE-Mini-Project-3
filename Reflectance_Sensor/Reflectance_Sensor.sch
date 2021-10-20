EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Module:Arduino_UNO_R3 A1
U 1 1 616F5C54
P 2700 2900
F 0 "A1" H 2700 4081 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 2700 3990 50  0000 C CNN
F 2 "Module:Arduino_UNO_R3" H 2700 2900 50  0001 C CIN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 2700 2900 50  0001 C CNN
	1    2700 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R1
U 1 1 616F9443
P 3750 2650
F 0 "R1" H 3818 2696 50  0000 L CNN
F 1 "R_200" H 3818 2605 50  0000 L CNN
F 2 "" V 3790 2640 50  0001 C CNN
F 3 "~" H 3750 2650 50  0001 C CNN
	1    3750 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R3
U 1 1 616F9DD7
P 5750 2650
F 0 "R3" H 5818 2696 50  0000 L CNN
F 1 "R_200" H 5818 2605 50  0000 L CNN
F 2 "" V 5790 2640 50  0001 C CNN
F 3 "~" H 5750 2650 50  0001 C CNN
	1    5750 2650
	1    0    0    -1  
$EndComp
$Comp
L LED:IR204A D1
U 1 1 616FAD17
P 3750 3000
F 0 "D1" V 3746 2921 50  0000 R CNN
F 1 "IR LED" V 3655 2921 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_IRBlack" H 3750 3175 50  0001 C CNN
F 3 "http://www.everlight.com/file/ProductFile/IR204-A.pdf" H 3700 3000 50  0001 C CNN
	1    3750 3000
	0    -1   -1   0   
$EndComp
$Comp
L LED:IR204A D2
U 1 1 616FB227
P 5750 3000
F 0 "D2" V 5746 2921 50  0000 R CNN
F 1 "IR LED" V 5655 2921 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_IRBlack" H 5750 3175 50  0001 C CNN
F 3 "http://www.everlight.com/file/ProductFile/IR204-A.pdf" H 5700 3000 50  0001 C CNN
	1    5750 3000
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 616FCA83
P 3750 3300
F 0 "#PWR02" H 3750 3050 50  0001 C CNN
F 1 "GND" H 3755 3127 50  0000 C CNN
F 2 "" H 3750 3300 50  0001 C CNN
F 3 "" H 3750 3300 50  0001 C CNN
	1    3750 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 3200 3750 3300
$Comp
L power:GND #PWR04
U 1 1 616FCF5E
P 5750 3300
F 0 "#PWR04" H 5750 3050 50  0001 C CNN
F 1 "GND" H 5755 3127 50  0000 C CNN
F 2 "" H 5750 3300 50  0001 C CNN
F 3 "" H 5750 3300 50  0001 C CNN
	1    5750 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 3300 5750 3200
Wire Wire Line
	5750 2800 5750 2900
Wire Wire Line
	3750 2900 3750 2800
$Comp
L Device:Q_Photo_NPN_CE Q1
U 1 1 61706F55
P 4500 3050
F 0 "Q1" H 4690 3096 50  0000 L CNN
F 1 "Phototransistor" H 4690 3005 50  0000 L CNN
F 2 "" H 4700 3150 50  0001 C CNN
F 3 "~" H 4500 3050 50  0001 C CNN
	1    4500 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 61707A50
P 4600 2650
F 0 "R2" H 4668 2696 50  0000 L CNN
F 1 "R_10k" H 4668 2605 50  0000 L CNN
F 2 "" V 4640 2640 50  0001 C CNN
F 3 "~" H 4600 2650 50  0001 C CNN
	1    4600 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 2800 4600 2850
$Comp
L power:GND #PWR03
U 1 1 61708195
P 4600 3300
F 0 "#PWR03" H 4600 3050 50  0001 C CNN
F 1 "GND" H 4605 3127 50  0000 C CNN
F 2 "" H 4600 3300 50  0001 C CNN
F 3 "" H 4600 3300 50  0001 C CNN
	1    4600 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3250 4600 3300
$Comp
L Device:Q_Photo_NPN_CE Q2
U 1 1 6170BCD3
P 6350 3050
F 0 "Q2" H 6540 3096 50  0000 L CNN
F 1 "Phototransistor" H 6540 3005 50  0000 L CNN
F 2 "" H 6550 3150 50  0001 C CNN
F 3 "~" H 6350 3050 50  0001 C CNN
	1    6350 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R4
U 1 1 6170BCD9
P 6450 2650
F 0 "R4" H 6518 2696 50  0000 L CNN
F 1 "R_10k" H 6518 2605 50  0000 L CNN
F 2 "" V 6490 2640 50  0001 C CNN
F 3 "~" H 6450 2650 50  0001 C CNN
	1    6450 2650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 2800 6450 2850
$Comp
L power:GND #PWR05
U 1 1 6170BCE0
P 6450 3300
F 0 "#PWR05" H 6450 3050 50  0001 C CNN
F 1 "GND" H 6455 3127 50  0000 C CNN
F 2 "" H 6450 3300 50  0001 C CNN
F 3 "" H 6450 3300 50  0001 C CNN
	1    6450 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 3300 6450 3250
$Comp
L power:+5V #PWR01
U 1 1 6170FAE2
P 3750 2450
F 0 "#PWR01" H 3750 2300 50  0001 C CNN
F 1 "+5V" H 3765 2623 50  0000 C CNN
F 2 "" H 3750 2450 50  0001 C CNN
F 3 "" H 3750 2450 50  0001 C CNN
	1    3750 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 2450 3750 2500
Connection ~ 3750 2450
Wire Wire Line
	5750 2500 5750 2450
Connection ~ 5750 2450
Wire Wire Line
	3750 2450 4600 2450
Wire Wire Line
	4600 2500 4600 2450
Connection ~ 4600 2450
Wire Wire Line
	4600 2450 5750 2450
Wire Wire Line
	4600 2800 4200 2800
Wire Wire Line
	4200 2800 4200 3650
Wire Wire Line
	4200 3650 3500 3650
Wire Wire Line
	3500 3650 3500 2900
Wire Wire Line
	3500 2900 3200 2900
Connection ~ 4600 2800
Wire Wire Line
	6450 2450 6450 2500
Wire Wire Line
	5750 2450 6450 2450
Wire Wire Line
	6450 2800 6150 2800
Wire Wire Line
	6150 2800 6150 3700
Wire Wire Line
	6150 3700 3450 3700
Wire Wire Line
	3450 3700 3450 3000
Wire Wire Line
	3450 3000 3200 3000
Connection ~ 6450 2800
Text Notes 3800 2150 0    50   ~ 0
Left Reflectance Sensor\n
Text Notes 5550 2150 0    50   ~ 0
Right Reflectance Sensor\n
$EndSCHEMATC