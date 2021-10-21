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
P 3850 2200
F 0 "R1" H 3918 2246 50  0000 L CNN
F 1 "R_200" H 3918 2155 50  0000 L CNN
F 2 "" V 3890 2190 50  0001 C CNN
F 3 "~" H 3850 2200 50  0001 C CNN
	1    3850 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R3
U 1 1 616F9DD7
P 5850 2200
F 0 "R3" H 5918 2246 50  0000 L CNN
F 1 "R_200" H 5918 2155 50  0000 L CNN
F 2 "" V 5890 2190 50  0001 C CNN
F 3 "~" H 5850 2200 50  0001 C CNN
	1    5850 2200
	1    0    0    -1  
$EndComp
$Comp
L LED:IR204A D1
U 1 1 616FAD17
P 3850 2550
F 0 "D1" V 3846 2471 50  0000 R CNN
F 1 "IR LED" V 3755 2471 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_IRBlack" H 3850 2725 50  0001 C CNN
F 3 "http://www.everlight.com/file/ProductFile/IR204-A.pdf" H 3800 2550 50  0001 C CNN
	1    3850 2550
	0    -1   -1   0   
$EndComp
$Comp
L LED:IR204A D2
U 1 1 616FB227
P 5850 2550
F 0 "D2" V 5846 2471 50  0000 R CNN
F 1 "IR LED" V 5755 2471 50  0000 R CNN
F 2 "LED_THT:LED_D3.0mm_IRBlack" H 5850 2725 50  0001 C CNN
F 3 "http://www.everlight.com/file/ProductFile/IR204-A.pdf" H 5800 2550 50  0001 C CNN
	1    5850 2550
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR02
U 1 1 616FCA83
P 3850 2850
F 0 "#PWR02" H 3850 2600 50  0001 C CNN
F 1 "GND" H 3855 2677 50  0000 C CNN
F 2 "" H 3850 2850 50  0001 C CNN
F 3 "" H 3850 2850 50  0001 C CNN
	1    3850 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2750 3850 2850
$Comp
L power:GND #PWR04
U 1 1 616FCF5E
P 5850 2850
F 0 "#PWR04" H 5850 2600 50  0001 C CNN
F 1 "GND" H 5855 2677 50  0000 C CNN
F 2 "" H 5850 2850 50  0001 C CNN
F 3 "" H 5850 2850 50  0001 C CNN
	1    5850 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5850 2850 5850 2750
Wire Wire Line
	5850 2350 5850 2450
Wire Wire Line
	3850 2450 3850 2350
$Comp
L Device:Q_Photo_NPN_CE Q1
U 1 1 61706F55
P 4600 2600
F 0 "Q1" H 4790 2646 50  0000 L CNN
F 1 "Phototransistor" H 4790 2555 50  0000 L CNN
F 2 "" H 4800 2700 50  0001 C CNN
F 3 "~" H 4600 2600 50  0001 C CNN
	1    4600 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R2
U 1 1 61707A50
P 4700 2200
F 0 "R2" H 4768 2246 50  0000 L CNN
F 1 "R_10k" H 4768 2155 50  0000 L CNN
F 2 "" V 4740 2190 50  0001 C CNN
F 3 "~" H 4700 2200 50  0001 C CNN
	1    4700 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2350 4700 2400
$Comp
L power:GND #PWR03
U 1 1 61708195
P 4700 2850
F 0 "#PWR03" H 4700 2600 50  0001 C CNN
F 1 "GND" H 4705 2677 50  0000 C CNN
F 2 "" H 4700 2850 50  0001 C CNN
F 3 "" H 4700 2850 50  0001 C CNN
	1    4700 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 2800 4700 2850
$Comp
L Device:Q_Photo_NPN_CE Q2
U 1 1 6170BCD3
P 6450 2600
F 0 "Q2" H 6640 2646 50  0000 L CNN
F 1 "Phototransistor" H 6640 2555 50  0000 L CNN
F 2 "" H 6650 2700 50  0001 C CNN
F 3 "~" H 6450 2600 50  0001 C CNN
	1    6450 2600
	1    0    0    -1  
$EndComp
$Comp
L Device:R_US R4
U 1 1 6170BCD9
P 6550 2200
F 0 "R4" H 6618 2246 50  0000 L CNN
F 1 "R_10k" H 6618 2155 50  0000 L CNN
F 2 "" V 6590 2190 50  0001 C CNN
F 3 "~" H 6550 2200 50  0001 C CNN
	1    6550 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2350 6550 2400
$Comp
L power:GND #PWR05
U 1 1 6170BCE0
P 6550 2850
F 0 "#PWR05" H 6550 2600 50  0001 C CNN
F 1 "GND" H 6555 2677 50  0000 C CNN
F 2 "" H 6550 2850 50  0001 C CNN
F 3 "" H 6550 2850 50  0001 C CNN
	1    6550 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6550 2850 6550 2800
$Comp
L power:+5V #PWR01
U 1 1 6170FAE2
P 3850 2000
F 0 "#PWR01" H 3850 1850 50  0001 C CNN
F 1 "+5V" H 3865 2173 50  0000 C CNN
F 2 "" H 3850 2000 50  0001 C CNN
F 3 "" H 3850 2000 50  0001 C CNN
	1    3850 2000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 2000 3850 2050
Connection ~ 3850 2000
Wire Wire Line
	5850 2050 5850 2000
Connection ~ 5850 2000
Wire Wire Line
	3850 2000 4700 2000
Wire Wire Line
	4700 2050 4700 2000
Connection ~ 4700 2000
Wire Wire Line
	4700 2000 5850 2000
Wire Wire Line
	4700 2350 4300 2350
Wire Wire Line
	4300 2350 4300 3200
Wire Wire Line
	4300 3200 3600 3200
Connection ~ 4700 2350
Wire Wire Line
	6550 2000 6550 2050
Wire Wire Line
	5850 2000 6550 2000
Wire Wire Line
	6550 2350 6250 2350
Wire Wire Line
	6250 2350 6250 3250
Wire Wire Line
	6250 3250 3550 3250
Connection ~ 6550 2350
Text Notes 3900 1700 0    50   ~ 0
Left Reflectance Sensor\n
Text Notes 5650 1700 0    50   ~ 0
Right Reflectance Sensor\n
Wire Wire Line
	3600 2900 3600 3200
Wire Wire Line
	3200 2900 3600 2900
Wire Wire Line
	3550 3250 3550 3000
Wire Wire Line
	3200 3000 3550 3000
Wire Wire Line
	4100 3850 4000 3850
Wire Wire Line
	4000 3850 4000 3750
Wire Wire Line
	4000 3750 3300 3750
Wire Wire Line
	3300 3750 3300 3700
Wire Wire Line
	3300 3700 3200 3700
Wire Wire Line
	4100 3700 4000 3700
Wire Wire Line
	4000 3700 4000 3600
Wire Wire Line
	3200 3600 4000 3600
$Comp
L Motor:Motor_DC M2
U 1 1 6172B918
P 5450 3800
F 0 "M2" H 5608 3796 50  0000 L CNN
F 1 "Motor_DC" H 5608 3705 50  0000 L CNN
F 2 "" H 5450 3710 50  0001 C CNN
F 3 "~" H 5450 3710 50  0001 C CNN
	1    5450 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3950 4100 3950
$Comp
L Motor:Motor_DC M1
U 1 1 6172D958
P 6150 3800
F 0 "M1" H 6308 3796 50  0000 L CNN
F 1 "Motor_DC" H 6308 3705 50  0000 L CNN
F 2 "" H 6150 3710 50  0001 C CNN
F 3 "~" H 6150 3710 50  0001 C CNN
	1    6150 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4700 3850 4800 3850
Wire Wire Line
	4800 3850 4800 3800
Wire Wire Line
	4800 3800 5250 3800
Wire Wire Line
	5250 3800 5250 3500
Wire Wire Line
	5250 3500 5450 3500
Wire Wire Line
	5450 3500 5450 3600
Wire Wire Line
	4700 3700 4800 3700
Wire Wire Line
	4800 3700 4800 3400
Wire Wire Line
	4800 3400 6150 3400
Wire Wire Line
	6150 3400 6150 3600
$Comp
L power:GND #PWR?
U 1 1 61735BE9
P 5450 4100
F 0 "#PWR?" H 5450 3850 50  0001 C CNN
F 1 "GND" H 5455 3927 50  0000 C CNN
F 2 "" H 5450 4100 50  0001 C CNN
F 3 "" H 5450 4100 50  0001 C CNN
	1    5450 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 617361D5
P 6150 4100
F 0 "#PWR?" H 6150 3850 50  0001 C CNN
F 1 "GND" H 6155 3927 50  0000 C CNN
F 2 "" H 6150 4100 50  0001 C CNN
F 3 "" H 6150 4100 50  0001 C CNN
	1    6150 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 61736677
P 4000 4100
F 0 "#PWR?" H 4000 3850 50  0001 C CNN
F 1 "GND" H 4005 3927 50  0000 C CNN
F 2 "" H 4000 4100 50  0001 C CNN
F 3 "" H 4000 4100 50  0001 C CNN
	1    4000 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 4100 4000 4050
Wire Wire Line
	4000 4050 4100 4050
Text Notes 5250 4450 0    50   ~ 0
Left Wheel
Text Notes 5950 4450 0    50   ~ 0
Right Wheel
$Comp
L block_diagram_parts:MotorShield U?
U 1 1 6172772E
P 4400 3650
F 0 "U?" H 4400 3800 50  0000 C CNN
F 1 "MotorShield" H 4400 3500 50  0000 C CNN
F 2 "" H 4400 3650 50  0001 C CNN
F 3 "" H 4400 3650 50  0001 C CNN
	1    4400 3650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3900 4000 3900
Wire Wire Line
	4000 3900 4000 3950
$Comp
L power:+12V #PWR?
U 1 1 61741886
P 3700 3900
F 0 "#PWR?" H 3700 3750 50  0001 C CNN
F 1 "+12V" V 3715 4028 50  0000 L CNN
F 2 "" H 3700 3900 50  0001 C CNN
F 3 "" H 3700 3900 50  0001 C CNN
	1    3700 3900
	0    -1   -1   0   
$EndComp
$EndSCHEMATC
