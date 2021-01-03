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
L DC_DC_5V4A-rescue:TPS565208-Regulator_Switching U1
U 1 1 5FDFB5FA
P 5250 3000
F 0 "U1" H 5250 3367 50  0000 C CNN
F 1 "TPS565208" H 5250 3276 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 5300 2750 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/tps565208.pdf" H 5250 3000 50  0001 C CNN
	1    5250 3000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5FDFCF37
P 3550 3100
F 0 "C1" H 3665 3146 50  0000 L CNN
F 1 "10uF" H 3665 3055 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3588 2950 50  0001 C CNN
F 3 "~" H 3550 3100 50  0001 C CNN
	1    3550 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 5FDFE597
P 3800 3100
F 0 "C2" H 3915 3146 50  0000 L CNN
F 1 "10uF" H 3915 3055 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3838 2950 50  0001 C CNN
F 3 "~" H 3800 3100 50  0001 C CNN
	1    3800 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 5FDFEC02
P 4100 3100
F 0 "C3" H 4215 3146 50  0000 L CNN
F 1 "100nF" H 4215 3055 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4138 2950 50  0001 C CNN
F 3 "~" H 4100 3100 50  0001 C CNN
	1    4100 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5FDFF406
P 7200 3250
F 0 "C6" H 7315 3296 50  0000 L CNN
F 1 "22uF" H 7315 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 7238 3100 50  0001 C CNN
F 3 "~" H 7200 3250 50  0001 C CNN
	1    7200 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5FE00823
P 6850 3250
F 0 "C5" H 6965 3296 50  0000 L CNN
F 1 "22uF" H 6965 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 6888 3100 50  0001 C CNN
F 3 "~" H 6850 3250 50  0001 C CNN
	1    6850 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5FE01292
P 5800 3000
F 0 "C4" V 6052 3000 50  0000 C CNN
F 1 "100nF" V 5961 3000 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 5838 2850 50  0001 C CNN
F 3 "~" H 5800 3000 50  0001 C CNN
	1    5800 3000
	0    -1   -1   0   
$EndComp
$Comp
L Device:L L1
U 1 1 5FE064D3
P 6350 2900
F 0 "L1" V 6540 2900 50  0000 C CNN
F 1 "3.3uH, 17.7.mOhm, 9.2A" V 6449 2900 50  0000 C CNN
F 2 "Inductor_SMD:L_Vishay_IHLP-3232" H 6350 2900 50  0001 C CNN
F 3 "https://www.mouser.com/datasheet/2/427/ihlp-3232dz-01-1763002.pdf~" H 6350 2900 50  0001 C CNN
	1    6350 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5FE080A9
P 5250 3600
F 0 "#PWR0101" H 5250 3350 50  0001 C CNN
F 1 "GND" H 5255 3427 50  0000 C CNN
F 2 "" H 5250 3600 50  0001 C CNN
F 3 "" H 5250 3600 50  0001 C CNN
	1    5250 3600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 3300 5250 3600
Wire Wire Line
	4100 3250 4100 3500
Connection ~ 4100 3500
Wire Wire Line
	3800 3250 3800 3500
Connection ~ 3800 3500
Wire Wire Line
	3800 3500 4100 3500
Wire Wire Line
	3550 3250 3550 3500
Connection ~ 3550 3500
Wire Wire Line
	3550 3500 3800 3500
Wire Wire Line
	5650 2900 5950 2900
Wire Wire Line
	5950 3000 5950 2900
Connection ~ 5950 2900
Wire Wire Line
	5950 2900 6200 2900
Wire Wire Line
	4100 3500 6500 3500
Wire Wire Line
	6850 3400 6850 3500
Connection ~ 6850 3500
Wire Wire Line
	6850 3500 7200 3500
Wire Wire Line
	7200 3400 7200 3500
$Comp
L Device:R R2
U 1 1 5FE100F4
P 6500 3350
F 0 "R2" H 6570 3396 50  0000 L CNN
F 1 "10kOhm" H 6570 3305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6430 3350 50  0001 C CNN
F 3 "~" H 6500 3350 50  0001 C CNN
	1    6500 3350
	1    0    0    -1  
$EndComp
Connection ~ 6500 3500
Wire Wire Line
	6500 3500 6850 3500
$Comp
L Device:R R1
U 1 1 5FE10C6F
P 6500 3050
F 0 "R1" H 6570 3096 50  0000 L CNN
F 1 "54.9kOhm" H 6570 3005 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 6430 3050 50  0001 C CNN
F 3 "~" H 6500 3050 50  0001 C CNN
	1    6500 3050
	1    0    0    -1  
$EndComp
Wire Wire Line
	6500 3200 5650 3200
Wire Wire Line
	5650 3200 5650 3100
Connection ~ 6500 3200
Wire Wire Line
	6500 2900 6850 2900
Wire Wire Line
	7200 2900 7200 3100
Connection ~ 6500 2900
Wire Wire Line
	6850 3100 6850 2900
Connection ~ 6850 2900
Wire Wire Line
	6850 2900 7200 2900
Connection ~ 7200 2900
Wire Wire Line
	4850 2900 4750 2900
Wire Wire Line
	3550 2950 3550 2900
Connection ~ 3550 2900
Wire Wire Line
	3550 2900 3200 2900
Wire Wire Line
	3800 2950 3800 2900
Connection ~ 3800 2900
Wire Wire Line
	3800 2900 3550 2900
Wire Wire Line
	4100 2950 4100 2900
Connection ~ 4100 2900
Wire Wire Line
	4100 2900 3800 2900
Wire Wire Line
	4850 3100 4750 3100
Wire Wire Line
	4750 3100 4750 2900
Connection ~ 4750 2900
Wire Wire Line
	4750 2900 4100 2900
$Comp
L power:+BATT #PWR0102
U 1 1 5FE0924C
P 3200 2900
F 0 "#PWR0102" H 3200 2750 50  0001 C CNN
F 1 "+BATT" H 3215 3073 50  0000 C CNN
F 2 "" H 3200 2900 50  0001 C CNN
F 3 "" H 3200 2900 50  0001 C CNN
	1    3200 2900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 5FE09E37
P 7150 2700
F 0 "#PWR0103" H 7150 2550 50  0001 C CNN
F 1 "+5V" H 7165 2873 50  0000 C CNN
F 2 "" H 7150 2700 50  0001 C CNN
F 3 "" H 7150 2700 50  0001 C CNN
	1    7150 2700
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x03 J1
U 1 1 5FE0C0AF
P 2600 3000
F 0 "J1" H 2518 2675 50  0000 C CNN
F 1 "Conn_01x02" H 2518 2766 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical" H 2600 3000 50  0001 C CNN
F 3 "~" H 2600 3000 50  0001 C CNN
	1    2600 3000
	-1   0    0    1   
$EndComp
Connection ~ 3200 2900
Wire Wire Line
	3000 3000 3000 3500
Wire Wire Line
	3000 3500 3550 3500
$Comp
L power:GND #PWR0104
U 1 1 5FE1224F
P 2950 3000
F 0 "#PWR0104" H 2950 2750 50  0001 C CNN
F 1 "GND" H 2955 2827 50  0000 C CNN
F 2 "" H 2950 3000 50  0001 C CNN
F 3 "" H 2950 3000 50  0001 C CNN
	1    2950 3000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FE12D51
P 3550 3250
F 0 "#PWR?" H 3550 3000 50  0001 C CNN
F 1 "GND" V 3555 3122 50  0000 R CNN
F 2 "" H 3550 3250 50  0001 C CNN
F 3 "" H 3550 3250 50  0001 C CNN
	1    3550 3250
	0    1    1    0   
$EndComp
Connection ~ 3550 3250
Wire Wire Line
	3000 3000 2950 3000
Connection ~ 2950 3000
Wire Wire Line
	2950 3000 2800 3000
Wire Wire Line
	2800 2900 3200 2900
Wire Wire Line
	2800 3100 2800 3250
Wire Wire Line
	2800 3250 2400 3250
Wire Wire Line
	2400 3250 2400 2700
Wire Wire Line
	2400 2700 7150 2700
Wire Wire Line
	7200 2700 7200 2900
Connection ~ 7150 2700
Wire Wire Line
	7150 2700 7200 2700
$EndSCHEMATC
