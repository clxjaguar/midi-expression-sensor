EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "MIDI Pressure Expression Sensor"
Date "2022-02-04"
Rev "1"
Comp "cLx"
Comment1 "http://clx.freeshell.org/midi-expression-sensor.html"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R_POT_TRIM R1
U 1 1 60B0A0DF
P 7350 2150
F 0 "R1" V 7143 2150 50  0000 C CNN
F 1 "2M" V 7234 2150 50  0000 C CNN
F 2 "Potentiometers:Potentiometer_Trimmer_Piher_PT-10v10_Horizontal_Px10.0mm_Py5.0mm" H 7350 2150 50  0001 C CNN
F 3 "~" H 7350 2150 50  0001 C CNN
	1    7350 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	6700 2150 6700 1650
Wire Wire Line
	6700 1650 6850 1650
Connection ~ 6700 1650
Wire Wire Line
	6850 1450 6150 1450
Wire Wire Line
	7350 2300 7350 2350
Wire Wire Line
	7350 2350 7600 2350
Wire Wire Line
	7500 2150 7600 2150
Wire Wire Line
	7600 2150 7600 2350
Wire Wire Line
	7600 2150 7800 2150
Wire Wire Line
	7800 2150 7800 1550
Wire Wire Line
	7800 1550 7450 1550
Connection ~ 7600 2150
$Comp
L power:+5V #PWR03
U 1 1 60B4B53D
P 7050 1250
F 0 "#PWR03" H 7050 1100 50  0001 C CNN
F 1 "+5V" H 7065 1423 50  0000 C CNN
F 2 "" H 7050 1250 50  0001 C CNN
F 3 "" H 7050 1250 50  0001 C CNN
	1    7050 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR011
U 1 1 60B4B5A4
P 7050 1850
F 0 "#PWR011" H 7050 1600 50  0001 C CNN
F 1 "GND" H 7055 1677 50  0001 C CNN
F 2 "" H 7050 1850 50  0001 C CNN
F 3 "" H 7050 1850 50  0001 C CNN
	1    7050 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	7800 1550 8000 1550
Connection ~ 7800 1550
$Comp
L MidiPressureSensor-rescue:MPS20N0040D-MPS20N0040D U2
U 1 1 60B543D0
P 5800 1450
F 0 "U2" H 6050 1750 50  0000 L CNN
F 1 "MPS20N0040D" H 6050 1650 50  0000 L CNN
F 2 "MPS20N0040D:MPS20N0040D" H 5850 1850 50  0001 C CNN
F 3 "" H 5850 1850 50  0001 C CNN
	1    5800 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 60B54652
P 5800 1800
F 0 "#PWR010" H 5800 1550 50  0001 C CNN
F 1 "GND" H 5805 1627 50  0000 C CNN
F 2 "" H 5800 1800 50  0001 C CNN
F 3 "" H 5800 1800 50  0001 C CNN
	1    5800 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	6250 1650 6250 2150
Wire Wire Line
	6250 2150 5350 2150
Wire Wire Line
	5350 2150 5350 1500
Wire Wire Line
	5350 1400 5450 1400
Wire Wire Line
	6250 1650 6700 1650
Wire Wire Line
	5450 1500 5350 1500
Connection ~ 5350 1500
Wire Wire Line
	5350 1500 5350 1400
$Comp
L power:+5V #PWR01
U 1 1 60B54954
P 5800 1100
F 0 "#PWR01" H 5800 950 50  0001 C CNN
F 1 "+5V" H 5815 1273 50  0000 C CNN
F 2 "" H 5800 1100 50  0001 C CNN
F 3 "" H 5800 1100 50  0001 C CNN
	1    5800 1100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 60B591DC
P 6950 2150
F 0 "R2" V 6743 2150 50  0000 C CNN
F 1 "100k" V 6834 2150 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 6880 2150 50  0001 C CNN
F 3 "~" H 6950 2150 50  0001 C CNN
	1    6950 2150
	0    1    1    0   
$EndComp
Wire Wire Line
	7200 2150 7100 2150
Wire Wire Line
	6800 2150 6700 2150
$Comp
L Device:R R3
U 1 1 60B59792
P 8150 1550
F 0 "R3" V 7943 1550 50  0000 C CNN
F 1 "10k" V 8034 1550 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8080 1550 50  0001 C CNN
F 3 "~" H 8150 1550 50  0001 C CNN
	1    8150 1550
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 60B59A7D
P 8400 1800
F 0 "C5" H 8515 1846 50  0000 L CNN
F 1 "100n" H 8515 1755 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 8438 1650 50  0001 C CNN
F 3 "~" H 8400 1800 50  0001 C CNN
	1    8400 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 60B59B57
P 8400 1950
F 0 "#PWR012" H 8400 1700 50  0001 C CNN
F 1 "GND" H 8405 1777 50  0000 C CNN
F 2 "" H 8400 1950 50  0001 C CNN
F 3 "" H 8400 1950 50  0001 C CNN
	1    8400 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 1550 8400 1550
Wire Wire Line
	8400 1650 8400 1550
Connection ~ 8400 1550
Wire Wire Line
	8400 1550 8700 1550
$Comp
L Device:R R6
U 1 1 60B5A09D
P 1200 3150
F 0 "R6" V 993 3150 50  0000 C CNN
F 1 "220" V 1084 3150 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1130 3150 50  0001 C CNN
F 3 "~" H 1200 3150 50  0001 C CNN
	1    1200 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 60B5A175
P 2150 2900
F 0 "R5" H 2081 2854 50  0000 R CNN
F 1 "220" H 2081 2945 50  0000 R CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2080 2900 50  0001 C CNN
F 3 "~" H 2150 2900 50  0001 C CNN
	1    2150 2900
	1    0    0    1   
$EndComp
$Comp
L Connector:DIN-5_180degree J3
U 1 1 60B5A210
P 1750 3250
F 0 "J3" H 1750 2976 50  0000 C CNN
F 1 "KCDX-5S-N2" H 1750 2885 50  0000 C CNN
F 2 "KCDX:KCDX-5S-N2" H 1750 3250 50  0001 C CNN
F 3 "http://www.mouser.com/ds/2/18/40_c091_abd_e-75918.pdf" H 1750 3250 50  0001 C CNN
	1    1750 3250
	-1   0    0    -1  
$EndComp
NoConn ~ 1450 3250
NoConn ~ 2050 3250
$Comp
L power:GND #PWR016
U 1 1 60B5AA51
P 1600 2950
F 0 "#PWR016" H 1600 2700 50  0001 C CNN
F 1 "GND" H 1605 2777 50  0001 C CNN
F 2 "" H 1600 2950 50  0001 C CNN
F 3 "" H 1600 2950 50  0001 C CNN
	1    1600 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 2900 1600 2900
Wire Wire Line
	1600 2900 1600 2950
Wire Wire Line
	1750 2900 1750 2950
$Comp
L power:+5V #PWR014
U 1 1 60B5AD99
P 2150 2750
F 0 "#PWR014" H 2150 2600 50  0001 C CNN
F 1 "+5V" H 2165 2923 50  0000 C CNN
F 2 "" H 2150 2750 50  0001 C CNN
F 3 "" H 2150 2750 50  0001 C CNN
	1    2150 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 3050 2150 3150
Wire Wire Line
	2150 3150 2050 3150
Wire Wire Line
	1450 3150 1350 3150
Wire Wire Line
	1050 3150 900  3150
$Comp
L MidiPressureSensor-rescue:ATmega328P-PU-MCU_Microchip_ATmega U4
U 1 1 60B7BB70
P 6000 4750
F 0 "U4" H 6300 6350 50  0000 R CNN
F 1 "ATmega328P-PU" H 6850 6250 50  0000 R CNN
F 2 "Housings_DIP:DIP-28_W7.62mm_Socket_LongPads" H 6000 4750 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 6000 4750 50  0001 C CNN
	1    6000 4750
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 60B7BCF5
P 5850 3100
F 0 "C6" V 5598 3100 50  0000 C CNN
F 1 "100n" V 5689 3100 50  0000 C CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 5888 2950 50  0001 C CNN
F 3 "~" H 5850 3100 50  0001 C CNN
	1    5850 3100
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR018
U 1 1 60B7BD95
P 5700 3100
F 0 "#PWR018" H 5700 2850 50  0001 C CNN
F 1 "GND" H 5705 2927 50  0000 C CNN
F 2 "" H 5700 3100 50  0001 C CNN
F 3 "" H 5700 3100 50  0001 C CNN
	1    5700 3100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR017
U 1 1 60B7BEA4
P 6000 3000
F 0 "#PWR017" H 6000 2850 50  0001 C CNN
F 1 "+5V" H 6015 3173 50  0000 C CNN
F 2 "" H 6000 3000 50  0001 C CNN
F 3 "" H 6000 3000 50  0001 C CNN
	1    6000 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 3250 6000 3100
Connection ~ 6000 3100
Wire Wire Line
	6000 3100 6000 3000
Wire Wire Line
	6000 3100 6100 3100
Wire Wire Line
	6100 3100 6100 3250
$Comp
L power:GND #PWR029
U 1 1 60B7C69D
P 6000 6250
F 0 "#PWR029" H 6000 6000 50  0001 C CNN
F 1 "GND" H 6005 6077 50  0000 C CNN
F 2 "" H 6000 6250 50  0001 C CNN
F 3 "" H 6000 6250 50  0001 C CNN
	1    6000 6250
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y1
U 1 1 60B7D2EF
P 7700 4250
F 0 "Y1" H 7700 4600 50  0000 C CNN
F 1 "16MHz" H 7700 4500 50  0000 C CNN
F 2 "Crystals:Crystal_HC49-4H_Vertical" H 7700 4250 50  0001 C CNN
F 3 "~" H 7700 4250 50  0001 C CNN
	1    7700 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	7900 4250 7850 4250
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J4
U 1 1 60B7E32C
P 1400 4550
F 0 "J4" H 1450 4750 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 1400 4300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x03_Pitch2.54mm" H 1400 4550 50  0001 C CNN
F 3 "~" H 1400 4550 50  0001 C CNN
	1    1400 4550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR022
U 1 1 60B7E3A2
P 2000 4400
F 0 "#PWR022" H 2000 4250 50  0001 C CNN
F 1 "+5V" H 2015 4573 50  0000 C CNN
F 2 "" H 2000 4400 50  0001 C CNN
F 3 "" H 2000 4400 50  0001 C CNN
	1    2000 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 4450 2000 4450
Wire Wire Line
	2000 4450 2000 4400
$Comp
L power:GND #PWR027
U 1 1 60B7EABA
P 2000 4700
F 0 "#PWR027" H 2000 4450 50  0001 C CNN
F 1 "GND" H 2005 4527 50  0000 C CNN
F 2 "" H 2000 4700 50  0001 C CNN
F 3 "" H 2000 4700 50  0001 C CNN
	1    2000 4700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 4650 2000 4650
Wire Wire Line
	2000 4650 2000 4700
Wire Wire Line
	1700 4550 1900 4550
Wire Wire Line
	1200 4650 1000 4650
Wire Wire Line
	1200 4550 1000 4550
Wire Wire Line
	1200 4450 1000 4450
Text Label 1900 4450 2    50   ~ 0
VCC
Text Label 1900 4550 2    50   ~ 0
MOSI
Text Label 1900 4650 2    50   ~ 0
GND
Text Label 1000 4450 0    50   ~ 0
MISO
Text Label 1000 4550 0    50   ~ 0
SCK
Text Label 1000 4650 0    50   ~ 0
~RST
Wire Wire Line
	6600 5050 6850 5050
Text Label 6850 5050 2    50   ~ 0
~RST
Text Label 7150 3850 2    50   ~ 0
MOSI
Text Label 7150 3950 2    50   ~ 0
MISO
Text Label 7150 4050 2    50   ~ 0
SCK
$Comp
L Connector:Conn_01x06_Female J5
U 1 1 60BB5B4A
P 950 6300
F 0 "J5" H 800 5800 50  0000 C CNN
F 1 "Conn_01x06_Female" H 600 6600 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06_Pitch2.54mm" H 950 6300 50  0001 C CNN
F 3 "~" H 950 6300 50  0001 C CNN
	1    950  6300
	-1   0    0    1   
$EndComp
$Comp
L Device:R R17
U 1 1 60BB5CB2
P 1800 5750
F 0 "R17" H 1730 5704 50  0000 R CNN
F 1 "10k" H 1730 5795 50  0000 R CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 1730 5750 50  0001 C CNN
F 3 "~" H 1800 5750 50  0001 C CNN
	1    1800 5750
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR028
U 1 1 60BB6B31
P 1800 5600
F 0 "#PWR028" H 1800 5450 50  0001 C CNN
F 1 "+5V" H 1815 5773 50  0000 C CNN
F 2 "" H 1800 5600 50  0001 C CNN
F 3 "" H 1800 5600 50  0001 C CNN
	1    1800 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 6000 1400 6000
$Comp
L Device:C C10
U 1 1 60BB852D
P 1550 6000
F 0 "C10" V 1298 6000 50  0000 C CNN
F 1 "100n" V 1389 6000 50  0000 C CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 1588 5850 50  0001 C CNN
F 3 "~" H 1550 6000 50  0001 C CNN
	1    1550 6000
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 6000 1800 6000
Wire Wire Line
	1800 5900 1800 6000
Connection ~ 1800 6000
Wire Wire Line
	1800 6000 2100 6000
Text Label 2100 6000 2    50   ~ 0
~RST
$Comp
L power:GND #PWR030
U 1 1 60BBD8BD
P 1800 6600
F 0 "#PWR030" H 1800 6350 50  0001 C CNN
F 1 "GND" H 1805 6427 50  0000 C CNN
F 2 "" H 1800 6600 50  0001 C CNN
F 3 "" H 1800 6600 50  0001 C CNN
	1    1800 6600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 6500 1800 6500
Wire Wire Line
	1800 6500 1800 6600
Wire Wire Line
	1150 6400 1800 6400
Wire Wire Line
	1800 6400 1800 6500
Connection ~ 1800 6500
Text Label 1200 6000 0    50   ~ 0
DTR
Text Label 1200 6500 0    50   ~ 0
GND
Text Label 1200 6400 0    50   ~ 0
CTS
Wire Wire Line
	1150 6300 2100 6300
Wire Wire Line
	1150 6200 2100 6200
Wire Wire Line
	1150 6100 2100 6100
Wire Notes Line
	800  3900 2300 3900
Wire Notes Line
	2300 3900 2300 5100
Wire Notes Line
	2300 5100 800  5100
Wire Notes Line
	800  5100 800  3900
Wire Notes Line
	800  5300 2300 5300
Wire Notes Line
	2300 5300 2300 7000
Wire Notes Line
	2300 7000 800  7000
Wire Notes Line
	800  7000 800  5300
Text Label 900  3150 0    50   ~ 0
TXD
$Comp
L Amplifier_Operational:TL072 U3
U 1 1 60BCBB10
P 7150 1550
F 0 "U3" H 7200 1850 50  0000 C CNN
F 1 "TL972" H 7250 1750 50  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket_LongPads" H 7150 1550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 7150 1550 50  0001 C CNN
	1    7150 1550
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:TL072 U3
U 3 1 60BCBF85
P 7150 1550
F 0 "U3" H 7108 1550 50  0001 L CNN
F 1 "TL972" H 7108 1505 50  0001 L CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket_LongPads" H 7150 1550 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 7150 1550 50  0001 C CNN
	3    7150 1550
	1    0    0    -1  
$EndComp
$Comp
L Regulator_Linear:L7805 U1
U 1 1 60BCE257
P 3000 1350
F 0 "U1" H 3000 1592 50  0000 C CNN
F 1 "L7805" H 3000 1501 50  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 3025 1200 50  0001 L CIN
F 3 "http://www.st.com/content/ccc/resource/technical/document/datasheet/41/4f/b3/b0/12/d4/47/88/CD00000444.pdf/files/CD00000444.pdf/jcr:content/translations/en.CD00000444.pdf" H 3000 1300 50  0001 C CNN
	1    3000 1350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 60BCF894
P 3900 1250
F 0 "#PWR02" H 3900 1100 50  0001 C CNN
F 1 "+5V" H 3915 1423 50  0000 C CNN
F 2 "" H 3900 1250 50  0001 C CNN
F 3 "" H 3900 1250 50  0001 C CNN
	1    3900 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 1350 3450 1350
Wire Wire Line
	3900 1350 3900 1250
$Comp
L Device:C C3
U 1 1 60BD0EDD
P 3450 1500
F 0 "C3" H 3565 1546 50  0000 L CNN
F 1 "100n" H 3565 1455 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3488 1350 50  0001 C CNN
F 3 "~" H 3450 1500 50  0001 C CNN
	1    3450 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 60BD105A
P 3000 1650
F 0 "#PWR07" H 3000 1400 50  0001 C CNN
F 1 "GND" H 3005 1477 50  0000 C CNN
F 2 "" H 3000 1650 50  0001 C CNN
F 3 "" H 3000 1650 50  0001 C CNN
	1    3000 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 60BD110B
P 3450 1650
F 0 "#PWR08" H 3450 1400 50  0001 C CNN
F 1 "GND" H 3455 1477 50  0000 C CNN
F 2 "" H 3450 1650 50  0001 C CNN
F 3 "" H 3450 1650 50  0001 C CNN
	1    3450 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6600 5250 6850 5250
Wire Wire Line
	6600 5350 6850 5350
Text Label 6850 5250 2    50   ~ 0
RXD
Text Label 6850 5350 2    50   ~ 0
TXD
Text Label 2100 6100 2    50   ~ 0
TXD
Text Label 2100 6200 2    50   ~ 0
RXD
Text Label 2100 6300 2    50   ~ 0
+5V
Text Label 8700 1550 2    50   ~ 0
AN1
Wire Wire Line
	6600 4450 6850 4450
Text Label 6850 4450 2    50   ~ 0
AN0
NoConn ~ 6600 4950
$Comp
L power:PWR_FLAG #FLG01
U 1 1 60BF6F81
P 2550 1300
F 0 "#FLG01" H 2550 1375 50  0001 C CNN
F 1 "PWR_FLAG" H 2550 1474 50  0000 C CNN
F 2 "" H 2550 1300 50  0001 C CNN
F 3 "~" H 2550 1300 50  0001 C CNN
	1    2550 1300
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG02
U 1 1 60BF6FDC
P 1650 1550
F 0 "#FLG02" H 1650 1625 50  0001 C CNN
F 1 "PWR_FLAG" H 1650 1724 50  0001 C CNN
F 2 "" H 1650 1550 50  0001 C CNN
F 3 "~" H 1650 1550 50  0001 C CNN
	1    1650 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 1350 2550 1350
$Comp
L power:GND #PWR04
U 1 1 60BF8817
P 1650 1650
F 0 "#PWR04" H 1650 1400 50  0001 C CNN
F 1 "GND" H 1655 1477 50  0000 C CNN
F 2 "" H 1650 1650 50  0001 C CNN
F 3 "" H 1650 1650 50  0001 C CNN
	1    1650 1650
	1    0    0    -1  
$EndComp
Wire Notes Line
	800  3700 2300 3700
Wire Notes Line
	2300 3700 2300 2400
Wire Notes Line
	2300 2400 800  2400
Wire Notes Line
	800  2400 800  3700
Text Notes 900  5450 0    50   ~ 0
Serial Programming
Text Notes 900  4050 0    50   ~ 0
ISP Programming
Text Notes 900  2550 0    50   ~ 0
MIDI Output Connector
$Comp
L Device:CP C4
U 1 1 60C1C72E
P 3900 1500
F 0 "C4" H 4018 1546 50  0000 L CNN
F 1 "100u" H 4018 1455 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D8.0mm_P5.00mm" H 3938 1350 50  0001 C CNN
F 3 "~" H 3900 1500 50  0001 C CNN
	1    3900 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C1
U 1 1 60C1C8B4
P 2100 1500
F 0 "C1" H 1900 1550 50  0000 L CNN
F 1 "22u" H 1850 1450 50  0000 L CNN
F 2 "Capacitors_THT:CP_Radial_D8.0mm_P5.00mm" H 2138 1350 50  0001 C CNN
F 3 "~" H 2100 1500 50  0001 C CNN
	1    2100 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 60C1FC52
P 2550 1500
F 0 "C2" H 2435 1454 50  0000 R CNN
F 1 "100n" H 2435 1545 50  0000 R CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 2588 1350 50  0001 C CNN
F 3 "~" H 2550 1500 50  0001 C CNN
	1    2550 1500
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR05
U 1 1 60C26238
P 2100 1650
F 0 "#PWR05" H 2100 1400 50  0001 C CNN
F 1 "GND" H 2105 1477 50  0000 C CNN
F 2 "" H 2100 1650 50  0001 C CNN
F 3 "" H 2100 1650 50  0001 C CNN
	1    2100 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3900 1350 3450 1350
Connection ~ 3900 1350
Connection ~ 3450 1350
$Comp
L power:GND #PWR09
U 1 1 60C296F5
P 3900 1650
F 0 "#PWR09" H 3900 1400 50  0001 C CNN
F 1 "GND" H 3905 1477 50  0000 C CNN
F 2 "" H 3900 1650 50  0001 C CNN
F 3 "" H 3900 1650 50  0001 C CNN
	1    3900 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 60C2972C
P 2550 1650
F 0 "#PWR06" H 2550 1400 50  0001 C CNN
F 1 "GND" H 2555 1477 50  0000 C CNN
F 2 "" H 2550 1650 50  0001 C CNN
F 3 "" H 2550 1650 50  0001 C CNN
	1    2550 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 1550 1550 1550
Wire Wire Line
	1650 1550 1650 1650
Wire Wire Line
	1500 1350 1650 1350
Connection ~ 2550 1350
Wire Notes Line
	800  2200 800  800 
Wire Notes Line
	800  800  4300 800 
Wire Notes Line
	4300 800  4300 2200
Wire Notes Line
	4300 2200 800  2200
Text Notes 900  950  0    50   ~ 0
Power Input
Wire Wire Line
	7500 4250 7550 4250
$Comp
L Diode:1N4007 D1
U 1 1 60C625DA
P 1800 1350
F 0 "D1" H 1800 1134 50  0000 C CNN
F 1 "1N4007" H 1800 1225 50  0000 C CNN
F 2 "Diodes_THT:D_DO-41_SOD81_P10.16mm_Horizontal" H 1800 1175 50  0001 C CNN
F 3 "http://www.vishay.com/docs/88503/1n4001.pdf" H 1800 1350 50  0001 C CNN
	1    1800 1350
	-1   0    0    1   
$EndComp
Wire Wire Line
	2550 1300 2550 1350
NoConn ~ 5400 3550
Wire Wire Line
	6600 4550 6850 4550
Text Label 6850 4550 2    50   ~ 0
AN1
$Comp
L Device:R R4
U 1 1 60C73000
P 3550 3000
F 0 "R4" V 3343 3000 50  0000 C CNN
F 1 "10k" V 3434 3000 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3480 3000 50  0001 C CNN
F 3 "~" H 3550 3000 50  0001 C CNN
	1    3550 3000
	0    1    1    0   
$EndComp
$Comp
L Device:C C7
U 1 1 60C73007
P 3800 3250
F 0 "C7" H 3915 3296 50  0000 L CNN
F 1 "100n" H 3915 3205 50  0000 L CNN
F 2 "Capacitors_THT:C_Rect_L7.0mm_W2.5mm_P5.00mm" H 3838 3100 50  0001 C CNN
F 3 "~" H 3800 3250 50  0001 C CNN
	1    3800 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 60C7300E
P 3800 3400
F 0 "#PWR020" H 3800 3150 50  0001 C CNN
F 1 "GND" H 3805 3227 50  0000 C CNN
F 2 "" H 3800 3400 50  0001 C CNN
F 3 "" H 3800 3400 50  0001 C CNN
	1    3800 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 3000 3800 3000
Wire Wire Line
	3800 3100 3800 3000
Connection ~ 3800 3000
Wire Wire Line
	3800 3000 4100 3000
Text Label 4100 3000 2    50   ~ 0
AN0
$Comp
L Connector:Conn_01x03_Male J2
U 1 1 60C75DE1
P 2650 3000
F 0 "J2" H 2750 3200 50  0000 R CNN
F 1 "Conn_01x03_Male" H 3300 3500 50  0000 R CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x03_Pitch2.54mm" H 2650 3000 50  0001 C CNN
F 3 "~" H 2650 3000 50  0001 C CNN
	1    2650 3000
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR019
U 1 1 60C7ACC7
P 3000 3200
F 0 "#PWR019" H 3000 2950 50  0001 C CNN
F 1 "GND" H 3005 3027 50  0000 C CNN
F 2 "" H 3000 3200 50  0001 C CNN
F 3 "" H 3000 3200 50  0001 C CNN
	1    3000 3200
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 3100 3000 3100
Wire Wire Line
	3000 3100 3000 3200
Wire Wire Line
	2850 3000 3400 3000
$Comp
L power:+5V #PWR015
U 1 1 60C896A8
P 3000 2850
F 0 "#PWR015" H 3000 2700 50  0001 C CNN
F 1 "+5V" H 3015 3023 50  0000 C CNN
F 2 "" H 3000 2850 50  0001 C CNN
F 3 "" H 3000 2850 50  0001 C CNN
	1    3000 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2850 2900 3000 2900
Wire Wire Line
	3000 2900 3000 2850
$Comp
L power:GND #PWR013
U 1 1 60C8BFD6
P 9450 2450
F 0 "#PWR013" H 9450 2200 50  0001 C CNN
F 1 "GND" H 9455 2277 50  0001 C CNN
F 2 "" H 9450 2450 50  0001 C CNN
F 3 "" H 9450 2450 50  0001 C CNN
	1    9450 2450
	1    0    0    -1  
$EndComp
$Comp
L Amplifier_Operational:TL072 U3
U 2 1 60BCBF03
P 9850 2250
F 0 "U3" H 9850 2617 50  0000 C CNN
F 1 "TL972" H 9850 2526 50  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm_Socket_LongPads" H 9850 2250 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/tl071.pdf" H 9850 2250 50  0001 C CNN
	2    9850 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 2150 9450 2150
Wire Wire Line
	9450 2150 9450 2350
Wire Wire Line
	9550 2350 9450 2350
Connection ~ 9450 2350
Wire Wire Line
	9450 2350 9450 2450
Wire Notes Line
	2500 2400 4300 2400
Wire Notes Line
	4300 2400 4300 3700
Wire Notes Line
	4300 3700 2500 3700
Wire Notes Line
	2500 3700 2500 2400
Text Notes 2600 2550 0    50   ~ 0
Auxiliary Sensor
NoConn ~ 10150 2250
$Comp
L Device:C C8
U 1 1 60CCF18A
P 7500 4400
F 0 "C8" H 7385 4354 50  0000 R CNN
F 1 "15p" H 7385 4445 50  0000 R CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 7538 4250 50  0001 C CNN
F 3 "~" H 7500 4400 50  0001 C CNN
	1    7500 4400
	1    0    0    1   
$EndComp
$Comp
L Device:C C9
U 1 1 60CCF301
P 7900 4400
F 0 "C9" H 8015 4446 50  0000 L CNN
F 1 "15p" H 8015 4355 50  0000 L CNN
F 2 "Capacitors_THT:C_Disc_D4.3mm_W1.9mm_P5.00mm" H 7938 4250 50  0001 C CNN
F 3 "~" H 7900 4400 50  0001 C CNN
	1    7900 4400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR024
U 1 1 60CD1DC0
P 7500 4550
F 0 "#PWR024" H 7500 4300 50  0001 C CNN
F 1 "GND" H 7505 4377 50  0000 C CNN
F 2 "" H 7500 4550 50  0001 C CNN
F 3 "" H 7500 4550 50  0001 C CNN
	1    7500 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 60CD1E05
P 7900 4550
F 0 "#PWR025" H 7900 4300 50  0001 C CNN
F 1 "GND" H 7905 4377 50  0000 C CNN
F 2 "" H 7900 4550 50  0001 C CNN
F 3 "" H 7900 4550 50  0001 C CNN
	1    7900 4550
	1    0    0    -1  
$EndComp
$Comp
L Connector:Barrel_Jack_Switch J1
U 1 1 60CE5747
P 1200 1450
F 0 "J1" H 1255 1767 50  0000 C CNN
F 1 "Barrel_Jack_Switch" H 1255 1676 50  0000 C CNN
F 2 "Connectors:BARREL_JACK" H 1250 1410 50  0001 C CNN
F 3 "~" H 1250 1410 50  0001 C CNN
	1    1200 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D2
U 1 1 60CECBCA
P 3700 4550
F 0 "D2" H 3738 4433 50  0000 R CNN
F 1 "LED" H 3750 4650 50  0000 R CNN
F 2 "LEDs:LED_D3.0mm" H 3700 4550 50  0001 C CNN
F 3 "~" H 3700 4550 50  0001 C CNN
	1    3700 4550
	-1   0    0    1   
$EndComp
$Comp
L Device:R R8
U 1 1 60CECE4F
P 3300 4550
F 0 "R8" V 3200 4550 50  0000 R CNN
F 1 "1k" V 3300 4600 50  0000 R CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 3230 4550 50  0001 C CNN
F 3 "~" H 3300 4550 50  0001 C CNN
	1    3300 4550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR026
U 1 1 60CEF9FC
P 3950 4650
F 0 "#PWR026" H 3950 4400 50  0001 C CNN
F 1 "GND" H 3955 4477 50  0000 C CNN
F 2 "" H 3950 4650 50  0001 C CNN
F 3 "" H 3950 4650 50  0001 C CNN
	1    3950 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 4550 3950 4550
Wire Wire Line
	3950 4550 3950 4650
Wire Wire Line
	3450 4550 3550 4550
Wire Wire Line
	3150 4550 2950 4550
Text Label 2950 4550 0    50   ~ 0
LED
Wire Wire Line
	6600 4050 7150 4050
Wire Wire Line
	6600 3950 7150 3950
Wire Wire Line
	6600 3850 7150 3850
Wire Wire Line
	6600 3550 6850 3550
Wire Wire Line
	6600 3650 6850 3650
Wire Wire Line
	6600 3750 6850 3750
Text Label 6850 3550 2    50   ~ 0
~SEG_A
Text Label 6850 3650 2    50   ~ 0
~SEG_B
Text Label 6850 3750 2    50   ~ 0
~SEG_C
Text Label 6850 3850 2    50   ~ 0
~SEG_D
Text Label 6850 3950 2    50   ~ 0
~SEG_E
Text Label 6850 4050 2    50   ~ 0
~SEG_F
Wire Wire Line
	6600 5450 6850 5450
Wire Wire Line
	6600 5750 6850 5750
Text Label 6850 5750 2    50   ~ 0
~SEG_G
Text Label 6850 5450 2    50   ~ 0
LED
$Comp
L Display_Character:KCSA02-105 U5
U 1 1 60D301D0
P 9200 5550
F 0 "U5" H 9200 6217 50  0000 C CNN
F 1 "KCSA02-105" H 9200 6126 50  0000 C CNN
F 2 "Displays_7-Segment:7SegmentLED_LTS6760_LTS6780" H 9200 4950 50  0001 C CNN
F 3 "http://www.kingbright.com/attachments/file/psearch/000/00/00/KCSA02-105(Ver.10A).pdf" H 8700 6025 50  0001 L CNN
	1    9200 5550
	1    0    0    -1  
$EndComp
$Comp
L Display_Character:KCSA02-105 U6
U 1 1 60D4E9E5
P 10250 5550
F 0 "U6" H 10250 6217 50  0000 C CNN
F 1 "KCSA02-105" H 10250 6126 50  0000 C CNN
F 2 "Displays_7-Segment:7SegmentLED_LTS6760_LTS6780" H 10250 4950 50  0001 C CNN
F 3 "http://www.kingbright.com/attachments/file/psearch/000/00/00/KCSA02-105(Ver.10A).pdf" H 9750 6025 50  0001 L CNN
	1    10250 5550
	1    0    0    -1  
$EndComp
Text Label 9850 5250 0    50   ~ 0
A
Text Label 8800 5250 0    50   ~ 0
A
Wire Wire Line
	9850 5250 9950 5250
Wire Wire Line
	9850 5350 9950 5350
Wire Wire Line
	9850 5450 9950 5450
Wire Wire Line
	9850 5550 9950 5550
Wire Wire Line
	9850 5650 9950 5650
Wire Wire Line
	9850 5750 9950 5750
Wire Wire Line
	9850 5850 9950 5850
Wire Wire Line
	8900 5250 8800 5250
Entry Wire Line
	9750 5350 9850 5250
Entry Wire Line
	8700 5350 8800 5250
Entry Wire Line
	8700 5450 8800 5350
Entry Wire Line
	8700 5550 8800 5450
Entry Wire Line
	8700 5650 8800 5550
Entry Wire Line
	8700 5750 8800 5650
Entry Wire Line
	8700 5850 8800 5750
Entry Wire Line
	8700 5950 8800 5850
Entry Wire Line
	9750 5950 9850 5850
Entry Wire Line
	9750 5850 9850 5750
Entry Wire Line
	9750 5750 9850 5650
Entry Wire Line
	9750 5650 9850 5550
Entry Wire Line
	9750 5550 9850 5450
Entry Wire Line
	9750 5450 9850 5350
Wire Wire Line
	8800 5350 8900 5350
Wire Wire Line
	8800 5450 8900 5450
Wire Wire Line
	8800 5550 8900 5550
Wire Wire Line
	8800 5650 8900 5650
Wire Wire Line
	8800 5750 8900 5750
Wire Wire Line
	8800 5850 8900 5850
Text Label 8800 5350 0    50   ~ 0
B
Text Label 8800 5450 0    50   ~ 0
C
Text Label 8800 5550 0    50   ~ 0
D
Text Label 8800 5650 0    50   ~ 0
E
Text Label 8800 5750 0    50   ~ 0
F
Text Label 8800 5850 0    50   ~ 0
G
$Comp
L Device:R R10
U 1 1 60DE2070
P 8100 5250
F 0 "R10" V 8150 5450 50  0000 C CNN
F 1 "470" V 8100 5250 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8030 5250 50  0001 C CNN
F 3 "~" H 8100 5250 50  0001 C CNN
	1    8100 5250
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R11
U 1 1 60DE6CAE
P 8100 5350
F 0 "R11" V 8150 5550 50  0000 C CNN
F 1 "470" V 8100 5350 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8030 5350 50  0001 C CNN
F 3 "~" H 8100 5350 50  0001 C CNN
	1    8100 5350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R12
U 1 1 60DE6D08
P 8100 5450
F 0 "R12" V 8150 5650 50  0000 C CNN
F 1 "470" V 8100 5450 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8030 5450 50  0001 C CNN
F 3 "~" H 8100 5450 50  0001 C CNN
	1    8100 5450
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R13
U 1 1 60DE6D60
P 8100 5550
F 0 "R13" V 8150 5750 50  0000 C CNN
F 1 "470" V 8100 5550 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8030 5550 50  0001 C CNN
F 3 "~" H 8100 5550 50  0001 C CNN
	1    8100 5550
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R14
U 1 1 60DE6DBA
P 8100 5650
F 0 "R14" V 8150 5850 50  0000 C CNN
F 1 "470" V 8100 5650 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8030 5650 50  0001 C CNN
F 3 "~" H 8100 5650 50  0001 C CNN
	1    8100 5650
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R15
U 1 1 60DE6E1A
P 8100 5750
F 0 "R15" V 8150 5950 50  0000 C CNN
F 1 "470" V 8100 5750 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8030 5750 50  0001 C CNN
F 3 "~" H 8100 5750 50  0001 C CNN
	1    8100 5750
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R16
U 1 1 60DE6E78
P 8100 5850
F 0 "R16" V 8150 6050 50  0000 C CNN
F 1 "470" V 8100 5850 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 8030 5850 50  0001 C CNN
F 3 "~" H 8100 5850 50  0001 C CNN
	1    8100 5850
	0    -1   -1   0   
$EndComp
NoConn ~ 9950 5950
NoConn ~ 8900 5950
$Comp
L Transistor_BJT:BC327 Q2
U 1 1 60DEF1BA
P 9500 4700
F 0 "Q2" H 9691 4654 50  0000 L CNN
F 1 "BC327" H 9691 4745 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 9700 4625 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/BC327-D.PDF" H 9500 4700 50  0001 L CNN
	1    9500 4700
	1    0    0    1   
$EndComp
$Comp
L Transistor_BJT:BC327 Q1
U 1 1 60DEF423
P 10550 4150
F 0 "Q1" H 10741 4104 50  0000 L CNN
F 1 "BC327" H 10741 4195 50  0000 L CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Wide" H 10750 4075 50  0001 L CIN
F 3 "http://www.onsemi.com/pub_link/Collateral/BC327-D.PDF" H 10550 4150 50  0001 L CNN
	1    10550 4150
	1    0    0    1   
$EndComp
$Comp
L power:+5V #PWR023
U 1 1 60DEF505
P 9600 4500
F 0 "#PWR023" H 9600 4350 50  0001 C CNN
F 1 "+5V" H 9615 4673 50  0000 C CNN
F 2 "" H 9600 4500 50  0001 C CNN
F 3 "" H 9600 4500 50  0001 C CNN
	1    9600 4500
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR021
U 1 1 60DEF58E
P 10650 3950
F 0 "#PWR021" H 10650 3800 50  0001 C CNN
F 1 "+5V" H 10665 4123 50  0000 C CNN
F 2 "" H 10650 3950 50  0001 C CNN
F 3 "" H 10650 3950 50  0001 C CNN
	1    10650 3950
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 5850 9500 5850
Wire Wire Line
	9600 4900 9600 5850
Wire Wire Line
	9500 5950 9600 5950
Wire Wire Line
	9600 5950 9600 5850
Connection ~ 9600 5850
Wire Wire Line
	10550 5950 10650 5950
Wire Wire Line
	10650 4350 10650 5850
Wire Wire Line
	10550 5850 10650 5850
Connection ~ 10650 5850
Wire Wire Line
	10650 5850 10650 5950
$Comp
L Device:R R9
U 1 1 60E193C0
P 9150 4700
F 0 "R9" V 8943 4700 50  0000 C CNN
F 1 "10k" V 9034 4700 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 9080 4700 50  0001 C CNN
F 3 "~" H 9150 4700 50  0001 C CNN
	1    9150 4700
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 60E197CF
P 10200 4150
F 0 "R7" V 9993 4150 50  0000 C CNN
F 1 "10k" V 10084 4150 50  0000 C CNN
F 2 "Resistors_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 10130 4150 50  0001 C CNN
F 3 "~" H 10200 4150 50  0001 C CNN
	1    10200 4150
	0    1    1    0   
$EndComp
Entry Wire Line
	8350 5250 8450 5350
Entry Wire Line
	8350 5350 8450 5450
Entry Wire Line
	8350 5450 8450 5550
Entry Wire Line
	8350 5550 8450 5650
Entry Wire Line
	8350 5650 8450 5750
Entry Wire Line
	8350 5750 8450 5850
Entry Wire Line
	8350 5850 8450 5950
Wire Wire Line
	8250 5250 8350 5250
Wire Wire Line
	8250 5350 8350 5350
Wire Wire Line
	8250 5450 8350 5450
Wire Wire Line
	8250 5550 8350 5550
Wire Wire Line
	8250 5650 8350 5650
Wire Wire Line
	8250 5750 8350 5750
Wire Wire Line
	8250 5850 8350 5850
Wire Bus Line
	8450 6150 8700 6150
Connection ~ 8700 6150
Wire Bus Line
	8700 6150 9750 6150
Text Label 8250 5250 0    50   ~ 0
A
Text Label 8250 5350 0    50   ~ 0
B
Text Label 8250 5450 0    50   ~ 0
C
Text Label 8250 5550 0    50   ~ 0
D
Text Label 8250 5650 0    50   ~ 0
E
Text Label 8250 5750 0    50   ~ 0
F
Text Label 8250 5850 0    50   ~ 0
G
Text Label 9850 5350 0    50   ~ 0
B
Text Label 9850 5450 0    50   ~ 0
C
Text Label 9850 5550 0    50   ~ 0
D
Text Label 9850 5650 0    50   ~ 0
E
Text Label 9850 5750 0    50   ~ 0
F
Text Label 9850 5850 0    50   ~ 0
G
Wire Wire Line
	7550 5250 7950 5250
Wire Wire Line
	7550 5350 7950 5350
Text Label 7550 5250 0    50   ~ 0
~SEG_A
Wire Wire Line
	7550 5450 7950 5450
Wire Wire Line
	7550 5550 7950 5550
Wire Wire Line
	7550 5650 7950 5650
Wire Wire Line
	7550 5750 7950 5750
Wire Wire Line
	7550 5850 7950 5850
Text Label 7550 5350 0    50   ~ 0
~SEG_B
Text Label 7550 5450 0    50   ~ 0
~SEG_C
Text Label 7550 5550 0    50   ~ 0
~SEG_D
Text Label 7550 5650 0    50   ~ 0
~SEG_E
Text Label 7550 5750 0    50   ~ 0
~SEG_F
Text Label 7550 5850 0    50   ~ 0
~SEG_G
Wire Wire Line
	10050 4150 9800 4150
Wire Wire Line
	9000 4700 8750 4700
Text Label 9800 4150 0    50   ~ 0
~DIGIT0
Text Label 8750 4700 0    50   ~ 0
~DIGIT1
Wire Wire Line
	6600 5950 6850 5950
Text Label 6850 5950 2    50   ~ 0
~DIGIT0
Wire Wire Line
	6600 5850 6850 5850
Text Label 6850 5850 2    50   ~ 0
~DIGIT1
Wire Wire Line
	6600 4250 7500 4250
Connection ~ 7500 4250
Wire Wire Line
	6600 4150 7500 4150
Wire Wire Line
	7500 4150 7500 4050
Wire Wire Line
	7500 4050 7900 4050
Connection ~ 7900 4250
Wire Wire Line
	7900 4050 7900 4250
Wire Wire Line
	1500 1450 1550 1450
Wire Wire Line
	1550 1450 1550 1550
Connection ~ 1550 1550
Wire Wire Line
	1550 1550 1650 1550
Connection ~ 1650 1550
Wire Wire Line
	1950 1350 2100 1350
Connection ~ 2100 1350
Wire Wire Line
	2100 1350 2550 1350
Text Notes 1200 6300 0    50   ~ 0
VCC
Text Notes 1200 6200 0    50   ~ 0
TX
Text Notes 1200 6100 0    50   ~ 0
RX
NoConn ~ 6600 5550
NoConn ~ 6600 5650
$Comp
L Switch:SW_Push SW1
U 1 1 60FA2ACF
P 3100 5400
F 0 "SW1" H 3100 5685 50  0000 C CNN
F 1 "SW_Push" H 3100 5594 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_TH_Tactile_Omron_B3F-10xx" H 3100 5600 50  0001 C CNN
F 3 "" H 3100 5600 50  0001 C CNN
	1    3100 5400
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 60FA2BDC
P 3100 5850
F 0 "SW2" H 3100 6135 50  0000 C CNN
F 1 "SW_Push" H 3100 6044 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_TH_Tactile_Omron_B3F-10xx" H 3100 6050 50  0001 C CNN
F 3 "" H 3100 6050 50  0001 C CNN
	1    3100 5850
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 60FA2C6E
P 3100 6300
F 0 "SW3" H 3100 6585 50  0000 C CNN
F 1 "SW_Push" H 3100 6494 50  0000 C CNN
F 2 "Buttons_Switches_THT:SW_TH_Tactile_Omron_B3F-10xx" H 3100 6500 50  0001 C CNN
F 3 "" H 3100 6500 50  0001 C CNN
	1    3100 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 60FB2592
P 2800 6400
F 0 "#PWR0101" H 2800 6150 50  0001 C CNN
F 1 "GND" H 2805 6227 50  0000 C CNN
F 2 "" H 2800 6400 50  0001 C CNN
F 3 "" H 2800 6400 50  0001 C CNN
	1    2800 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 5400 2800 5400
Wire Wire Line
	2800 5400 2800 5850
Wire Wire Line
	2900 6300 2800 6300
Connection ~ 2800 6300
Wire Wire Line
	2800 6300 2800 6400
Wire Wire Line
	2900 5850 2800 5850
Connection ~ 2800 5850
Wire Wire Line
	2800 5850 2800 6300
Wire Wire Line
	3300 5400 3500 5400
Wire Wire Line
	3300 5850 3500 5850
Wire Wire Line
	3300 6300 3500 6300
Text Label 3500 5400 2    50   ~ 0
~SW1
Text Label 3500 5850 2    50   ~ 0
~SW2
Text Label 3500 6300 2    50   ~ 0
~SW3
Wire Wire Line
	6600 4850 6850 4850
Wire Wire Line
	6600 4750 6850 4750
Wire Wire Line
	6600 4650 6850 4650
Text Label 6850 4650 2    50   ~ 0
~SW3
Text Label 6850 4750 2    50   ~ 0
~SW2
Text Label 6850 4850 2    50   ~ 0
~SW1
Wire Bus Line
	8700 5350 8700 6150
Wire Bus Line
	9750 5350 9750 6150
Wire Bus Line
	8450 5350 8450 6150
$EndSCHEMATC