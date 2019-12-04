EESchema Schematic File Version 5
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Controller für DomLicht und Stromversorgung des RasPis"
Date "2019-11-30"
Rev "0.1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
Comment5 ""
Comment6 ""
Comment7 ""
Comment8 ""
Comment9 ""
$EndDescr
$Comp
L Connector:Raspberry_Pi_2_3 J102
U 1 1 5DE252CE
P 2200 5600
F 0 "J102" H 2600 6950 50  0000 C CNN
F 1 "Raspberry_Pi_2_3" H 2850 6850 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Horizontal" H 2200 5600 50  0001 C CNN
F 3 "https://www.raspberrypi.org/documentation/hardware/raspberrypi/schematics/rpi_SCH_3bplus_1p0_reduced.pdf" H 2200 5600 50  0001 C CNN
	1    2200 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5DE2EA60
P 1800 7050
F 0 "#PWR0105" H 1800 6800 50  0001 C CNN
F 1 "GND" H 1805 6877 50  0000 C CNN
F 2 "" H 1800 7050 50  0001 C CNN
F 3 "" H 1800 7050 50  0001 C CNN
	1    1800 7050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1800 7050 1800 6950
Wire Wire Line
	1800 6950 1900 6950
Wire Wire Line
	2400 6950 2400 6900
Connection ~ 1800 6950
Wire Wire Line
	1800 6950 1800 6900
Wire Wire Line
	2400 6950 2500 6950
Wire Wire Line
	2500 6950 2500 6900
Connection ~ 2400 6950
Wire Wire Line
	2300 6900 2300 6950
Connection ~ 2300 6950
Wire Wire Line
	2300 6950 2400 6950
Wire Wire Line
	2200 6900 2200 6950
Connection ~ 2200 6950
Wire Wire Line
	2200 6950 2300 6950
Wire Wire Line
	2100 6900 2100 6950
Connection ~ 2100 6950
Wire Wire Line
	2100 6950 2200 6950
Wire Wire Line
	2000 6900 2000 6950
Connection ~ 2000 6950
Wire Wire Line
	2000 6950 2100 6950
Wire Wire Line
	1900 6900 1900 6950
Connection ~ 1900 6950
Wire Wire Line
	1900 6950 2000 6950
$Comp
L power:+5V #PWR0110
U 1 1 5DE2FC41
P 2000 4000
F 0 "#PWR0110" H 2000 3850 50  0001 C CNN
F 1 "+5V" H 2015 4173 50  0000 C CNN
F 2 "" H 2000 4000 50  0001 C CNN
F 3 "" H 2000 4000 50  0001 C CNN
	1    2000 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2000 4000 2000 4200
$Comp
L power:+3.3V #PWR0111
U 1 1 5DE30ED1
P 2300 4000
F 0 "#PWR0111" H 2300 3850 50  0001 C CNN
F 1 "+3.3V" H 2315 4173 50  0000 C CNN
F 2 "" H 2300 4000 50  0001 C CNN
F 3 "" H 2300 4000 50  0001 C CNN
	1    2300 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 4300 2300 4200
Wire Wire Line
	2400 4300 2400 4200
Wire Wire Line
	2400 4200 2300 4200
Connection ~ 2300 4200
Wire Wire Line
	2300 4200 2300 4000
Wire Wire Line
	2100 4300 2100 4200
Wire Wire Line
	2100 4200 2000 4200
Connection ~ 2000 4200
Wire Wire Line
	2000 4200 2000 4300
$Comp
L Connector:Conn_01x02_Male J101
U 1 1 5DE3ABE1
P 1150 2650
F 0 "J101" H 1258 2831 50  0000 C CNN
F 1 "Conn_01x02_Male" H 1258 2740 50  0000 C CNN
F 2 "Connector_JST:JST_XH_S02B-XH-A-1_1x02_P2.50mm_Horizontal" H 1150 2650 50  0001 C CNN
F 3 "~" H 1150 2650 50  0001 C CNN
	1    1150 2650
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR0109
U 1 1 5DE3C300
P 1950 2650
F 0 "#PWR0109" H 1950 2500 50  0001 C CNN
F 1 "+24V" V 1965 2778 50  0000 L CNN
F 2 "" H 1950 2650 50  0001 C CNN
F 3 "" H 1950 2650 50  0001 C CNN
	1    1950 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	1650 2650 1350 2650
$Comp
L power:GND #PWR0103
U 1 1 5DE3DFA3
P 1450 2850
F 0 "#PWR0103" H 1450 2600 50  0001 C CNN
F 1 "GND" H 1455 2677 50  0000 C CNN
F 2 "" H 1450 2850 50  0001 C CNN
F 3 "" H 1450 2850 50  0001 C CNN
	1    1450 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 2850 1450 2750
Wire Wire Line
	1450 2750 1350 2750
$Comp
L 74xx:74HC595 U101
U 1 1 5DE3E69A
P 8650 1400
F 0 "U101" H 8900 2131 50  0000 C CNN
F 1 "74HC595" H 8900 2040 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_5.3x10.2mm_P1.27mm" H 8650 1400 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8650 1400 50  0001 C CNN
	1    8650 1400
	1    0    0    -1  
$EndComp
Text Label 8250 1000 2    50   ~ 0
SPI_MOSI
$Comp
L power:+3.3V #PWR0130
U 1 1 5DE41E82
P 8650 800
F 0 "#PWR0130" H 8650 650 50  0001 C CNN
F 1 "+3.3V" H 8665 973 50  0000 C CNN
F 2 "" H 8650 800 50  0001 C CNN
F 3 "" H 8650 800 50  0001 C CNN
	1    8650 800 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 5DE42AEA
P 8650 2100
F 0 "#PWR0131" H 8650 1850 50  0001 C CNN
F 1 "GND" H 8655 1927 50  0000 C CNN
F 2 "" H 8650 2100 50  0001 C CNN
F 3 "" H 8650 2100 50  0001 C CNN
	1    8650 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 5DE431D6
P 8250 1600
F 0 "#PWR0125" H 8250 1350 50  0001 C CNN
F 1 "GND" V 8255 1472 50  0000 R CNN
F 2 "" H 8250 1600 50  0001 C CNN
F 3 "" H 8250 1600 50  0001 C CNN
	1    8250 1600
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0124
U 1 1 5DE43825
P 8250 1300
F 0 "#PWR0124" H 8250 1150 50  0001 C CNN
F 1 "+3.3V" V 8265 1428 50  0000 L CNN
F 2 "" H 8250 1300 50  0001 C CNN
F 3 "" H 8250 1300 50  0001 C CNN
	1    8250 1300
	0    -1   -1   0   
$EndComp
Text Label 8250 1200 2    50   ~ 0
SPI_SCK
Text Label 8250 1500 2    50   ~ 0
SPI_CS
Text Label 8250 3250 2    50   ~ 0
SPI_CS
$Comp
L power:GND #PWR0133
U 1 1 5DE4458E
P 8650 3850
F 0 "#PWR0133" H 8650 3600 50  0001 C CNN
F 1 "GND" H 8655 3677 50  0000 C CNN
F 2 "" H 8650 3850 50  0001 C CNN
F 3 "" H 8650 3850 50  0001 C CNN
	1    8650 3850
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0126
U 1 1 5DE4459B
P 8250 3050
F 0 "#PWR0126" H 8250 2900 50  0001 C CNN
F 1 "+3.3V" V 8265 3178 50  0000 L CNN
F 2 "" H 8250 3050 50  0001 C CNN
F 3 "" H 8250 3050 50  0001 C CNN
	1    8250 3050
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0132
U 1 1 5DE445A8
P 8650 2550
F 0 "#PWR0132" H 8650 2400 50  0001 C CNN
F 1 "+3.3V" H 8665 2723 50  0000 C CNN
F 2 "" H 8650 2550 50  0001 C CNN
F 3 "" H 8650 2550 50  0001 C CNN
	1    8650 2550
	1    0    0    -1  
$EndComp
Text Label 8250 2950 2    50   ~ 0
SPI_SCK
$Comp
L power:GND #PWR0127
U 1 1 5DE445B6
P 8250 3350
F 0 "#PWR0127" H 8250 3100 50  0001 C CNN
F 1 "GND" V 8255 3222 50  0000 R CNN
F 2 "" H 8250 3350 50  0001 C CNN
F 3 "" H 8250 3350 50  0001 C CNN
	1    8250 3350
	0    1    1    0   
$EndComp
$Comp
L 74xx:74HC595 U102
U 1 1 5DE445D3
P 8650 3150
F 0 "U102" H 8900 3881 50  0000 C CNN
F 1 "74HC595" H 8900 3790 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_5.3x10.2mm_P1.27mm" H 8650 3150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8650 3150 50  0001 C CNN
	1    8650 3150
	1    0    0    -1  
$EndComp
Text Label 8250 5000 2    50   ~ 0
SPI_CS
$Comp
L 74xx:74HC595 U103
U 1 1 5DE47E23
P 8650 4900
F 0 "U103" H 8900 5631 50  0000 C CNN
F 1 "74HC595" H 8900 5540 50  0000 C CNN
F 2 "Package_SO:SOIC-16W_5.3x10.2mm_P1.27mm" H 8650 4900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/sn74hc595.pdf" H 8650 4900 50  0001 C CNN
	1    8650 4900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0135
U 1 1 5DE47E30
P 8650 5600
F 0 "#PWR0135" H 8650 5350 50  0001 C CNN
F 1 "GND" H 8655 5427 50  0000 C CNN
F 2 "" H 8650 5600 50  0001 C CNN
F 3 "" H 8650 5600 50  0001 C CNN
	1    8650 5600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0129
U 1 1 5DE47E3D
P 8250 5100
F 0 "#PWR0129" H 8250 4850 50  0001 C CNN
F 1 "GND" V 8255 4972 50  0000 R CNN
F 2 "" H 8250 5100 50  0001 C CNN
F 3 "" H 8250 5100 50  0001 C CNN
	1    8250 5100
	0    1    1    0   
$EndComp
Text Label 8250 4700 2    50   ~ 0
SPI_SCK
$Comp
L power:+3.3V #PWR0128
U 1 1 5DE47E4B
P 8250 4800
F 0 "#PWR0128" H 8250 4650 50  0001 C CNN
F 1 "+3.3V" V 8265 4928 50  0000 L CNN
F 2 "" H 8250 4800 50  0001 C CNN
F 3 "" H 8250 4800 50  0001 C CNN
	1    8250 4800
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR0134
U 1 1 5DE47E59
P 8650 4300
F 0 "#PWR0134" H 8650 4150 50  0001 C CNN
F 1 "+3.3V" H 8665 4473 50  0000 C CNN
F 2 "" H 8650 4300 50  0001 C CNN
F 3 "" H 8650 4300 50  0001 C CNN
	1    8650 4300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9050 1900 9200 1900
Wire Wire Line
	9200 1900 9200 2300
Wire Wire Line
	9200 2300 8100 2300
Wire Wire Line
	8100 2750 8250 2750
Wire Wire Line
	8100 2300 8100 2750
Wire Wire Line
	9050 3650 9200 3650
Wire Wire Line
	9200 3650 9200 4050
Wire Wire Line
	9200 4050 8100 4050
Wire Wire Line
	8100 4050 8100 4500
Wire Wire Line
	8100 4500 8250 4500
Text Label 3100 5800 0    50   ~ 0
SPI_CS
Text Label 3100 6100 0    50   ~ 0
SPI_SCK
Text Label 3100 6000 0    50   ~ 0
SPI_MOSI
Wire Wire Line
	3100 5800 3000 5800
Wire Wire Line
	3100 6000 3000 6000
Wire Wire Line
	3000 6100 3100 6100
NoConn ~ 3000 5900
NoConn ~ 9050 5400
$Comp
L Regulator_Switching:TPS5430DDA U1
U 1 1 5CAA1F8C
P 3600 1400
F 0 "U1" H 3600 1867 50  0000 C CNN
F 1 "TPS5430DDA" H 3600 1776 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 3650 1050 50  0001 L CIN
F 3 "http://www.ti.com/lit/ds/symlink/tps5430.pdf" H 3600 1400 50  0001 C CNN
	1    3600 1400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0101
U 1 1 5CAA263A
P 3500 1800
F 0 "#PWR0101" H 3500 1550 50  0001 C CNN
F 1 "GND" H 3505 1627 50  0000 C CNN
F 2 "" H 3500 1800 50  0001 C CNN
F 3 "" H 3500 1800 50  0001 C CNN
	1    3500 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 5CAA3B41
P 4250 1200
F 0 "C4" V 3998 1200 50  0000 C CNN
F 1 "10n 50V X7R" V 4089 1200 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4288 1050 50  0001 C CNN
F 3 "~" H 4250 1200 50  0001 C CNN
	1    4250 1200
	0    1    1    0   
$EndComp
Wire Wire Line
	4100 1400 4400 1400
Wire Wire Line
	4400 1400 4400 1200
$Comp
L Device:D_Schottky D1
U 1 1 5CAA4711
P 4750 1350
F 0 "D1" V 4704 1429 50  0000 L CNN
F 1 "STPS340U" V 4795 1429 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 4750 1350 50  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/CD00000844.pdf" H 4750 1350 50  0001 C CNN
	1    4750 1350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5CAA4CD7
P 4750 1500
F 0 "#PWR0104" H 4750 1250 50  0001 C CNN
F 1 "GND" H 4755 1327 50  0000 C CNN
F 2 "" H 4750 1500 50  0001 C CNN
F 3 "" H 4750 1500 50  0001 C CNN
	1    4750 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:L L1
U 1 1 5CAA5200
P 5250 1200
F 0 "L1" V 5069 1200 50  0000 C CNN
F 1 "33µH" V 5160 1200 50  0000 C CNN
F 2 "Inductor_SMD:L_Wuerth_HCI-1040" H 5250 1200 50  0001 C CNN
F 3 "https://www.bourns.com/pdfs/SRU1048.pdf" H 5250 1200 50  0001 C CNN
	1    5250 1200
	0    1    1    0   
$EndComp
$Comp
L Device:C C5
U 1 1 5CAA5546
P 5600 1350
F 0 "C5" H 5715 1396 50  0000 L CNN
F 1 "1µ 16V" H 5715 1305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5638 1200 50  0001 C CNN
F 3 "~" H 5600 1350 50  0001 C CNN
	1    5600 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C6
U 1 1 5CAA5B00
P 6700 1350
F 0 "C6" H 6818 1396 50  0000 L CNN
F 1 "330µF 6,3V" H 6818 1305 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 6738 1200 50  0001 C CNN
F 3 "http://www.farnell.com/datasheets/2002520.pdf" H 6700 1350 50  0001 C CNN
	1    6700 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5CAA6225
P 7350 1350
F 0 "R3" H 7420 1396 50  0000 L CNN
F 1 "10k" H 7420 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7280 1350 50  0001 C CNN
F 3 "~" H 7350 1350 50  0001 C CNN
	1    7350 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4100 1600 4550 1600
Wire Wire Line
	4550 1600 4550 1750
Wire Wire Line
	4400 1200 4750 1200
Connection ~ 4400 1200
Connection ~ 4750 1200
Wire Wire Line
	4750 1200 5100 1200
Wire Wire Line
	5400 1200 5600 1200
Connection ~ 5600 1200
$Comp
L power:GND #PWR0106
U 1 1 5CAA78AA
P 5600 1500
F 0 "#PWR0106" H 5600 1250 50  0001 C CNN
F 1 "GND" H 5605 1327 50  0000 C CNN
F 2 "" H 5600 1500 50  0001 C CNN
F 3 "" H 5600 1500 50  0001 C CNN
	1    5600 1500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5CAA7B10
P 6700 1500
F 0 "#PWR0107" H 6700 1250 50  0001 C CNN
F 1 "GND" H 6705 1327 50  0000 C CNN
F 2 "" H 6700 1500 50  0001 C CNN
F 3 "" H 6700 1500 50  0001 C CNN
	1    6700 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 5CAA940A
P 7350 2000
F 0 "R5" H 7420 2046 50  0000 L CNN
F 1 "3,24k" H 7420 1955 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 7280 2000 50  0001 C CNN
F 3 "~" H 7350 2000 50  0001 C CNN
	1    7350 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5CAA97FF
P 7350 2150
F 0 "#PWR0108" H 7350 1900 50  0001 C CNN
F 1 "GND" H 7355 1977 50  0000 C CNN
F 2 "" H 7350 2150 50  0001 C CNN
F 3 "" H 7350 2150 50  0001 C CNN
	1    7350 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 1750 7350 1850
$Comp
L Device:CP C1
U 1 1 5CAAFAB5
P 1700 1350
F 0 "C1" H 1818 1396 50  0000 L CNN
F 1 "100µF 25V" H 1818 1305 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 1738 1200 50  0001 C CNN
F 3 "~" H 1700 1350 50  0001 C CNN
	1    1700 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5CAB2514
P 1200 1350
F 0 "R1" H 1270 1396 50  0000 L CNN
F 1 "100k" H 1270 1305 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1130 1350 50  0001 C CNN
F 3 "~" H 1200 1350 50  0001 C CNN
	1    1200 1350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5CAB2A0F
P 1200 1650
F 0 "R2" H 1270 1696 50  0000 L CNN
F 1 "100k" H 1270 1605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 1130 1650 50  0001 C CNN
F 3 "~" H 1200 1650 50  0001 C CNN
	1    1200 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1500 1700 1500
Connection ~ 1200 1500
$Comp
L power:GND #PWR0114
U 1 1 5CAB3C16
P 1700 1800
F 0 "#PWR0114" H 1700 1550 50  0001 C CNN
F 1 "GND" H 1705 1627 50  0000 C CNN
F 2 "" H 1700 1800 50  0001 C CNN
F 3 "" H 1700 1800 50  0001 C CNN
	1    1700 1800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5CAB4074
P 1200 1800
F 0 "#PWR0115" H 1200 1550 50  0001 C CNN
F 1 "GND" H 1205 1627 50  0000 C CNN
F 2 "" H 1200 1800 50  0001 C CNN
F 3 "" H 1200 1800 50  0001 C CNN
	1    1200 1800
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C2
U 1 1 5CAB4673
P 1700 1650
F 0 "C2" H 1818 1696 50  0000 L CNN
F 1 "100µF 25V" H 1818 1605 50  0000 L CNN
F 2 "Capacitor_SMD:CP_Elec_6.3x7.7" H 1738 1500 50  0001 C CNN
F 3 "~" H 1700 1650 50  0001 C CNN
	1    1700 1650
	1    0    0    -1  
$EndComp
Connection ~ 1700 1500
$Comp
L Device:C C3
U 1 1 5CAC2C7C
P 2450 1350
F 0 "C3" H 2565 1396 50  0000 L CNN
F 1 "100n 50V" H 2565 1305 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2488 1200 50  0001 C CNN
F 3 "~" H 2450 1350 50  0001 C CNN
	1    2450 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5CAC34B8
P 2450 1500
F 0 "#PWR0116" H 2450 1250 50  0001 C CNN
F 1 "GND" H 2455 1327 50  0000 C CNN
F 2 "" H 2450 1500 50  0001 C CNN
F 3 "" H 2450 1500 50  0001 C CNN
	1    2450 1500
	1    0    0    -1  
$EndComp
NoConn ~ 3100 1600
Text Notes 6750 2000 0    50   ~ 0
3,24k: 5,0V
Connection ~ 7350 1750
$Comp
L Device:LED D2
U 1 1 5CAD3943
P 4000 6150
F 0 "D2" V 4039 6032 50  0000 R CNN
F 1 "grün" V 3948 6032 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 4000 6150 50  0001 C CNN
F 3 "~" H 4000 6150 50  0001 C CNN
	1    4000 6150
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R7
U 1 1 5CAD4957
P 4000 6550
F 0 "R7" H 4070 6596 50  0000 L CNN
F 1 "470" H 4070 6505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3930 6550 50  0001 C CNN
F 3 "~" H 4000 6550 50  0001 C CNN
	1    4000 6550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5CAD4AC3
P 4000 6700
F 0 "#PWR01" H 4000 6450 50  0001 C CNN
F 1 "GND" H 4005 6527 50  0000 C CNN
F 2 "" H 4000 6700 50  0001 C CNN
F 3 "" H 4000 6700 50  0001 C CNN
	1    4000 6700
	1    0    0    -1  
$EndComp
NoConn ~ 3600 1800
$Comp
L Device:C C7
U 1 1 5CB6E88A
P 6100 1350
F 0 "C7" H 6215 1396 50  0000 L CNN
F 1 "100n 16V" H 6215 1305 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6138 1200 50  0001 C CNN
F 3 "~" H 6100 1350 50  0001 C CNN
	1    6100 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 1500 7350 1750
Connection ~ 6700 1200
Wire Wire Line
	6700 1200 7350 1200
Wire Wire Line
	5600 1200 6100 1200
Connection ~ 6100 1200
Wire Wire Line
	6100 1200 6700 1200
$Comp
L power:GND #PWR02
U 1 1 5CB70AE3
P 6100 1500
F 0 "#PWR02" H 6100 1250 50  0001 C CNN
F 1 "GND" H 6105 1327 50  0000 C CNN
F 2 "" H 6100 1500 50  0001 C CNN
F 3 "" H 6100 1500 50  0001 C CNN
	1    6100 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D101
U 1 1 5DEB3539
P 1800 2650
F 0 "D101" V 1754 2729 50  0000 L CNN
F 1 "STPS340U" V 1845 2729 50  0000 L CNN
F 2 "Diode_SMD:D_SMB" H 1800 2650 50  0001 C CNN
F 3 "https://www.st.com/resource/en/datasheet/CD00000844.pdf" H 1800 2650 50  0001 C CNN
	1    1800 2650
	-1   0    0    1   
$EndComp
Wire Wire Line
	4550 1750 7350 1750
Wire Wire Line
	4000 6300 4000 6400
$Comp
L power:+5V #PWR0119
U 1 1 5DEBEDAD
P 4000 6000
F 0 "#PWR0119" H 4000 5850 50  0001 C CNN
F 1 "+5V" H 4015 6173 50  0000 C CNN
F 2 "" H 4000 6000 50  0001 C CNN
F 3 "" H 4000 6000 50  0001 C CNN
	1    4000 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:R R103
U 1 1 5DEBF9A7
P 4500 6550
F 0 "R103" H 4570 6596 50  0000 L CNN
F 1 "220" H 4570 6505 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 4430 6550 50  0001 C CNN
F 3 "~" H 4500 6550 50  0001 C CNN
	1    4500 6550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 5DEBF9B4
P 4500 6700
F 0 "#PWR0121" H 4500 6450 50  0001 C CNN
F 1 "GND" H 4505 6527 50  0000 C CNN
F 2 "" H 4500 6700 50  0001 C CNN
F 3 "" H 4500 6700 50  0001 C CNN
	1    4500 6700
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D103
U 1 1 5DEBF9C2
P 4500 6150
F 0 "D103" V 4539 6032 50  0000 R CNN
F 1 "grün" V 4448 6032 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 4500 6150 50  0001 C CNN
F 3 "~" H 4500 6150 50  0001 C CNN
	1    4500 6150
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4500 6300 4500 6400
$Comp
L power:+3.3V #PWR0120
U 1 1 5DEC1D75
P 4500 6000
F 0 "#PWR0120" H 4500 5850 50  0001 C CNN
F 1 "+3.3V" H 4515 6173 50  0000 C CNN
F 2 "" H 4500 6000 50  0001 C CNN
F 3 "" H 4500 6000 50  0001 C CNN
	1    4500 6000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3100 1200 2450 1200
Connection ~ 1700 1200
Wire Wire Line
	1700 1200 1200 1200
Connection ~ 2450 1200
Wire Wire Line
	2450 1200 1700 1200
$Comp
L power:GND #PWR0165
U 1 1 5DEDC662
P 10500 6300
F 0 "#PWR0165" H 10500 6050 50  0001 C CNN
F 1 "GND" H 10505 6127 50  0000 C CNN
F 2 "" H 10500 6300 50  0001 C CNN
F 3 "" H 10500 6300 50  0001 C CNN
	1    10500 6300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C104
U 1 1 5DEDC670
P 10500 6150
F 0 "C104" H 10615 6196 50  0000 L CNN
F 1 "1µ 16V" H 10615 6105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10538 6000 50  0001 C CNN
F 3 "~" H 10500 6150 50  0001 C CNN
	1    10500 6150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0164
U 1 1 5DEDE3BD
P 10500 6000
F 0 "#PWR0164" H 10500 5850 50  0001 C CNN
F 1 "+3.3V" H 10515 6173 50  0000 C CNN
F 2 "" H 10500 6000 50  0001 C CNN
F 3 "" H 10500 6000 50  0001 C CNN
	1    10500 6000
	1    0    0    -1  
$EndComp
$Comp
L Device:C C103
U 1 1 5DEDED9F
P 10000 6150
F 0 "C103" H 10115 6196 50  0000 L CNN
F 1 "1µ 16V" H 10115 6105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 10038 6000 50  0001 C CNN
F 3 "~" H 10000 6150 50  0001 C CNN
	1    10000 6150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0150
U 1 1 5DEDEDAC
P 10000 6000
F 0 "#PWR0150" H 10000 5850 50  0001 C CNN
F 1 "+3.3V" H 10015 6173 50  0000 C CNN
F 2 "" H 10000 6000 50  0001 C CNN
F 3 "" H 10000 6000 50  0001 C CNN
	1    10000 6000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0151
U 1 1 5DEDEDB9
P 10000 6300
F 0 "#PWR0151" H 10000 6050 50  0001 C CNN
F 1 "GND" H 10005 6127 50  0000 C CNN
F 2 "" H 10000 6300 50  0001 C CNN
F 3 "" H 10000 6300 50  0001 C CNN
	1    10000 6300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0137
U 1 1 5DEE03A3
P 9500 6300
F 0 "#PWR0137" H 9500 6050 50  0001 C CNN
F 1 "GND" H 9505 6127 50  0000 C CNN
F 2 "" H 9500 6300 50  0001 C CNN
F 3 "" H 9500 6300 50  0001 C CNN
	1    9500 6300
	1    0    0    -1  
$EndComp
$Comp
L Device:C C102
U 1 1 5DEE03B1
P 9500 6150
F 0 "C102" H 9615 6196 50  0000 L CNN
F 1 "1µ 16V" H 9615 6105 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 9538 6000 50  0001 C CNN
F 3 "~" H 9500 6150 50  0001 C CNN
	1    9500 6150
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0136
U 1 1 5DEE03BE
P 9500 6000
F 0 "#PWR0136" H 9500 5850 50  0001 C CNN
F 1 "+3.3V" H 9515 6173 50  0000 C CNN
F 2 "" H 9500 6000 50  0001 C CNN
F 3 "" H 9500 6000 50  0001 C CNN
	1    9500 6000
	1    0    0    -1  
$EndComp
NoConn ~ 1400 4700
NoConn ~ 1400 4800
NoConn ~ 3000 4700
NoConn ~ 3000 4800
NoConn ~ 1400 5600
NoConn ~ 1400 5500
NoConn ~ 1400 5400
NoConn ~ 3000 5700
NoConn ~ 3000 6400
NoConn ~ 3000 6300
NoConn ~ 3000 5300
NoConn ~ 3000 5400
NoConn ~ 3000 5500
NoConn ~ 1400 5000
NoConn ~ 1400 5100
Text Label 1300 5200 2    50   ~ 0
WS2812B
Wire Wire Line
	1300 5200 1400 5200
$Comp
L LED:WS2812B D102
U 1 1 5DEE3A32
P 3750 2750
F 0 "D102" H 4094 2796 50  0000 L CNN
F 1 "WS2812B" H 4094 2705 50  0000 L CNN
F 2 "LED_SMD:LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm" H 3800 2450 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf" H 3850 2375 50  0001 L TNN
	1    3750 2750
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0113
U 1 1 5DEE4E31
P 3750 2450
F 0 "#PWR0113" H 3750 2300 50  0001 C CNN
F 1 "+5V" H 3765 2623 50  0000 C CNN
F 2 "" H 3750 2450 50  0001 C CNN
F 3 "" H 3750 2450 50  0001 C CNN
	1    3750 2450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5DEE5642
P 3750 3050
F 0 "#PWR0117" H 3750 2800 50  0001 C CNN
F 1 "GND" H 3755 2877 50  0000 C CNN
F 2 "" H 3750 3050 50  0001 C CNN
F 3 "" H 3750 3050 50  0001 C CNN
	1    3750 3050
	1    0    0    -1  
$EndComp
Text Label 3350 2750 2    50   ~ 0
WS2812B
Wire Wire Line
	3350 2750 3450 2750
NoConn ~ 4050 2750
$Comp
L Device:C C101
U 1 1 5DEE6005
P 4650 2750
F 0 "C101" H 4765 2796 50  0000 L CNN
F 1 "1µ 16V" H 4765 2705 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4688 2600 50  0001 C CNN
F 3 "~" H 4650 2750 50  0001 C CNN
	1    4650 2750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 5DEE6012
P 4650 2900
F 0 "#PWR0123" H 4650 2650 50  0001 C CNN
F 1 "GND" H 4655 2727 50  0000 C CNN
F 2 "" H 4650 2900 50  0001 C CNN
F 3 "" H 4650 2900 50  0001 C CNN
	1    4650 2900
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0122
U 1 1 5DEE776A
P 4650 2600
F 0 "#PWR0122" H 4650 2450 50  0001 C CNN
F 1 "+5V" H 4665 2773 50  0000 C CNN
F 2 "" H 4650 2600 50  0001 C CNN
F 3 "" H 4650 2600 50  0001 C CNN
	1    4650 2600
	1    0    0    -1  
$EndComp
Text Label 3100 5000 0    50   ~ 0
I2C_SDA
Text Label 3100 5100 0    50   ~ 0
I2C_SCL
$Comp
L Device:R R101
U 1 1 5DEE8510
P 3500 4250
F 0 "R101" H 3570 4296 50  0000 L CNN
F 1 "4,7k" H 3570 4205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3430 4250 50  0001 C CNN
F 3 "~" H 3500 4250 50  0001 C CNN
	1    3500 4250
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0112
U 1 1 5DEE8B13
P 3500 4100
F 0 "#PWR0112" H 3500 3950 50  0001 C CNN
F 1 "+3.3V" H 3515 4273 50  0000 C CNN
F 2 "" H 3500 4100 50  0001 C CNN
F 3 "" H 3500 4100 50  0001 C CNN
	1    3500 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0118
U 1 1 5DEE919A
P 3850 4100
F 0 "#PWR0118" H 3850 3950 50  0001 C CNN
F 1 "+3.3V" H 3865 4273 50  0000 C CNN
F 2 "" H 3850 4100 50  0001 C CNN
F 3 "" H 3850 4100 50  0001 C CNN
	1    3850 4100
	1    0    0    -1  
$EndComp
$Comp
L Device:R R102
U 1 1 5DEE91A8
P 3850 4250
F 0 "R102" H 3920 4296 50  0000 L CNN
F 1 "4,7k" H 3920 4205 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 3780 4250 50  0001 C CNN
F 3 "~" H 3850 4250 50  0001 C CNN
	1    3850 4250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3500 4400 3500 4650
Wire Wire Line
	3000 5000 3500 5000
Wire Wire Line
	3850 5100 3850 4850
Wire Wire Line
	3000 5100 3850 5100
$Comp
L Connector:Test_Point TP104
U 1 1 5DEEB058
P 7350 1200
F 0 "TP104" H 7408 1318 50  0000 L CNN
F 1 "+5V" H 7408 1227 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5005-5009_Compact" H 7550 1200 50  0001 C CNN
F 3 "~" H 7550 1200 50  0001 C CNN
	1    7350 1200
	1    0    0    -1  
$EndComp
$Comp
L Connector:Test_Point TP102
U 1 1 5DEEB7E8
P 4000 4650
F 0 "TP102" V 3954 4838 50  0000 L CNN
F 1 "SDA" V 4045 4838 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5005-5009_Compact" H 4200 4650 50  0001 C CNN
F 3 "~" H 4200 4650 50  0001 C CNN
	1    4000 4650
	0    1    1    0   
$EndComp
Connection ~ 7350 1200
$Comp
L Connector:Test_Point TP103
U 1 1 5DEEC70E
P 4000 4850
F 0 "TP103" V 3954 5038 50  0000 L CNN
F 1 "SCL" V 4045 5038 50  0000 L CNN
F 2 "TestPoint:TestPoint_Keystone_5005-5009_Compact" H 4200 4850 50  0001 C CNN
F 3 "~" H 4200 4850 50  0001 C CNN
	1    4000 4850
	0    1    1    0   
$EndComp
Wire Wire Line
	4000 4650 3500 4650
Connection ~ 3500 4650
Wire Wire Line
	3500 4650 3500 5000
Wire Wire Line
	3850 4850 4000 4850
Connection ~ 3850 4850
Wire Wire Line
	3850 4850 3850 4400
$Comp
L Connector:Test_Point TP101
U 1 1 5DEED0EA
P 2450 1200
F 0 "TP101" H 2450 1525 50  0000 C CNN
F 1 "+24V" H 2450 1434 50  0000 C CNN
F 2 "TestPoint:TestPoint_Keystone_5005-5009_Compact" H 2650 1200 50  0001 C CNN
F 3 "~" H 2650 1200 50  0001 C CNN
	1    2450 1200
	1    0    0    -1  
$EndComp
Text Notes 6850 6350 0    50   ~ 0
Stecker für Dom:\n1. 34 Pins für 8 Lichtelemente mit je 4 Pins\n2. 14 Pins für 2 Lichtelemente mit je 8 Pins\nAchtung: Pinzahlen auf übliche Steckergrößen aufgerundet.
$Comp
L Connector_Generic:Conn_02x17_Odd_Even J103
U 1 1 5DEEEBFF
P 10200 2300
F 0 "J103" H 10250 3317 50  0000 C CNN
F 1 "Stecker der Lichtdomplatten" H 10250 3226 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x17_P2.54mm_Horizontal" H 10200 2300 50  0001 C CNN
F 3 "~" H 10200 2300 50  0001 C CNN
	1    10200 2300
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x07_Odd_Even J104
U 1 1 5DEF1354
P 10200 4800
F 0 "J104" H 10250 5317 50  0000 C CNN
F 1 "Stecker des Zweifarb-Ringlichts" H 10250 5226 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x07_P2.54mm_Horizontal" H 10200 4800 50  0001 C CNN
F 3 "~" H 10200 4800 50  0001 C CNN
	1    10200 4800
	1    0    0    -1  
$EndComp
$Comp
L power:+24V #PWR0102
U 1 1 5DEF2929
P 1200 1200
F 0 "#PWR0102" H 1200 1050 50  0001 C CNN
F 1 "+24V" V 1215 1328 50  0000 L CNN
F 2 "" H 1200 1200 50  0001 C CNN
F 3 "" H 1200 1200 50  0001 C CNN
	1    1200 1200
	1    0    0    -1  
$EndComp
Connection ~ 1200 1200
$Comp
L power:+24V #PWR0138
U 1 1 5DEF3622
P 10000 1500
F 0 "#PWR0138" H 10000 1350 50  0001 C CNN
F 1 "+24V" V 10015 1628 50  0000 L CNN
F 2 "" H 10000 1500 50  0001 C CNN
F 3 "" H 10000 1500 50  0001 C CNN
	1    10000 1500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0152
U 1 1 5DEF480C
P 10500 1500
F 0 "#PWR0152" H 10500 1250 50  0001 C CNN
F 1 "GND" H 10505 1327 50  0000 C CNN
F 2 "" H 10500 1500 50  0001 C CNN
F 3 "" H 10500 1500 50  0001 C CNN
	1    10500 1500
	0    -1   -1   0   
$EndComp
Text Label 10000 1600 2    50   ~ 0
R1
Text Label 10500 1600 0    50   ~ 0
G1
Text Label 10000 1800 2    50   ~ 0
R2
Text Label 10000 2000 2    50   ~ 0
R3
Text Label 10000 2200 2    50   ~ 0
R4
Text Label 10000 2400 2    50   ~ 0
R5
Text Label 10000 2600 2    50   ~ 0
R6
Text Label 10000 2800 2    50   ~ 0
R7
Text Label 10000 3000 2    50   ~ 0
R8
Text Label 10500 1800 0    50   ~ 0
G2
Text Label 10500 2000 0    50   ~ 0
G3
Text Label 10500 2200 0    50   ~ 0
G4
Text Label 10500 2400 0    50   ~ 0
G5
Text Label 10500 2600 0    50   ~ 0
G6
Text Label 10500 2800 0    50   ~ 0
G7
Text Label 10500 3000 0    50   ~ 0
G8
$Comp
L power:GND #PWR0153
U 1 1 5DEF6805
P 10500 1700
F 0 "#PWR0153" H 10500 1450 50  0001 C CNN
F 1 "GND" H 10505 1527 50  0000 C CNN
F 2 "" H 10500 1700 50  0001 C CNN
F 3 "" H 10500 1700 50  0001 C CNN
	1    10500 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0154
U 1 1 5DEF6C67
P 10500 1900
F 0 "#PWR0154" H 10500 1650 50  0001 C CNN
F 1 "GND" H 10505 1727 50  0000 C CNN
F 2 "" H 10500 1900 50  0001 C CNN
F 3 "" H 10500 1900 50  0001 C CNN
	1    10500 1900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0155
U 1 1 5DEF6FA9
P 10500 2100
F 0 "#PWR0155" H 10500 1850 50  0001 C CNN
F 1 "GND" H 10505 1927 50  0000 C CNN
F 2 "" H 10500 2100 50  0001 C CNN
F 3 "" H 10500 2100 50  0001 C CNN
	1    10500 2100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0156
U 1 1 5DEF725B
P 10500 2300
F 0 "#PWR0156" H 10500 2050 50  0001 C CNN
F 1 "GND" H 10505 2127 50  0000 C CNN
F 2 "" H 10500 2300 50  0001 C CNN
F 3 "" H 10500 2300 50  0001 C CNN
	1    10500 2300
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0157
U 1 1 5DEF761B
P 10500 2500
F 0 "#PWR0157" H 10500 2250 50  0001 C CNN
F 1 "GND" H 10505 2327 50  0000 C CNN
F 2 "" H 10500 2500 50  0001 C CNN
F 3 "" H 10500 2500 50  0001 C CNN
	1    10500 2500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0158
U 1 1 5DEF7C1B
P 10500 2700
F 0 "#PWR0158" H 10500 2450 50  0001 C CNN
F 1 "GND" H 10505 2527 50  0000 C CNN
F 2 "" H 10500 2700 50  0001 C CNN
F 3 "" H 10500 2700 50  0001 C CNN
	1    10500 2700
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0159
U 1 1 5DEF7ECD
P 10500 2900
F 0 "#PWR0159" H 10500 2650 50  0001 C CNN
F 1 "GND" H 10505 2727 50  0000 C CNN
F 2 "" H 10500 2900 50  0001 C CNN
F 3 "" H 10500 2900 50  0001 C CNN
	1    10500 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0160
U 1 1 5DEF820F
P 10500 3100
F 0 "#PWR0160" H 10500 2850 50  0001 C CNN
F 1 "GND" H 10505 2927 50  0000 C CNN
F 2 "" H 10500 3100 50  0001 C CNN
F 3 "" H 10500 3100 50  0001 C CNN
	1    10500 3100
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0139
U 1 1 5DEF85B2
P 10000 1700
F 0 "#PWR0139" H 10000 1550 50  0001 C CNN
F 1 "+24V" V 10015 1828 50  0000 L CNN
F 2 "" H 10000 1700 50  0001 C CNN
F 3 "" H 10000 1700 50  0001 C CNN
	1    10000 1700
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0140
U 1 1 5DEF8C00
P 10000 1900
F 0 "#PWR0140" H 10000 1750 50  0001 C CNN
F 1 "+24V" V 10015 2028 50  0000 L CNN
F 2 "" H 10000 1900 50  0001 C CNN
F 3 "" H 10000 1900 50  0001 C CNN
	1    10000 1900
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0142
U 1 1 5DEF92FD
P 10000 2300
F 0 "#PWR0142" H 10000 2150 50  0001 C CNN
F 1 "+24V" V 10015 2428 50  0000 L CNN
F 2 "" H 10000 2300 50  0001 C CNN
F 3 "" H 10000 2300 50  0001 C CNN
	1    10000 2300
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0143
U 1 1 5DEF96AB
P 10000 2500
F 0 "#PWR0143" H 10000 2350 50  0001 C CNN
F 1 "+24V" V 10015 2628 50  0000 L CNN
F 2 "" H 10000 2500 50  0001 C CNN
F 3 "" H 10000 2500 50  0001 C CNN
	1    10000 2500
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0144
U 1 1 5DEF9AD1
P 10000 2700
F 0 "#PWR0144" H 10000 2550 50  0001 C CNN
F 1 "+24V" V 10015 2828 50  0000 L CNN
F 2 "" H 10000 2700 50  0001 C CNN
F 3 "" H 10000 2700 50  0001 C CNN
	1    10000 2700
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0145
U 1 1 5DEF9F87
P 10000 2900
F 0 "#PWR0145" H 10000 2750 50  0001 C CNN
F 1 "+24V" V 10015 3028 50  0000 L CNN
F 2 "" H 10000 2900 50  0001 C CNN
F 3 "" H 10000 2900 50  0001 C CNN
	1    10000 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0146
U 1 1 5DEFA485
P 10000 3100
F 0 "#PWR0146" H 10000 2950 50  0001 C CNN
F 1 "+24V" V 10015 3228 50  0000 L CNN
F 2 "" H 10000 3100 50  0001 C CNN
F 3 "" H 10000 3100 50  0001 C CNN
	1    10000 3100
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0141
U 1 1 5DEF909E
P 10000 2100
F 0 "#PWR0141" H 10000 1950 50  0001 C CNN
F 1 "+24V" V 10015 2228 50  0000 L CNN
F 2 "" H 10000 2100 50  0001 C CNN
F 3 "" H 10000 2100 50  0001 C CNN
	1    10000 2100
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0147
U 1 1 5DEFC15C
P 10000 4500
F 0 "#PWR0147" H 10000 4350 50  0001 C CNN
F 1 "+24V" V 10015 4628 50  0000 L CNN
F 2 "" H 10000 4500 50  0001 C CNN
F 3 "" H 10000 4500 50  0001 C CNN
	1    10000 4500
	0    -1   -1   0   
$EndComp
$Comp
L power:+24V #PWR0148
U 1 1 5DEFC762
P 10000 4800
F 0 "#PWR0148" H 10000 4650 50  0001 C CNN
F 1 "+24V" V 10015 4928 50  0000 L CNN
F 2 "" H 10000 4800 50  0001 C CNN
F 3 "" H 10000 4800 50  0001 C CNN
	1    10000 4800
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0161
U 1 1 5DEFCC07
P 10500 4500
F 0 "#PWR0161" H 10500 4250 50  0001 C CNN
F 1 "GND" H 10505 4327 50  0000 C CNN
F 2 "" H 10500 4500 50  0001 C CNN
F 3 "" H 10500 4500 50  0001 C CNN
	1    10500 4500
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0162
U 1 1 5DEFD135
P 10500 4800
F 0 "#PWR0162" H 10500 4550 50  0001 C CNN
F 1 "GND" H 10505 4627 50  0000 C CNN
F 2 "" H 10500 4800 50  0001 C CNN
F 3 "" H 10500 4800 50  0001 C CNN
	1    10500 4800
	0    -1   -1   0   
$EndComp
Text Label 10000 4600 2    50   ~ 0
R1.1
Text Label 10000 4700 2    50   ~ 0
R1.2
Text Label 10000 4900 2    50   ~ 0
R1.3
Text Label 10000 5000 2    50   ~ 0
R1.4
Text Label 10500 4600 0    50   ~ 0
G1.1
Text Label 10500 4700 0    50   ~ 0
G1.2
Text Label 10500 4900 0    50   ~ 0
G1.3
Text Label 10500 5000 0    50   ~ 0
G1.4
$Comp
L power:+24V #PWR0149
U 1 1 5DEFDA80
P 10000 5100
F 0 "#PWR0149" H 10000 4950 50  0001 C CNN
F 1 "+24V" V 10015 5228 50  0000 L CNN
F 2 "" H 10000 5100 50  0001 C CNN
F 3 "" H 10000 5100 50  0001 C CNN
	1    10000 5100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0163
U 1 1 5DEFE02D
P 10500 5100
F 0 "#PWR0163" H 10500 4850 50  0001 C CNN
F 1 "GND" H 10505 4927 50  0000 C CNN
F 2 "" H 10500 5100 50  0001 C CNN
F 3 "" H 10500 5100 50  0001 C CNN
	1    10500 5100
	0    -1   -1   0   
$EndComp
Text Label 9050 3350 0    50   ~ 0
R8
Text Label 9050 1600 0    50   ~ 0
R4
Text Label 9050 2950 0    50   ~ 0
R6
Text Label 9050 1200 0    50   ~ 0
R2
Text Label 9050 2750 0    50   ~ 0
R5
Text Label 9050 1000 0    50   ~ 0
R1
Text Label 9050 3150 0    50   ~ 0
R7
Text Label 9050 1400 0    50   ~ 0
R3
Text Label 9050 1500 0    50   ~ 0
G3
Text Label 9050 1700 0    50   ~ 0
G4
Text Label 9050 3250 0    50   ~ 0
G7
Text Label 9050 1100 0    50   ~ 0
G1
Text Label 9050 3050 0    50   ~ 0
G6
Text Label 9050 1300 0    50   ~ 0
G2
Text Label 9050 2850 0    50   ~ 0
G5
Text Label 9050 3450 0    50   ~ 0
G8
Text Label 9050 4900 0    50   ~ 0
R1.3
Text Label 9050 4500 0    50   ~ 0
R1.1
Text Label 9050 4700 0    50   ~ 0
R1.2
Text Label 9050 5100 0    50   ~ 0
R1.4
Text Label 9050 4600 0    50   ~ 0
G1.1
Text Label 9050 5000 0    50   ~ 0
G1.3
Text Label 9050 4800 0    50   ~ 0
G1.2
Text Label 9050 5200 0    50   ~ 0
G1.4
$Comp
L Connector_Generic:Conn_01x04 J105
U 1 1 5DF02E47
P 5950 4200
F 0 "J105" H 6030 4192 50  0000 L CNN
F 1 "0,96\" OLED" H 6030 4101 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5950 4200 50  0001 C CNN
F 3 "~" H 5950 4200 50  0001 C CNN
	1    5950 4200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0166
U 1 1 5DF03A4B
P 5750 4100
F 0 "#PWR0166" H 5750 3850 50  0001 C CNN
F 1 "GND" H 5755 3927 50  0000 C CNN
F 2 "" H 5750 4100 50  0001 C CNN
F 3 "" H 5750 4100 50  0001 C CNN
	1    5750 4100
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR0167
U 1 1 5DF05A5D
P 5750 4200
F 0 "#PWR0167" H 5750 4050 50  0001 C CNN
F 1 "+3.3V" H 5765 4373 50  0000 C CNN
F 2 "" H 5750 4200 50  0001 C CNN
F 3 "" H 5750 4200 50  0001 C CNN
	1    5750 4200
	0    -1   -1   0   
$EndComp
Text Label 5450 4300 2    50   ~ 0
I2C_SCL
Text Label 5450 4400 2    50   ~ 0
I2C_SDA
Wire Wire Line
	5450 4400 5750 4400
Wire Wire Line
	5750 4300 5450 4300
$Comp
L Mechanical:Mounting_Hole MK105
U 1 1 5DF0CFCD
P 5750 3000
F 0 "MK105" H 5850 3046 50  0000 L CNN
F 1 "Mounting_Hole" H 5850 2955 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965" H 5750 3000 50  0001 C CNN
F 3 "" H 5750 3000 50  0001 C CNN
	1    5750 3000
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Mounting_Hole MK106
U 1 1 5DF0D94C
P 5750 3250
F 0 "MK106" H 5850 3296 50  0000 L CNN
F 1 "Mounting_Hole" H 5850 3205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965" H 5750 3250 50  0001 C CNN
F 3 "" H 5750 3250 50  0001 C CNN
	1    5750 3250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Mounting_Hole MK107
U 1 1 5DF0DC9F
P 5750 3500
F 0 "MK107" H 5850 3546 50  0000 L CNN
F 1 "Mounting_Hole" H 5850 3455 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965" H 5750 3500 50  0001 C CNN
F 3 "" H 5750 3500 50  0001 C CNN
	1    5750 3500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Mounting_Hole MK108
U 1 1 5DF0DF98
P 5750 3750
F 0 "MK108" H 5850 3796 50  0000 L CNN
F 1 "Mounting_Hole" H 5850 3705 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3_DIN965" H 5750 3750 50  0001 C CNN
F 3 "" H 5750 3750 50  0001 C CNN
	1    5750 3750
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Mounting_Hole MK103
U 1 1 5DF10368
P 5500 6500
F 0 "MK103" H 5600 6546 50  0000 L CNN
F 1 "Mounting_Hole" H 5600 6455 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 5500 6500 50  0001 C CNN
F 3 "" H 5500 6500 50  0001 C CNN
	1    5500 6500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Mounting_Hole MK101
U 1 1 5DF10374
P 5500 6000
F 0 "MK101" H 5600 6046 50  0000 L CNN
F 1 "Mounting_Hole" H 5600 5955 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 5500 6000 50  0001 C CNN
F 3 "" H 5500 6000 50  0001 C CNN
	1    5500 6000
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Mounting_Hole MK102
U 1 1 5DF10380
P 5500 6250
F 0 "MK102" H 5600 6296 50  0000 L CNN
F 1 "Mounting_Hole" H 5600 6205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 5500 6250 50  0001 C CNN
F 3 "" H 5500 6250 50  0001 C CNN
	1    5500 6250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:Mounting_Hole MK104
U 1 1 5DF1038C
P 5500 6750
F 0 "MK104" H 5600 6796 50  0000 L CNN
F 1 "Mounting_Hole" H 5600 6705 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 5500 6750 50  0001 C CNN
F 3 "" H 5500 6750 50  0001 C CNN
	1    5500 6750
	1    0    0    -1  
$EndComp
Text Notes 5500 5800 0    50   ~ 0
Platinenanschlusslöcher
Text Notes 5750 2800 0    50   ~ 0
OLED-Montagelöcher
$Comp
L power:+5V #PWR?
U 1 1 5DE6E388
P 6700 1200
F 0 "#PWR?" H 6700 1050 50  0001 C CNN
F 1 "+5V" H 6715 1373 50  0000 C CNN
F 2 "" H 6700 1200 50  0001 C CNN
F 3 "" H 6700 1200 50  0001 C CNN
	1    6700 1200
	1    0    0    -1  
$EndComp
$EndSCHEMATC
