EESchema Schematic File Version 2
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
LIBS:contrib
LIBS:valves
LIBS:op-amps
LIBS:battery logger
LIBS:atmel
LIBS:battery logger-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 3
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
L C C7
U 1 1 54DA97A4
P 9850 5500
F 0 "C7" H 9900 5600 50  0000 L CNN
F 1 "0.1u" H 9900 5400 50  0000 L CNN
F 2 "" H 9888 5350 30  0000 C CNN
F 3 "" H 9850 5500 60  0000 C CNN
	1    9850 5500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR13
U 1 1 54DAB382
P 9850 5800
F 0 "#PWR13" H 9850 5550 60  0001 C CNN
F 1 "GND" H 9850 5650 60  0001 C CNN
F 2 "" H 9850 5800 60  0000 C CNN
F 3 "" H 9850 5800 60  0000 C CNN
F 4 "Value" H 9850 5800 60  0001 C CNN "MPN"
	1    9850 5800
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR12
U 1 1 54DAB3A9
P 9850 5150
F 0 "#PWR12" H 9850 5000 60  0001 C CNN
F 1 "VCC" H 9850 5300 60  0000 C CNN
F 2 "" H 9850 5150 60  0000 C CNN
F 3 "" H 9850 5150 60  0000 C CNN
	1    9850 5150
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR2
U 1 1 54DADBBE
P 2100 3500
F 0 "#PWR2" H 2100 3350 60  0001 C CNN
F 1 "VCC" H 2100 3650 60  0000 C CNN
F 2 "" H 2100 3500 60  0000 C CNN
F 3 "" H 2100 3500 60  0000 C CNN
	1    2100 3500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR3
U 1 1 54DAE755
P 2100 5100
F 0 "#PWR3" H 2100 4850 60  0001 C CNN
F 1 "GND" H 2100 4950 60  0001 C CNN
F 2 "" H 2100 5100 60  0000 C CNN
F 3 "" H 2100 5100 60  0000 C CNN
F 4 "Value" H 2100 5100 60  0001 C CNN "MPN"
	1    2100 5100
	1    0    0    -1  
$EndComp
$Comp
L CRYSTAL X?
U 1 1 54DB7661
P 1650 3200
F 0 "X?" H 1650 3350 50  0000 C CNN
F 1 "CRYSTAL" H 1650 3050 50  0000 C CNN
F 2 "" H 1650 3200 60  0000 C CNN
F 3 "" H 1650 3200 60  0000 C CNN
	1    1650 3200
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 54DB77C8
P 1350 2900
F 0 "C?" H 1400 3000 50  0000 L CNN
F 1 "12p" H 1400 2800 50  0000 L CNN
F 2 "" H 1388 2750 30  0000 C CNN
F 3 "" H 1350 2900 60  0000 C CNN
F 4 "Value" H 1350 2900 60  0001 C CNN "MPN"
	1    1350 2900
	0    1    1    0   
$EndComp
$Comp
L C C?
U 1 1 54DB78C1
P 1350 3500
F 0 "C?" H 1400 3600 50  0000 L CNN
F 1 "12p" H 1400 3400 50  0000 L CNN
F 2 "" H 1388 3350 30  0000 C CNN
F 3 "" H 1350 3500 60  0000 C CNN
F 4 "Value" H 1350 3500 60  0001 C CNN "MPN"
	1    1350 3500
	0    1    1    0   
$EndComp
$Comp
L CONN_02X03 P?
U 1 1 54DB9FFF
P 3200 1750
F 0 "P?" H 3200 1950 50  0000 C CNN
F 1 "CONN_02X03" H 3200 1550 50  0000 C CNN
F 2 "" H 3200 550 60  0000 C CNN
F 3 "" H 3200 550 60  0000 C CNN
	1    3200 1750
	0    1    1    0   
$EndComp
$Comp
L CONN_01X05 P?
U 1 1 54DBA1E1
P 3250 6500
F 0 "P?" H 3250 6800 50  0000 C CNN
F 1 "CONN_01X05" V 3350 6500 50  0000 C CNN
F 2 "" H 3250 6500 60  0000 C CNN
F 3 "" H 3250 6500 60  0000 C CNN
	1    3250 6500
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54DBA355
P 3700 6800
F 0 "#PWR?" H 3700 6550 60  0001 C CNN
F 1 "GND" H 3700 6650 60  0001 C CNN
F 2 "" H 3700 6800 60  0000 C CNN
F 3 "" H 3700 6800 60  0000 C CNN
F 4 "Value" H 3700 6800 60  0001 C CNN "MPN"
	1    3700 6800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54DBA62C
P 900 3250
F 0 "#PWR?" H 900 3000 60  0001 C CNN
F 1 "GND" H 900 3100 60  0001 C CNN
F 2 "" H 900 3250 60  0000 C CNN
F 3 "" H 900 3250 60  0000 C CNN
F 4 "Value" H 900 3250 60  0001 C CNN "MPN"
	1    900  3250
	1    0    0    -1  
$EndComp
$Comp
L ATMEGA328-P IC?
U 1 1 54DA7188
P 3050 3800
F 0 "IC?" H 2300 5050 40  0000 L BNN
F 1 "ATMEGA328-P" H 3450 2400 40  0000 L BNN
F 2 "DIL28" H 3050 3800 30  0000 C CIN
F 3 "" H 3050 3800 60  0000 C CNN
	1    3050 3800
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 54DAA1E2
P 1750 2300
F 0 "R?" V 1830 2300 50  0000 C CNN
F 1 "10k" V 1757 2301 50  0000 C CNN
F 2 "0805" V 1680 2300 30  0000 C CNN
F 3 "https://www.seielect.com/catalog/SEI-RNCP.pdf" H 1750 2300 30  0001 C CNN
F 4 "RNCP0805FTD10K0" V 1750 2300 60  0001 C CNN "MPN"
	1    1750 2300
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 54DAA30B
P 1400 2250
F 0 "#PWR?" H 1400 2100 60  0001 C CNN
F 1 "VCC" H 1400 2400 60  0000 C CNN
F 2 "" H 1400 2250 60  0000 C CNN
F 3 "" H 1400 2250 60  0000 C CNN
	1    1400 2250
	1    0    0    -1  
$EndComp
$Comp
L ZENER D?
U 1 1 54DAF867
P 1400 4950
F 0 "D?" H 1400 5050 50  0000 C CNN
F 1 "ZENER" H 1400 4850 50  0000 C CNN
F 2 "" H 1400 4950 60  0000 C CNN
F 3 "" H 1400 4950 60  0000 C CNN
	1    1400 4950
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR?
U 1 1 54DD1C73
P 2550 5900
F 0 "#PWR?" H 2550 5750 60  0001 C CNN
F 1 "VCC" H 2550 6050 60  0000 C CNN
F 2 "" H 2550 5900 60  0000 C CNN
F 3 "" H 2550 5900 60  0000 C CNN
	1    2550 5900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54DD1CCF
P 2400 6000
F 0 "#PWR?" H 2400 5750 60  0001 C CNN
F 1 "GND" H 2400 5850 60  0001 C CNN
F 2 "" H 2400 6000 60  0000 C CNN
F 3 "" H 2400 6000 60  0000 C CNN
F 4 "Value" H 2400 6000 60  0001 C CNN "MPN"
	1    2400 6000
	1    0    0    -1  
$EndComp
$Comp
L POT RV?
U 1 1 54DD1CDC
P 2550 6250
F 0 "RV?" H 2550 6150 50  0000 C CNN
F 1 "POT" H 2550 6250 50  0000 C CNN
F 2 "" H 2550 6250 60  0000 C CNN
F 3 "" H 2550 6250 60  0000 C CNN
	1    2550 6250
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR?
U 1 1 54DD244B
P 2400 6600
F 0 "#PWR?" H 2400 6350 60  0001 C CNN
F 1 "GND" H 2400 6450 60  0001 C CNN
F 2 "" H 2400 6600 60  0000 C CNN
F 3 "" H 2400 6600 60  0000 C CNN
F 4 "Value" H 2400 6600 60  0001 C CNN "MPN"
	1    2400 6600
	1    0    0    -1  
$EndComp
Text Label 4100 4500 0    28   ~ 0
LCD-04
Text Label 4100 4600 0    28   ~ 0
LCD-06
Text Label 4100 4700 0    28   ~ 0
LCD-11
Text Label 4100 4800 0    28   ~ 0
LCD-12
Text Label 4100 4900 0    28   ~ 0
LCD-13
Text Label 4100 5000 0    28   ~ 0
LCD-14
Entry Wire Line
	4300 4500 4400 4600
Entry Wire Line
	4300 4600 4400 4700
Entry Wire Line
	4300 4700 4400 4800
Entry Wire Line
	4300 4800 4400 4900
Entry Wire Line
	4300 4900 4400 5000
Entry Wire Line
	4300 5000 4400 5100
$Comp
L CONN_01X10 P1
U 1 1 54DAFD2A
P 1500 6700
F 0 "P1" H 1500 7250 50  0000 C CNN
F 1 "CONN_01X10" V 1600 6700 50  0000 C CNN
F 2 "" H 1500 6700 60  0000 C CNN
F 3 "" H 1500 6700 60  0000 C CNN
	1    1500 6700
	-1   0    0    -1  
$EndComp
Text Label 1750 6250 0    28   ~ 0
LCD-01
Text Label 1750 6350 0    28   ~ 0
LCD-02
Text Label 1750 6450 0    28   ~ 0
LCD-03
Text Label 1750 6550 0    28   ~ 0
LCD-04
Text Label 1750 6650 0    28   ~ 0
LCD-05
Text Label 1750 6750 0    28   ~ 0
LCD-06
Text Label 1750 6850 0    28   ~ 0
LCD-11
Text Label 1750 6950 0    28   ~ 0
LCD-12
Text Label 1750 7050 0    28   ~ 0
LCD-13
Text Label 1750 7150 0    28   ~ 0
LCD-14
Entry Wire Line
	1950 7150 2050 7050
Entry Wire Line
	1950 7050 2050 6950
Entry Wire Line
	1950 6950 2050 6850
Entry Wire Line
	1950 6850 2050 6750
Entry Wire Line
	1950 6750 2050 6650
Entry Wire Line
	1950 6650 2050 6550
Entry Wire Line
	1950 6550 2050 6450
Entry Wire Line
	1950 6450 2050 6350
Entry Wire Line
	1950 6350 2050 6250
Entry Wire Line
	1950 6250 2050 6150
Entry Wire Line
	2050 6550 2150 6450
Text Label 2200 6450 0    28   ~ 0
LCD-05
Entry Wire Line
	2050 6350 2150 6250
$Comp
L GND #PWR?
U 1 1 54DDBCC0
P 2550 6600
F 0 "#PWR?" H 2550 6350 60  0001 C CNN
F 1 "GND" H 2550 6450 60  0001 C CNN
F 2 "" H 2550 6600 60  0000 C CNN
F 3 "" H 2550 6600 60  0000 C CNN
F 4 "Value" H 2550 6600 60  0001 C CNN "MPN"
	1    2550 6600
	1    0    0    -1  
$EndComp
Entry Wire Line
	2050 6050 2150 5950
Entry Wire Line
	2050 5900 2150 5800
Text Label 2200 5950 0    28   ~ 0
LCD-02
$Comp
L VCC #PWR?
U 1 1 54DDCA76
P 2400 5750
F 0 "#PWR?" H 2400 5600 60  0001 C CNN
F 1 "VCC" H 2400 5900 60  0000 C CNN
F 2 "" H 2400 5750 60  0000 C CNN
F 3 "" H 2400 5750 60  0000 C CNN
	1    2400 5750
	1    0    0    -1  
$EndComp
Text Label 2200 5800 0    28   ~ 0
LCD-01
Text Notes 1550 7400 0    59   ~ 0
To LCD
Text Notes 3100 7000 0    59   ~ 0
To rotary\nencoder
Text Notes 2900 1800 1    59   ~ 0
ISP
$Comp
L C C?
U 1 1 54E0FEC0
P 9550 5500
F 0 "C?" H 9600 5600 50  0000 L CNN
F 1 "0.1u" H 9600 5400 50  0000 L CNN
F 2 "" H 9588 5350 30  0000 C CNN
F 3 "" H 9550 5500 60  0000 C CNN
	1    9550 5500
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54E0FEF9
P 9550 5800
F 0 "#PWR?" H 9550 5550 60  0001 C CNN
F 1 "GND" H 9550 5650 60  0001 C CNN
F 2 "" H 9550 5800 60  0000 C CNN
F 3 "" H 9550 5800 60  0000 C CNN
F 4 "Value" H 9550 5800 60  0001 C CNN "MPN"
	1    9550 5800
	1    0    0    -1  
$EndComp
Text Label 9550 5250 1    28   ~ 0
Enable
$Sheet
S 5600 900  850  750 
U 54E428BF
F0 "Voltage Regulator" 59
F1 "VoltageRegulator.sch" 59
$EndSheet
$Comp
L GND #PWR?
U 1 1 54E4B89D
P 3100 2100
F 0 "#PWR?" H 3100 1850 60  0001 C CNN
F 1 "GND" H 3100 1950 60  0001 C CNN
F 2 "" H 3100 2100 60  0000 C CNN
F 3 "" H 3100 2100 60  0000 C CNN
F 4 "Value" H 3100 2100 60  0001 C CNN "MPN"
	1    3100 2100
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 54E4B959
P 3550 1950
F 0 "#PWR?" H 3550 1800 60  0001 C CNN
F 1 "VCC" H 3550 2100 60  0000 C CNN
F 2 "" H 3550 1950 60  0000 C CNN
F 3 "" H 3550 1950 60  0000 C CNN
	1    3550 1950
	1    0    0    -1  
$EndComp
$Sheet
S 5600 2500 950  1600
U 54E6466D
F0 "Current Sink" 59
F1 "CurrentSink.sch" 59
F2 "Enable" I L 5600 2700 28 
F3 "V_SNS" O L 5600 3750 28 
F4 "I_SNS" O L 5600 3850 28 
F5 "V_SET" I L 5600 2800 28 
$EndSheet
$Comp
L TS922 U?
U 1 1 54E8DC9B
P 8300 5600
F 0 "U?" H 8250 5800 60  0000 L CNN
F 1 "TS922" H 8250 5350 60  0000 L CNN
F 2 "" H 8300 5600 60  0000 C CNN
F 3 "" H 8300 5600 60  0000 C CNN
	1    8300 5600
	1    0    0    -1  
$EndComp
Text Label 4500 2700 0    28   ~ 0
Enable
Text Label 4500 2800 0    28   ~ 0
V_SET
Wire Wire Line
	9850 5150 9850 5300
Wire Wire Line
	9850 5700 9850 5800
Wire Wire Line
	4050 3550 4600 3550
Wire Wire Line
	4050 3200 4550 3200
Wire Wire Line
	1550 2900 1850 2900
Wire Wire Line
	1650 2900 1650 2900
Wire Wire Line
	1550 3500 1850 3500
Wire Wire Line
	1150 2900 1100 2900
Wire Wire Line
	1100 2900 1100 3500
Wire Wire Line
	1100 3500 1150 3500
Wire Wire Line
	900  3200 1100 3200
Connection ~ 1100 3200
Wire Wire Line
	2100 3500 2100 3750
Wire Wire Line
	2100 3600 2150 3600
Wire Wire Line
	2150 4900 2100 4900
Wire Wire Line
	2100 4900 2100 5100
Wire Wire Line
	2150 5000 2100 5000
Connection ~ 2100 5000
Wire Wire Line
	2150 3100 1850 3100
Wire Wire Line
	1850 3100 1850 2900
Connection ~ 1650 2900
Wire Wire Line
	2150 3250 1850 3250
Wire Wire Line
	1850 3250 1850 3500
Connection ~ 1650 3500
Wire Wire Line
	2100 3750 2150 3750
Connection ~ 2100 3600
Wire Wire Line
	4050 5000 4300 5000
Wire Wire Line
	4050 4900 4300 4900
Wire Wire Line
	4050 4800 4300 4800
Wire Wire Line
	4050 4700 4300 4700
Wire Wire Line
	4050 4600 4300 4600
Wire Wire Line
	4050 4500 4300 4500
Wire Wire Line
	2550 5900 2550 6000
Wire Wire Line
	1700 6450 1950 6450
Wire Wire Line
	1700 6550 1950 6550
Wire Wire Line
	1700 6650 1950 6650
Wire Wire Line
	1700 6750 1950 6750
Wire Wire Line
	1700 6850 1950 6850
Wire Wire Line
	1700 6950 1950 6950
Wire Wire Line
	1700 7050 1950 7050
Wire Wire Line
	1700 7150 1950 7150
Wire Wire Line
	1700 6250 1950 6250
Wire Wire Line
	1700 6350 1950 6350
Wire Bus Line
	2050 5350 2050 7050
Wire Bus Line
	4400 5350 2050 5350
Wire Bus Line
	4400 4600 4400 5350
Wire Wire Line
	2400 6450 2150 6450
Wire Wire Line
	2150 6250 2400 6250
Wire Wire Line
	2400 6450 2400 6600
Wire Wire Line
	2550 6500 2550 6600
Wire Wire Line
	2150 5950 2400 5950
Wire Wire Line
	2400 5950 2400 6000
Wire Wire Line
	2150 5800 2400 5800
Wire Wire Line
	2400 5800 2400 5750
Wire Wire Line
	9550 5100 9550 5300
Wire Wire Line
	9550 5700 9550 5800
Wire Wire Line
	1500 2300 1400 2300
Wire Wire Line
	2100 2700 2150 2700
Wire Wire Line
	2100 1350 2100 2700
Wire Wire Line
	2100 2300 2000 2300
Wire Wire Line
	4050 3650 4650 3650
Wire Wire Line
	3450 6700 4650 6700
Wire Wire Line
	3450 6600 3700 6600
Wire Wire Line
	3450 6500 4600 6500
Wire Wire Line
	3450 6400 3700 6400
Wire Wire Line
	3450 6300 4550 6300
Wire Wire Line
	4550 6300 4550 3200
Wire Wire Line
	4600 6500 4600 3550
Wire Wire Line
	4650 6700 4650 3650
Wire Wire Line
	3700 6400 3700 6800
Connection ~ 3700 6600
Wire Wire Line
	3100 1500 3100 1350
Wire Wire Line
	3100 1350 2100 1350
Connection ~ 2100 2300
Wire Wire Line
	3100 2000 3100 2100
Wire Wire Line
	3300 2000 3550 2000
Wire Wire Line
	4050 3000 4150 3000
Wire Wire Line
	4150 3000 4150 2100
Wire Wire Line
	4150 2100 3200 2100
Wire Wire Line
	3200 2100 3200 2000
Wire Wire Line
	4050 3100 4200 3100
Wire Wire Line
	4200 3100 4200 1400
Wire Wire Line
	4200 1400 3300 1400
Wire Wire Line
	3300 1400 3300 1500
Wire Wire Line
	3200 1500 3200 1350
Wire Wire Line
	3200 1350 4250 1350
Wire Wire Line
	4250 1350 4250 3200
Connection ~ 4250 3200
Wire Wire Line
	4050 2700 5600 2700
Wire Wire Line
	4050 2800 5600 2800
Wire Wire Line
	4050 3750 5600 3750
Text Label 4350 3750 0    28   ~ 0
V_SNS
Wire Wire Line
	4050 3850 5600 3850
Text Label 4350 3850 0    28   ~ 0
I_SNS
$Comp
L VCC #PWR?
U 1 1 54E9AD82
P 1400 4050
F 0 "#PWR?" H 1400 3900 60  0001 C CNN
F 1 "VCC" H 1400 4200 60  0000 C CNN
F 2 "" H 1400 4050 60  0000 C CNN
F 3 "" H 1400 4050 60  0000 C CNN
	1    1400 4050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 54E9ADC3
P 1400 5200
F 0 "#PWR?" H 1400 4950 60  0001 C CNN
F 1 "GND" H 1400 5050 60  0001 C CNN
F 2 "" H 1400 5200 60  0000 C CNN
F 3 "" H 1400 5200 60  0000 C CNN
F 4 "Value" H 1400 5200 60  0001 C CNN "MPN"
	1    1400 5200
	1    0    0    -1  
$EndComp
$Comp
L R R?
U 1 1 54E9AE10
P 1400 4350
F 0 "R?" V 1480 4350 50  0000 C CNN
F 1 "10k" V 1407 4351 50  0000 C CNN
F 2 "0805" V 1330 4350 30  0000 C CNN
F 3 "https://www.seielect.com/catalog/SEI-RNCP.pdf" H 1400 4350 30  0001 C CNN
F 4 "RNCP0805FTD10K0" V 1400 4350 60  0001 C CNN "MPN"
	1    1400 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 4050 1400 4100
Wire Wire Line
	1400 4600 1400 4750
Wire Wire Line
	1400 5150 1400 5200
Wire Wire Line
	2150 4150 1900 4150
Wire Wire Line
	1900 4150 1900 4650
Wire Wire Line
	1900 4650 1400 4650
Connection ~ 1400 4650
Wire Wire Line
	900  3200 900  3250
Wire Wire Line
	3550 2000 3550 1950
Wire Wire Line
	1400 2300 1400 2250
$Sheet
S 7500 2700 1150 1750
U 54EC6BB3
F0 "uController" 59
F1 "uController.sch" 59
$EndSheet
$EndSCHEMATC
