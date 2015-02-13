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
Sheet 1 4
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
$Sheet
S 5600 2500 950  1600
U 54E6466D
F0 "Current Sink" 59
F1 "CurrentSink.sch" 59
F2 "Enable" I L 5600 2700 28 
F3 "V_SNS" O L 5600 3800 28 
F4 "I_SNS" O L 5600 3200 28 
F5 "V_SET" I L 5600 2950 28 
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
Wire Wire Line
	9550 5100 9550 5300
Wire Wire Line
	9550 5700 9550 5800
$Sheet
S 3100 2500 1150 1600
U 54EC6BB3
F0 "uController" 59
F1 "uController.sch" 59
F2 "Enable" O R 4250 2700 28 
F3 "V_SET" O R 4250 2950 28 
F4 "V_SNS" I R 4250 3800 28 
F5 "I_SNS" I R 4250 3200 28 
$EndSheet
Wire Wire Line
	4250 2700 5600 2700
Wire Wire Line
	4250 2950 5600 2950
Wire Wire Line
	4250 3200 5600 3200
Wire Wire Line
	4250 3800 5600 3800
$EndSCHEMATC
