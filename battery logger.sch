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
Sheet 1 6
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Sheet
S 5600 2500 950  1600
U 54E6466D
F0 "Current Sink" 59
F1 "CurrentSink.sch" 59
F2 "Enable" I L 5600 2700 28 
F3 "I_SNS" O L 5600 3200 28 
F4 "V_SET" I L 5600 2950 28 
F5 "BATT_V" O R 6550 3200 28 
$EndSheet
$Sheet
S 3100 2500 1150 1600
U 54EC6BB3
F0 "uController" 59
F1 "uController.sch" 59
F2 "Enable" O R 4250 2700 28 
F3 "V_SET" O R 4250 2950 28 
F4 "I_SNS" I R 4250 3200 28 
F5 "V_SNS1" I L 3100 3450 28 
F6 "V_SNS2" I L 3100 3600 28 
F7 "V_SNS3" I L 3100 3750 28 
F8 "V_SNS4" I L 3100 3900 28 
F9 "ENC_BTN" I L 3100 3000 28 
F10 "ENC_A" I L 3100 2800 28 
F11 "ENC_B" I L 3100 2900 28 
$EndSheet
Wire Wire Line
	4250 2700 5600 2700
Wire Wire Line
	4250 2950 5600 2950
Wire Wire Line
	4250 3200 5600 3200
$Sheet
S 7600 2500 1000 1600
U 54EE6259
F0 "Input Voltage Range Switch" 59
F1 "InputVoltageRangeSwitch.sch" 59
F2 "BATT_V" I L 7600 3200 28 
F3 "V_SNS4" O R 8600 3900 28 
F4 "V_SNS3" O R 8600 3750 28 
F5 "V_SNS2" O R 8600 3600 28 
F6 "V_SNS1" I R 8600 3450 28 
$EndSheet
$Sheet
S 1250 1150 850  750 
U 54E428BF
F0 "Voltage Regulator" 59
F1 "VoltageRegulator.sch" 59
$EndSheet
Wire Wire Line
	8600 3900 8750 3900
Wire Wire Line
	8750 3900 8750 4400
Wire Wire Line
	8750 4400 2950 4400
Wire Wire Line
	2950 4400 2950 3900
Wire Wire Line
	2950 3900 3100 3900
Wire Wire Line
	8600 3750 8800 3750
Wire Wire Line
	8800 3750 8800 4450
Wire Wire Line
	8800 4450 2900 4450
Wire Wire Line
	2900 4450 2900 3750
Wire Wire Line
	2900 3750 3100 3750
Wire Wire Line
	8600 3600 8850 3600
Wire Wire Line
	8850 3600 8850 4500
Wire Wire Line
	8850 4500 2850 4500
Wire Wire Line
	2850 4500 2850 3600
Wire Wire Line
	2850 3600 3100 3600
Wire Wire Line
	8600 3450 8900 3450
Wire Wire Line
	8900 3450 8900 4550
Wire Wire Line
	8900 4550 2800 4550
Wire Wire Line
	2800 4550 2800 3450
Wire Wire Line
	2800 3450 3100 3450
Wire Wire Line
	6550 3200 7600 3200
$Sheet
S 1500 2500 800  850 
U 54F30439
F0 "Encoder Interface" 59
F1 "EncoderInterface.sch" 59
F2 "ENC_A" O R 2300 2800 28 
F3 "ENC_B" O R 2300 2900 28 
F4 "ENC_BTN" O R 2300 3000 28 
$EndSheet
Wire Wire Line
	2300 2800 3100 2800
Wire Wire Line
	2300 2900 3100 2900
Wire Wire Line
	2300 3000 3100 3000
$EndSCHEMATC
