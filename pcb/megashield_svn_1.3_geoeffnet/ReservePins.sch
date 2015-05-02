EESchema Schematic File Version 2
LIBS:Lötpad_2,5mm
LIBS:power
LIBS:Bluetooth_ZS_HC_05
LIBS:Bluetooth_CZ_HC_05
LIBS:ds1307
LIBS:Wlan_ESP8266
LIBS:transistor-fet
LIBS:transistor-fet+irf7201
LIBS:transistors
LIBS:w_transistor
LIBS:diode
LIBS:diode-1
LIBS:led
LIBS:ardumower mega shield svn-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 6 17
Title "Ardumower shield SVN Version 1.3"
Date "Sonntag, 26. April 2015"
Rev "1.3"
Comp "von UweZ"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L CONN_3 P25
U 1 1 553B7663
P 4515 2720
F 0 "P25" V 4465 2720 50  0000 C CNN
F 1 "Res.Pin9" V 4565 2720 40  0000 C CNN
F 2 "ACS712:Pin_Header_Straight_1x03" H 4515 2720 60  0001 C CNN
F 3 "" H 4515 2720 60  0000 C CNN
F 4 "Value" H 4515 2720 60  0001 C CNN "Bestellnummer"
	1    4515 2720
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR041
U 1 1 553B76CF
P 5090 2720
F 0 "#PWR041" H 5090 2470 60  0001 C CNN
F 1 "GND" V 5090 2500 60  0000 C CNN
F 2 "" H 5090 2720 60  0000 C CNN
F 3 "" H 5090 2720 60  0000 C CNN
	1    5090 2720
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR042
U 1 1 553B771E
P 5090 2620
F 0 "#PWR042" H 5090 2470 60  0001 C CNN
F 1 "+5V" V 5090 2830 60  0000 C CNN
F 2 "" H 5090 2620 60  0000 C CNN
F 3 "" H 5090 2620 60  0000 C CNN
	1    5090 2620
	0    1    -1   0   
$EndComp
Text GLabel 5465 2820 2    60   Output ~ 0
ReservePin9
Wire Wire Line
	4865 2620 5090 2620
Wire Wire Line
	4865 2720 5090 2720
Wire Wire Line
	4865 2820 5465 2820
$Comp
L CONN_3 P26
U 1 1 553BA344
P 4515 3125
F 0 "P26" V 4465 3125 50  0000 C CNN
F 1 "Res.Pin4" V 4565 3125 40  0000 C CNN
F 2 "ACS712:Pin_Header_Straight_1x03" H 4515 3125 60  0001 C CNN
F 3 "" H 4515 3125 60  0000 C CNN
F 4 "Value" H 4515 3125 60  0001 C CNN "Bestellnummer"
	1    4515 3125
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR043
U 1 1 553BA34A
P 5090 3125
F 0 "#PWR043" H 5090 2875 60  0001 C CNN
F 1 "GND" V 5090 2905 60  0000 C CNN
F 2 "" H 5090 3125 60  0000 C CNN
F 3 "" H 5090 3125 60  0000 C CNN
	1    5090 3125
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR044
U 1 1 553BA350
P 5090 3025
F 0 "#PWR044" H 5090 2875 60  0001 C CNN
F 1 "+5V" V 5090 3235 60  0000 C CNN
F 2 "" H 5090 3025 60  0000 C CNN
F 3 "" H 5090 3025 60  0000 C CNN
	1    5090 3025
	0    1    -1   0   
$EndComp
Text GLabel 5465 3225 2    60   Output ~ 0
ReservePin4
Wire Wire Line
	4865 3025 5090 3025
Wire Wire Line
	4865 3125 5090 3125
Wire Wire Line
	4865 3225 5465 3225
$Comp
L CONN_3 P28
U 1 1 553BC8E0
P 4515 3530
F 0 "P28" V 4465 3530 50  0000 C CNN
F 1 "Res.Pin8" V 4565 3530 40  0000 C CNN
F 2 "ACS712:Pin_Header_Straight_1x03" H 4515 3530 60  0001 C CNN
F 3 "" H 4515 3530 60  0000 C CNN
F 4 "Value" H 4515 3530 60  0001 C CNN "Bestellnummer"
	1    4515 3530
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR045
U 1 1 553BC8E6
P 5090 3530
F 0 "#PWR045" H 5090 3280 60  0001 C CNN
F 1 "GND" V 5090 3310 60  0000 C CNN
F 2 "" H 5090 3530 60  0000 C CNN
F 3 "" H 5090 3530 60  0000 C CNN
	1    5090 3530
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR046
U 1 1 553BC8EC
P 5090 3430
F 0 "#PWR046" H 5090 3280 60  0001 C CNN
F 1 "+5V" V 5090 3640 60  0000 C CNN
F 2 "" H 5090 3430 60  0000 C CNN
F 3 "" H 5090 3430 60  0000 C CNN
	1    5090 3430
	0    1    -1   0   
$EndComp
Text GLabel 5465 3630 2    60   Output ~ 0
ReservePin8
Wire Wire Line
	4865 3430 5090 3430
Wire Wire Line
	4865 3530 5090 3530
Wire Wire Line
	4865 3630 5465 3630
$Comp
L CONN_3 P29
U 1 1 553BF1B4
P 4510 3900
F 0 "P29" V 4460 3900 50  0000 C CNN
F 1 "Res.Pin1" V 4560 3900 40  0000 C CNN
F 2 "ACS712:Pin_Header_Straight_1x03" H 4510 3900 60  0001 C CNN
F 3 "" H 4510 3900 60  0000 C CNN
F 4 "Value" H 4510 3900 60  0001 C CNN "Bestellnummer"
	1    4510 3900
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR047
U 1 1 553BF1BA
P 5085 3900
F 0 "#PWR047" H 5085 3650 60  0001 C CNN
F 1 "GND" V 5085 3680 60  0000 C CNN
F 2 "" H 5085 3900 60  0000 C CNN
F 3 "" H 5085 3900 60  0000 C CNN
	1    5085 3900
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR048
U 1 1 553BF1C0
P 5085 3800
F 0 "#PWR048" H 5085 3650 60  0001 C CNN
F 1 "+5V" V 5085 4010 60  0000 C CNN
F 2 "" H 5085 3800 60  0000 C CNN
F 3 "" H 5085 3800 60  0000 C CNN
	1    5085 3800
	0    1    -1   0   
$EndComp
Text GLabel 5460 4000 2    60   Output ~ 0
ReservePin1
Wire Wire Line
	4860 3800 5085 3800
Wire Wire Line
	4860 3900 5085 3900
Wire Wire Line
	4860 4000 5460 4000
$Comp
L CONN_3 P27
U 1 1 553C1023
P 4505 4325
F 0 "P27" V 4455 4325 50  0000 C CNN
F 1 "Res.Pin0" V 4555 4325 40  0000 C CNN
F 2 "ACS712:Pin_Header_Straight_1x03" H 4505 4325 60  0001 C CNN
F 3 "" H 4505 4325 60  0000 C CNN
F 4 "Value" H 4505 4325 60  0001 C CNN "Bestellnummer"
	1    4505 4325
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR049
U 1 1 553C1029
P 5080 4325
F 0 "#PWR049" H 5080 4075 60  0001 C CNN
F 1 "GND" V 5080 4105 60  0000 C CNN
F 2 "" H 5080 4325 60  0000 C CNN
F 3 "" H 5080 4325 60  0000 C CNN
	1    5080 4325
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR050
U 1 1 553C102F
P 5080 4225
F 0 "#PWR050" H 5080 4075 60  0001 C CNN
F 1 "+5V" V 5080 4435 60  0000 C CNN
F 2 "" H 5080 4225 60  0000 C CNN
F 3 "" H 5080 4225 60  0000 C CNN
	1    5080 4225
	0    1    -1   0   
$EndComp
Text GLabel 5455 4425 2    60   Output ~ 0
ReservePin0
Wire Wire Line
	4855 4225 5080 4225
Wire Wire Line
	4855 4325 5080 4325
Wire Wire Line
	4855 4425 5455 4425
$Comp
L CONN_3 P35
U 1 1 553C32D8
P 4505 4735
F 0 "P35" V 4455 4735 50  0000 C CNN
F 1 "Resv.Pin49" V 4555 4735 40  0000 C CNN
F 2 "ACS712:Pin_Header_Straight_1x03" H 4505 4735 60  0001 C CNN
F 3 "" H 4505 4735 60  0000 C CNN
F 4 "Value" H 4505 4735 60  0001 C CNN "Bestellnummer"
	1    4505 4735
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR051
U 1 1 553C32DE
P 5080 4735
F 0 "#PWR051" H 5080 4485 60  0001 C CNN
F 1 "GND" V 5080 4515 60  0000 C CNN
F 2 "" H 5080 4735 60  0000 C CNN
F 3 "" H 5080 4735 60  0000 C CNN
	1    5080 4735
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR052
U 1 1 553C32E4
P 5080 4635
F 0 "#PWR052" H 5080 4485 60  0001 C CNN
F 1 "+5V" V 5080 4845 60  0000 C CNN
F 2 "" H 5080 4635 60  0000 C CNN
F 3 "" H 5080 4635 60  0000 C CNN
	1    5080 4635
	0    1    -1   0   
$EndComp
Text GLabel 5455 4835 2    60   Output ~ 0
ReservePin49
Wire Wire Line
	4855 4635 5080 4635
Wire Wire Line
	4855 4735 5080 4735
Wire Wire Line
	4855 4835 5455 4835
$Comp
L CONN_3 P22
U 1 1 553C570A
P 4505 5155
F 0 "P22" V 4455 5155 50  0000 C CNN
F 1 "Res.AD6" V 4555 5155 40  0000 C CNN
F 2 "ACS712:Pin_Header_Straight_1x03" H 4505 5155 60  0001 C CNN
F 3 "" H 4505 5155 60  0000 C CNN
F 4 "Value" H 4505 5155 60  0001 C CNN "Bestellnummer"
	1    4505 5155
	-1   0    0    -1  
$EndComp
$Comp
L GND #PWR053
U 1 1 553C5710
P 5080 5155
F 0 "#PWR053" H 5080 4905 60  0001 C CNN
F 1 "GND" V 5080 4935 60  0000 C CNN
F 2 "" H 5080 5155 60  0000 C CNN
F 3 "" H 5080 5155 60  0000 C CNN
	1    5080 5155
	0    -1   -1   0   
$EndComp
$Comp
L +5V #PWR054
U 1 1 553C5716
P 5080 5055
F 0 "#PWR054" H 5080 4905 60  0001 C CNN
F 1 "+5V" V 5080 5265 60  0000 C CNN
F 2 "" H 5080 5055 60  0000 C CNN
F 3 "" H 5080 5055 60  0000 C CNN
	1    5080 5055
	0    1    -1   0   
$EndComp
Text GLabel 5455 5255 2    60   Output ~ 0
ReserveAD6
Wire Wire Line
	4855 5055 5080 5055
Wire Wire Line
	4855 5155 5080 5155
Wire Wire Line
	4855 5255 5455 5255
$EndSCHEMATC