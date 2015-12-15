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
LIBS:LPC1114
LIBS:mcp1700
LIBS:mb-cache
EELAYER 25 0
EELAYER END
$Descr A0 46811 33110
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
L LPC1114/302 U1
U 1 1 56698C7B
P 18700 10300
F 0 "U1" H 18700 10350 60  0000 C CNN
F 1 "LPC1114/302" H 18700 10450 60  0000 C CNN
F 2 "Housings_QFP:TQFP-48_7x7mm_Pitch0.5mm" H 18700 10300 60  0001 C CNN
F 3 "" H 18700 10300 60  0000 C CNN
	1    18700 10300
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 56698DB9
P 16900 7350
F 0 "SW1" H 17050 7460 50  0000 C CNN
F 1 "SW_PUSH" H 16900 7270 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_PTS645" H 16900 7350 60  0001 C CNN
F 3 "" H 16900 7350 60  0000 C CNN
	1    16900 7350
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 56698ECA
P 16900 7650
F 0 "SW2" H 17050 7760 50  0000 C CNN
F 1 "SW_PUSH" H 16900 7570 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_PTS645" H 16900 7650 60  0001 C CNN
F 3 "" H 16900 7650 60  0000 C CNN
	1    16900 7650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR01
U 1 1 56698F0D
P 16600 7850
F 0 "#PWR01" H 16600 7600 50  0001 C CNN
F 1 "GND" H 16600 7700 50  0000 C CNN
F 2 "" H 16600 7850 60  0000 C CNN
F 3 "" H 16600 7850 60  0000 C CNN
	1    16600 7850
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 56698F4D
P 17250 7050
F 0 "R1" V 17330 7050 50  0000 C CNN
F 1 "R" V 17250 7050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 17180 7050 30  0001 C CNN
F 3 "" H 17250 7050 30  0000 C CNN
	1    17250 7050
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 56698FFC
P 17400 7050
F 0 "R2" V 17480 7050 50  0000 C CNN
F 1 "R" V 17400 7050 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 17330 7050 30  0001 C CNN
F 3 "" H 17400 7050 30  0000 C CNN
	1    17400 7050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 566991BF
P 18550 8300
F 0 "#PWR02" H 18550 8050 50  0001 C CNN
F 1 "GND" H 18550 8150 50  0000 C CNN
F 2 "" H 18550 8300 60  0000 C CNN
F 3 "" H 18550 8300 60  0000 C CNN
	1    18550 8300
	-1   0    0    1   
$EndComp
NoConn ~ 18600 12200
NoConn ~ 18800 12200
$Comp
L CONN_02X04 P1
U 1 1 566992EF
P 11650 8950
F 0 "P1" H 11650 9200 50  0000 C CNN
F 1 "CONN_02X04" H 11650 8700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_2x04" H 11650 7750 60  0001 C CNN
F 3 "" H 11650 7750 60  0000 C CNN
	1    11650 8950
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X05 P2
U 1 1 5669948B
P 14200 5700
F 0 "P2" H 14200 6000 50  0000 C CNN
F 1 "CONN_01X05" V 14300 5700 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 14200 5700 60  0001 C CNN
F 3 "" H 14200 5700 60  0000 C CNN
	1    14200 5700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR03
U 1 1 566995A1
P 14000 6050
F 0 "#PWR03" H 14000 5800 50  0001 C CNN
F 1 "GND" H 14000 5900 50  0000 C CNN
F 2 "" H 14000 6050 60  0000 C CNN
F 3 "" H 14000 6050 60  0000 C CNN
	1    14000 6050
	1    0    0    -1  
$EndComp
NoConn ~ 14300 5900
Text Label 14800 6650 0    60   ~ 0
TX
Text Label 14800 6500 0    60   ~ 0
RX
Text Label 16850 11300 0    60   ~ 0
TX
Text Label 16850 11200 0    60   ~ 0
RX
Wire Wire Line
	16600 7350 16600 7850
Connection ~ 16600 7650
Wire Wire Line
	17200 7650 17400 7650
Wire Wire Line
	17400 7650 17400 9000
Wire Wire Line
	17400 9000 17750 9000
Wire Wire Line
	17750 8900 17500 8900
Wire Wire Line
	17500 8900 17500 7350
Wire Wire Line
	17500 7350 17200 7350
Wire Wire Line
	17400 7200 17400 7350
Connection ~ 17400 7350
Wire Wire Line
	17250 7200 17250 7650
Connection ~ 17250 7650
Wire Wire Line
	17400 6900 17250 6900
Wire Wire Line
	17300 6800 17300 6900
Connection ~ 17300 6900
Wire Wire Line
	18800 8350 18900 8350
Wire Wire Line
	18600 8350 18500 8350
Wire Wire Line
	18550 8300 18550 8350
Connection ~ 18550 8350
Wire Wire Line
	18850 8300 18850 8350
Connection ~ 18850 8350
Wire Wire Line
	14400 6050 14400 5900
Wire Wire Line
	14000 6050 14000 5900
Wire Wire Line
	14200 5900 14200 6500
Wire Wire Line
	14200 6500 15100 6500
Wire Wire Line
	14100 5900 14100 6650
Wire Wire Line
	14100 6650 15100 6650
Wire Wire Line
	17750 9100 16700 9100
Wire Wire Line
	17750 9200 16700 9200
Wire Wire Line
	16150 9300 17750 9300
Wire Wire Line
	15950 9400 17750 9400
Wire Wire Line
	17750 9500 16700 9500
Wire Wire Line
	17750 9600 16700 9600
Wire Wire Line
	17750 9700 16700 9700
Wire Wire Line
	17750 9800 16700 9800
Wire Wire Line
	17750 9900 16700 9900
Wire Wire Line
	17750 10000 16700 10000
Wire Wire Line
	17750 10600 16700 10600
Wire Wire Line
	17750 10700 16700 10700
Wire Wire Line
	17750 10800 16700 10800
Wire Wire Line
	17750 10900 16700 10900
Wire Wire Line
	17750 11000 16700 11000
Wire Wire Line
	17750 11100 16700 11100
Wire Wire Line
	17750 11200 16700 11200
Wire Wire Line
	17750 11300 16700 11300
Wire Wire Line
	17750 11400 16700 11400
Wire Wire Line
	17750 11500 16700 11500
Wire Wire Line
	17750 11600 16700 11600
Wire Wire Line
	17750 11700 16700 11700
Wire Wire Line
	20650 8900 19600 8900
Wire Wire Line
	19600 9000 20650 9000
Wire Wire Line
	19600 9100 20650 9100
Wire Wire Line
	19600 9200 20650 9200
Wire Wire Line
	19600 9300 20650 9300
Wire Wire Line
	19600 9400 20650 9400
Wire Wire Line
	19600 10600 20650 10600
Wire Wire Line
	19600 10700 20650 10700
Wire Wire Line
	19600 10800 20650 10800
Wire Wire Line
	19600 10900 20650 10900
Wire Wire Line
	19600 11000 20650 11000
Wire Wire Line
	19600 11100 20650 11100
Wire Wire Line
	19600 11200 20650 11200
Wire Wire Line
	19600 11300 20650 11300
Wire Wire Line
	19600 11400 20650 11400
Wire Wire Line
	19600 11500 20650 11500
Wire Wire Line
	19600 11600 20650 11600
Wire Wire Line
	19600 11700 20650 11700
Text Label 16850 9700 0    60   ~ 0
MISO
Text Label 16850 9800 0    60   ~ 0
MOSI
Text Label 16850 9900 0    60   ~ 0
SCK
Text Label 20150 10800 0    60   ~ 0
RADIO_CE
Text Label 20100 11600 0    60   ~ 0
RADIO_CSN
Wire Wire Line
	11400 8800 10900 8800
Wire Wire Line
	11400 8900 10900 8900
Wire Wire Line
	11400 9000 10900 9000
Wire Wire Line
	11400 9100 10900 9100
Wire Wire Line
	12400 8800 11900 8800
Wire Wire Line
	11900 8900 12400 8900
Wire Wire Line
	11900 9000 12400 9000
Wire Wire Line
	11900 9100 12400 9100
$Comp
L GND #PWR04
U 1 1 5669A123
P 10900 8800
F 0 "#PWR04" H 10900 8550 50  0001 C CNN
F 1 "GND" H 10900 8650 50  0000 C CNN
F 2 "" H 10900 8800 60  0000 C CNN
F 3 "" H 10900 8800 60  0000 C CNN
	1    10900 8800
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR05
U 1 1 5669A145
P 12400 8800
F 0 "#PWR05" H 12400 8650 50  0001 C CNN
F 1 "+3V3" H 12400 8940 50  0000 C CNN
F 2 "" H 12400 8800 60  0000 C CNN
F 3 "" H 12400 8800 60  0000 C CNN
	1    12400 8800
	0    1    1    0   
$EndComp
Text Label 10950 8900 0    60   ~ 0
RADIO_CE
Text Label 11950 8900 0    60   ~ 0
RADIO_CSN
Text Label 10950 9000 0    60   ~ 0
SCK
Text Label 11950 9000 0    60   ~ 0
MOSI
Text Label 10950 9100 0    60   ~ 0
MISO
NoConn ~ 12400 9100
$Comp
L CP C2
U 1 1 5669A994
P 18000 6650
F 0 "C2" H 18025 6750 50  0000 L CNN
F 1 "CP" H 18025 6550 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_4x5.7" H 18038 6500 30  0001 C CNN
F 3 "" H 18000 6650 60  0000 C CNN
	1    18000 6650
	1    0    0    -1  
$EndComp
$Comp
L CP C1
U 1 1 5669AA24
P 17700 6650
F 0 "C1" H 17725 6750 50  0000 L CNN
F 1 "CP" H 17725 6550 50  0000 L CNN
F 2 "Capacitors_SMD:c_elec_4x5.7" H 17738 6500 30  0001 C CNN
F 3 "" H 17700 6650 60  0000 C CNN
	1    17700 6650
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 5669AA7C
P 19850 6650
F 0 "C4" H 19875 6750 50  0000 L CNN
F 1 "1u" H 19875 6550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 19888 6500 30  0001 C CNN
F 3 "" H 19850 6650 60  0000 C CNN
	1    19850 6650
	1    0    0    -1  
$EndComp
$Comp
L C C6
U 1 1 5669AB21
P 20150 6650
F 0 "C6" H 20175 6750 50  0000 L CNN
F 1 "1u" H 20175 6550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 20188 6500 30  0001 C CNN
F 3 "" H 20150 6650 60  0000 C CNN
	1    20150 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	18900 7450 19150 7450
Wire Wire Line
	19150 7750 18900 7750
$Comp
L +3V3 #PWR06
U 1 1 5669AC7A
P 19050 7400
F 0 "#PWR06" H 19050 7250 50  0001 C CNN
F 1 "+3V3" H 19050 7540 50  0000 C CNN
F 2 "" H 19050 7400 60  0000 C CNN
F 3 "" H 19050 7400 60  0000 C CNN
	1    19050 7400
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR07
U 1 1 5669B018
P 17300 6800
F 0 "#PWR07" H 17300 6650 50  0001 C CNN
F 1 "+3V3" H 17300 6940 50  0000 C CNN
F 2 "" H 17300 6800 60  0000 C CNN
F 3 "" H 17300 6800 60  0000 C CNN
	1    17300 6800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR08
U 1 1 5669B144
P 18850 8300
F 0 "#PWR08" H 18850 8150 50  0001 C CNN
F 1 "+3V3" H 18850 8440 50  0000 C CNN
F 2 "" H 18850 8300 60  0000 C CNN
F 3 "" H 18850 8300 60  0000 C CNN
	1    18850 8300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5669B173
P 19050 7800
F 0 "#PWR09" H 19050 7550 50  0001 C CNN
F 1 "GND" H 19050 7650 50  0000 C CNN
F 2 "" H 19050 7800 60  0000 C CNN
F 3 "" H 19050 7800 60  0000 C CNN
	1    19050 7800
	1    0    0    -1  
$EndComp
Wire Wire Line
	19050 7800 19050 7750
Connection ~ 19050 7750
Wire Wire Line
	19050 7400 19050 7450
Connection ~ 19050 7450
Wire Wire Line
	17700 6500 18000 6500
Wire Wire Line
	17700 6800 18000 6800
Wire Wire Line
	18650 6500 20150 6500
Connection ~ 18950 6500
Connection ~ 19250 6500
Connection ~ 19550 6500
Connection ~ 19850 6500
Wire Wire Line
	18650 6800 20150 6800
Connection ~ 18950 6800
Connection ~ 19250 6800
Connection ~ 19550 6800
Connection ~ 19850 6800
$Comp
L +3V3 #PWR010
U 1 1 5669B81E
P 19400 6400
F 0 "#PWR010" H 19400 6250 50  0001 C CNN
F 1 "+3V3" H 19400 6540 50  0000 C CNN
F 2 "" H 19400 6400 60  0000 C CNN
F 3 "" H 19400 6400 60  0000 C CNN
	1    19400 6400
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 5669B85C
P 19400 6900
F 0 "#PWR011" H 19400 6650 50  0001 C CNN
F 1 "GND" H 19400 6750 50  0000 C CNN
F 2 "" H 19400 6900 60  0000 C CNN
F 3 "" H 19400 6900 60  0000 C CNN
	1    19400 6900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR012
U 1 1 5669B89A
P 17900 6900
F 0 "#PWR012" H 17900 6650 50  0001 C CNN
F 1 "GND" H 17900 6750 50  0000 C CNN
F 2 "" H 17900 6900 60  0000 C CNN
F 3 "" H 17900 6900 60  0000 C CNN
	1    17900 6900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR013
U 1 1 5669B8D8
P 17900 6400
F 0 "#PWR013" H 17900 6250 50  0001 C CNN
F 1 "+3V3" H 17900 6540 50  0000 C CNN
F 2 "" H 17900 6400 60  0000 C CNN
F 3 "" H 17900 6400 60  0000 C CNN
	1    17900 6400
	1    0    0    -1  
$EndComp
Wire Wire Line
	17900 6400 17900 6500
Connection ~ 17900 6500
Wire Wire Line
	17900 6900 17900 6800
Connection ~ 17900 6800
Wire Wire Line
	19400 6400 19400 6500
Connection ~ 19400 6500
Wire Wire Line
	19400 6900 19400 6800
Connection ~ 19400 6800
Text Label 16850 9400 0    60   ~ 0
SDA
Text Label 16850 9300 0    60   ~ 0
SCL
$Comp
L R R4
U 1 1 566ABFBD
P 16150 9150
F 0 "R4" V 16230 9150 50  0000 C CNN
F 1 "4k7" V 16150 9150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 16080 9150 30  0001 C CNN
F 3 "" H 16150 9150 30  0000 C CNN
	1    16150 9150
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 566AC1FF
P 15950 9150
F 0 "R3" V 16030 9150 50  0000 C CNN
F 1 "4k7" V 15950 9150 50  0000 C CNN
F 2 "Resistors_SMD:R_0805_HandSoldering" V 15880 9150 30  0001 C CNN
F 3 "" H 15950 9150 30  0000 C CNN
	1    15950 9150
	1    0    0    -1  
$EndComp
Wire Wire Line
	16150 9000 15950 9000
$Comp
L +3V3 #PWR014
U 1 1 566AC2F1
P 16050 8900
F 0 "#PWR014" H 16050 8750 50  0001 C CNN
F 1 "+3V3" H 16050 9040 50  0000 C CNN
F 2 "" H 16050 8900 60  0000 C CNN
F 3 "" H 16050 8900 60  0000 C CNN
	1    16050 8900
	1    0    0    -1  
$EndComp
Wire Wire Line
	16050 8900 16050 9000
Connection ~ 16050 9000
Wire Wire Line
	15950 9400 15950 9300
$Comp
L +3V3 #PWR015
U 1 1 566AC758
P 14400 6050
F 0 "#PWR015" H 14400 5900 50  0001 C CNN
F 1 "+3V3" H 14400 6190 50  0000 C CNN
F 2 "" H 14400 6050 60  0000 C CNN
F 3 "" H 14400 6050 60  0000 C CNN
	1    14400 6050
	-1   0    0    1   
$EndComp
$Comp
L mcp1700 U2
U 1 1 566ADFB7
P 22400 6550
F 0 "U2" H 22400 6550 60  0000 C CNN
F 1 "mcp1700" H 22400 6650 60  0000 C CNN
F 2 "Housings_SOT-23_SOT-143_TSOT-6:SOT-23_Handsoldering" H 22400 6550 60  0001 C CNN
F 3 "" H 22400 6550 60  0000 C CNN
	1    22400 6550
	1    0    0    -1  
$EndComp
$Comp
L CONN_01X02 P5
U 1 1 566AE14F
P 23300 5950
F 0 "P5" H 23300 6100 50  0000 C CNN
F 1 "CONN_01X02" V 23400 5950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 23300 5950 60  0001 C CNN
F 3 "" H 23300 5950 60  0000 C CNN
	1    23300 5950
	0    -1   -1   0   
$EndComp
$Comp
L CONN_01X02 P4
U 1 1 566AE268
P 21500 5950
F 0 "P4" H 21500 6100 50  0000 C CNN
F 1 "CONN_01X02" V 21600 5950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 21500 5950 60  0001 C CNN
F 3 "" H 21500 5950 60  0000 C CNN
	1    21500 5950
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR016
U 1 1 566AE350
P 23600 6550
F 0 "#PWR016" H 23600 6400 50  0001 C CNN
F 1 "+3V3" H 23600 6690 50  0000 C CNN
F 2 "" H 23600 6550 60  0000 C CNN
F 3 "" H 23600 6550 60  0000 C CNN
	1    23600 6550
	0    1    1    0   
$EndComp
$Comp
L GND #PWR017
U 1 1 566AE45D
P 22400 7150
F 0 "#PWR017" H 22400 6900 50  0001 C CNN
F 1 "GND" H 22400 7000 50  0000 C CNN
F 2 "" H 22400 7150 60  0000 C CNN
F 3 "" H 22400 7150 60  0000 C CNN
	1    22400 7150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR018
U 1 1 566AE4B4
P 21550 6250
F 0 "#PWR018" H 21550 6000 50  0001 C CNN
F 1 "GND" H 21550 6100 50  0000 C CNN
F 2 "" H 21550 6250 60  0000 C CNN
F 3 "" H 21550 6250 60  0000 C CNN
	1    21550 6250
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR019
U 1 1 566AE504
P 23350 6250
F 0 "#PWR019" H 23350 6000 50  0001 C CNN
F 1 "GND" H 23350 6100 50  0000 C CNN
F 2 "" H 23350 6250 60  0000 C CNN
F 3 "" H 23350 6250 60  0000 C CNN
	1    23350 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	21450 6550 21850 6550
Wire Wire Line
	21450 6150 21450 6550
Wire Wire Line
	21550 6150 21550 6250
Wire Wire Line
	22950 6550 23600 6550
Wire Wire Line
	23250 6150 23250 6550
Connection ~ 23250 6550
Wire Wire Line
	23350 6150 23350 6250
Wire Wire Line
	22400 7150 22400 7050
$Comp
L C C12
U 1 1 566AE98F
P 23550 6350
F 0 "C12" H 23575 6450 50  0000 L CNN
F 1 "1u" H 23575 6250 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 23588 6200 30  0001 C CNN
F 3 "" H 23550 6350 60  0000 C CNN
	1    23550 6350
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 566AEAAC
P 21750 6350
F 0 "C11" H 21775 6450 50  0000 L CNN
F 1 "1u" H 21775 6250 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 21788 6200 30  0001 C CNN
F 3 "" H 21750 6350 60  0000 C CNN
	1    21750 6350
	1    0    0    -1  
$EndComp
Wire Wire Line
	21750 6500 21750 6550
Connection ~ 21750 6550
Wire Wire Line
	21550 6200 21750 6200
Wire Wire Line
	23550 6550 23550 6500
Connection ~ 23550 6550
Wire Wire Line
	23550 6200 23350 6200
Connection ~ 23350 6200
Connection ~ 21550 6200
$Comp
L CONN_01X08 P6
U 1 1 566AFCCD
P 26150 9150
F 0 "P6" H 26150 9600 50  0000 C CNN
F 1 "CONN_01X08" V 26250 9150 50  0000 C CNN
F 2 "Connectors_Molex:Connector_Molex_PicoBlade_53398-0871" H 26150 9150 60  0001 C CNN
F 3 "" H 26150 9150 60  0000 C CNN
	1    26150 9150
	1    0    0    -1  
$EndComp
Wire Wire Line
	25950 8800 25100 8800
Wire Wire Line
	25950 8900 25100 8900
Wire Wire Line
	25950 9100 25100 9100
Wire Wire Line
	25950 9300 25100 9300
Wire Wire Line
	25950 9500 25100 9500
Text Label 25200 8800 0    60   ~ 0
SCL
Text Label 25200 8900 0    60   ~ 0
SDA
$Comp
L +3V3 #PWR020
U 1 1 566B05B2
P 25100 9100
F 0 "#PWR020" H 25100 8950 50  0001 C CNN
F 1 "+3V3" H 25100 9240 50  0000 C CNN
F 2 "" H 25100 9100 60  0000 C CNN
F 3 "" H 25100 9100 60  0000 C CNN
	1    25100 9100
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR021
U 1 1 566B062E
P 25100 9300
F 0 "#PWR021" H 25100 9050 50  0001 C CNN
F 1 "GND" H 25100 9150 50  0000 C CNN
F 2 "" H 25100 9300 60  0000 C CNN
F 3 "" H 25100 9300 60  0000 C CNN
	1    25100 9300
	0    1    1    0   
$EndComp
Text Label 25150 9500 0    60   ~ 0
IMUA0
NoConn ~ 25950 9000
NoConn ~ 25950 9200
NoConn ~ 25950 9400
$Comp
L CONN_01X08 P7
U 1 1 566B0CD6
P 26150 10200
F 0 "P7" H 26150 10650 50  0000 C CNN
F 1 "CONN_01X08" V 26250 10200 50  0000 C CNN
F 2 "Connectors_Molex:Connector_Molex_PicoBlade_53398-0871" H 26150 10200 60  0001 C CNN
F 3 "" H 26150 10200 60  0000 C CNN
	1    26150 10200
	1    0    0    -1  
$EndComp
Wire Wire Line
	25950 9850 25100 9850
Wire Wire Line
	25950 9950 25100 9950
Wire Wire Line
	25950 10150 25100 10150
Wire Wire Line
	25950 10350 25100 10350
Wire Wire Line
	25950 10550 25100 10550
Text Label 25200 9850 0    60   ~ 0
SCL
Text Label 25200 9950 0    60   ~ 0
SDA
$Comp
L +3V3 #PWR022
U 1 1 566B0CE3
P 25100 10150
F 0 "#PWR022" H 25100 10000 50  0001 C CNN
F 1 "+3V3" H 25100 10290 50  0000 C CNN
F 2 "" H 25100 10150 60  0000 C CNN
F 3 "" H 25100 10150 60  0000 C CNN
	1    25100 10150
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR023
U 1 1 566B0CE9
P 25100 10350
F 0 "#PWR023" H 25100 10100 50  0001 C CNN
F 1 "GND" H 25100 10200 50  0000 C CNN
F 2 "" H 25100 10350 60  0000 C CNN
F 3 "" H 25100 10350 60  0000 C CNN
	1    25100 10350
	0    1    1    0   
$EndComp
Text Label 25150 10550 0    60   ~ 0
IMUA1
NoConn ~ 25950 10050
NoConn ~ 25950 10250
NoConn ~ 25950 10450
$Comp
L CONN_01X08 P8
U 1 1 566B0D69
P 26150 11400
F 0 "P8" H 26150 11850 50  0000 C CNN
F 1 "CONN_01X08" V 26250 11400 50  0000 C CNN
F 2 "Connectors_Molex:Connector_Molex_PicoBlade_53398-0871" H 26150 11400 60  0001 C CNN
F 3 "" H 26150 11400 60  0000 C CNN
	1    26150 11400
	1    0    0    -1  
$EndComp
Wire Wire Line
	25950 11050 25100 11050
Wire Wire Line
	25950 11150 25100 11150
Wire Wire Line
	25950 11350 25100 11350
Wire Wire Line
	25950 11550 25100 11550
Wire Wire Line
	25950 11750 25100 11750
Text Label 25200 11050 0    60   ~ 0
SCL
Text Label 25200 11150 0    60   ~ 0
SDA
$Comp
L +3V3 #PWR024
U 1 1 566B0D76
P 25100 11350
F 0 "#PWR024" H 25100 11200 50  0001 C CNN
F 1 "+3V3" H 25100 11490 50  0000 C CNN
F 2 "" H 25100 11350 60  0000 C CNN
F 3 "" H 25100 11350 60  0000 C CNN
	1    25100 11350
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR025
U 1 1 566B0D7C
P 25100 11550
F 0 "#PWR025" H 25100 11300 50  0001 C CNN
F 1 "GND" H 25100 11400 50  0000 C CNN
F 2 "" H 25100 11550 60  0000 C CNN
F 3 "" H 25100 11550 60  0000 C CNN
	1    25100 11550
	0    1    1    0   
$EndComp
Text Label 25150 11750 0    60   ~ 0
IMUA2
NoConn ~ 25950 11250
NoConn ~ 25950 11450
NoConn ~ 25950 11650
$Comp
L CONN_01X08 P9
U 1 1 566B0EAC
P 26150 12550
F 0 "P9" H 26150 13000 50  0000 C CNN
F 1 "CONN_01X08" V 26250 12550 50  0000 C CNN
F 2 "Connectors_Molex:Connector_Molex_PicoBlade_53398-0871" H 26150 12550 60  0001 C CNN
F 3 "" H 26150 12550 60  0000 C CNN
	1    26150 12550
	1    0    0    -1  
$EndComp
Wire Wire Line
	25950 12200 25100 12200
Wire Wire Line
	25950 12300 25100 12300
Wire Wire Line
	25950 12500 25100 12500
Wire Wire Line
	25950 12700 25100 12700
Wire Wire Line
	25950 12900 25100 12900
Text Label 25200 12200 0    60   ~ 0
SCL
Text Label 25200 12300 0    60   ~ 0
SDA
$Comp
L +3V3 #PWR026
U 1 1 566B0EB9
P 25100 12500
F 0 "#PWR026" H 25100 12350 50  0001 C CNN
F 1 "+3V3" H 25100 12640 50  0000 C CNN
F 2 "" H 25100 12500 60  0000 C CNN
F 3 "" H 25100 12500 60  0000 C CNN
	1    25100 12500
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR027
U 1 1 566B0EBF
P 25100 12700
F 0 "#PWR027" H 25100 12450 50  0001 C CNN
F 1 "GND" H 25100 12550 50  0000 C CNN
F 2 "" H 25100 12700 60  0000 C CNN
F 3 "" H 25100 12700 60  0000 C CNN
	1    25100 12700
	0    1    1    0   
$EndComp
Text Label 25150 12900 0    60   ~ 0
IMUA3
NoConn ~ 25950 12600
NoConn ~ 25950 12800
$Comp
L CONN_01X08 P10
U 1 1 566B10E1
P 26150 13600
F 0 "P10" H 26150 14050 50  0000 C CNN
F 1 "CONN_01X08" V 26250 13600 50  0000 C CNN
F 2 "Connectors_Molex:Connector_Molex_PicoBlade_53398-0871" H 26150 13600 60  0001 C CNN
F 3 "" H 26150 13600 60  0000 C CNN
	1    26150 13600
	1    0    0    -1  
$EndComp
Wire Wire Line
	25950 13250 25100 13250
Wire Wire Line
	25950 13350 25100 13350
Wire Wire Line
	25950 13550 25100 13550
Wire Wire Line
	25950 13750 25100 13750
Wire Wire Line
	25950 13950 25100 13950
Text Label 25200 13250 0    60   ~ 0
SCL
Text Label 25200 13350 0    60   ~ 0
SDA
$Comp
L +3V3 #PWR028
U 1 1 566B10EE
P 25100 13550
F 0 "#PWR028" H 25100 13400 50  0001 C CNN
F 1 "+3V3" H 25100 13690 50  0000 C CNN
F 2 "" H 25100 13550 60  0000 C CNN
F 3 "" H 25100 13550 60  0000 C CNN
	1    25100 13550
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR029
U 1 1 566B10F4
P 25100 13750
F 0 "#PWR029" H 25100 13500 50  0001 C CNN
F 1 "GND" H 25100 13600 50  0000 C CNN
F 2 "" H 25100 13750 60  0000 C CNN
F 3 "" H 25100 13750 60  0000 C CNN
	1    25100 13750
	0    1    1    0   
$EndComp
Text Label 25150 13950 0    60   ~ 0
IMUA4
NoConn ~ 25950 13650
NoConn ~ 25950 13850
$Comp
L CONN_01X08 P11
U 1 1 566B10FE
P 26150 14750
F 0 "P11" H 26150 15200 50  0000 C CNN
F 1 "CONN_01X08" V 26250 14750 50  0000 C CNN
F 2 "Connectors_Molex:Connector_Molex_PicoBlade_53398-0871" H 26150 14750 60  0001 C CNN
F 3 "" H 26150 14750 60  0000 C CNN
	1    26150 14750
	1    0    0    -1  
$EndComp
Wire Wire Line
	25950 14400 25100 14400
Wire Wire Line
	25950 14500 25100 14500
Wire Wire Line
	25950 14700 25100 14700
Wire Wire Line
	25950 14900 25100 14900
Wire Wire Line
	25950 15100 25100 15100
Text Label 25200 14400 0    60   ~ 0
SCL
Text Label 25200 14500 0    60   ~ 0
SDA
$Comp
L +3V3 #PWR030
U 1 1 566B110B
P 25100 14700
F 0 "#PWR030" H 25100 14550 50  0001 C CNN
F 1 "+3V3" H 25100 14840 50  0000 C CNN
F 2 "" H 25100 14700 60  0000 C CNN
F 3 "" H 25100 14700 60  0000 C CNN
	1    25100 14700
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR031
U 1 1 566B1111
P 25100 14900
F 0 "#PWR031" H 25100 14650 50  0001 C CNN
F 1 "GND" H 25100 14750 50  0000 C CNN
F 2 "" H 25100 14900 60  0000 C CNN
F 3 "" H 25100 14900 60  0000 C CNN
	1    25100 14900
	0    1    1    0   
$EndComp
Text Label 25150 15100 0    60   ~ 0
IMUA5
NoConn ~ 25950 14600
NoConn ~ 25950 14800
NoConn ~ 25950 15000
Text Label 20300 11500 0    60   ~ 0
IMUA4
Text Label 16850 9600 0    60   ~ 0
IMUA3
Text Label 16850 9500 0    60   ~ 0
IMUA2
Text Label 20150 9400 0    60   ~ 0
IMUA1
Text Label 16850 9200 0    60   ~ 0
IMUA0
Text Label 20300 10700 0    60   ~ 0
IMUA5
$Comp
L C C3
U 1 1 566B2DBA
P 18650 6650
F 0 "C3" H 18675 6750 50  0000 L CNN
F 1 "1u" H 18675 6550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 18688 6500 30  0001 C CNN
F 3 "" H 18650 6650 60  0000 C CNN
	1    18650 6650
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 566B2E6B
P 18950 6650
F 0 "C5" H 18975 6750 50  0000 L CNN
F 1 "1u" H 18975 6550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 18988 6500 30  0001 C CNN
F 3 "" H 18950 6650 60  0000 C CNN
	1    18950 6650
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 566B2EDC
P 19250 6650
F 0 "C7" H 19275 6750 50  0000 L CNN
F 1 "1u" H 19275 6550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 19288 6500 30  0001 C CNN
F 3 "" H 19250 6650 60  0000 C CNN
	1    19250 6650
	1    0    0    -1  
$EndComp
$Comp
L C C8
U 1 1 566B2FB5
P 19550 6650
F 0 "C8" H 19575 6750 50  0000 L CNN
F 1 "1u" H 19575 6550 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 19588 6500 30  0001 C CNN
F 3 "" H 19550 6650 60  0000 C CNN
	1    19550 6650
	1    0    0    -1  
$EndComp
$Comp
L C C9
U 1 1 566B3021
P 18900 7600
F 0 "C9" H 18925 7700 50  0000 L CNN
F 1 "100n" H 18925 7500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 18938 7450 30  0001 C CNN
F 3 "" H 18900 7600 60  0000 C CNN
	1    18900 7600
	1    0    0    -1  
$EndComp
$Comp
L C C10
U 1 1 566B309A
P 19150 7600
F 0 "C10" H 19175 7700 50  0000 L CNN
F 1 "100n" H 19175 7500 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 19188 7450 30  0001 C CNN
F 3 "" H 19150 7600 60  0000 C CNN
	1    19150 7600
	1    0    0    -1  
$EndComp
$Comp
L C C13
U 1 1 566B349B
P 13250 8950
F 0 "C13" H 13275 9050 50  0000 L CNN
F 1 "100n" H 13275 8850 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805_HandSoldering" H 13288 8800 30  0001 C CNN
F 3 "" H 13250 8950 60  0000 C CNN
	1    13250 8950
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR032
U 1 1 566B3DAF
P 13250 9200
F 0 "#PWR032" H 13250 8950 50  0001 C CNN
F 1 "GND" H 13250 9050 50  0000 C CNN
F 2 "" H 13250 9200 60  0000 C CNN
F 3 "" H 13250 9200 60  0000 C CNN
	1    13250 9200
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR033
U 1 1 566B3E23
P 13250 8700
F 0 "#PWR033" H 13250 8550 50  0001 C CNN
F 1 "+3V3" H 13250 8840 50  0000 C CNN
F 2 "" H 13250 8700 60  0000 C CNN
F 3 "" H 13250 8700 60  0000 C CNN
	1    13250 8700
	1    0    0    -1  
$EndComp
Wire Wire Line
	13250 8700 13250 8800
Wire Wire Line
	13250 9200 13250 9100
$Comp
L GND #PWR034
U 1 1 566B79D2
P 20650 11000
F 0 "#PWR034" H 20650 10750 50  0001 C CNN
F 1 "GND" H 20650 10850 50  0000 C CNN
F 2 "" H 20650 11000 60  0000 C CNN
F 3 "" H 20650 11000 60  0000 C CNN
	1    20650 11000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR035
U 1 1 566B7B88
P 20650 11100
F 0 "#PWR035" H 20650 10850 50  0001 C CNN
F 1 "GND" H 20650 10950 50  0000 C CNN
F 2 "" H 20650 11100 60  0000 C CNN
F 3 "" H 20650 11100 60  0000 C CNN
	1    20650 11100
	0    -1   -1   0   
$EndComp
$Comp
L +3V3 #PWR036
U 1 1 566B7BF2
P 20650 9300
F 0 "#PWR036" H 20650 9150 50  0001 C CNN
F 1 "+3V3" H 20650 9440 50  0000 C CNN
F 2 "" H 20650 9300 60  0000 C CNN
F 3 "" H 20650 9300 60  0000 C CNN
	1    20650 9300
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR037
U 1 1 566B7E70
P 16700 11500
F 0 "#PWR037" H 16700 11350 50  0001 C CNN
F 1 "+3V3" H 16700 11640 50  0000 C CNN
F 2 "" H 16700 11500 60  0000 C CNN
F 3 "" H 16700 11500 60  0000 C CNN
	1    16700 11500
	0    -1   -1   0   
$EndComp
Text Label 25200 12400 0    60   ~ 0
IMUA3
Wire Wire Line
	25950 12400 25100 12400
$Comp
L GND #PWR038
U 1 1 566B8817
P 25100 13450
F 0 "#PWR038" H 25100 13200 50  0001 C CNN
F 1 "GND" H 25100 13300 50  0000 C CNN
F 2 "" H 25100 13450 60  0000 C CNN
F 3 "" H 25100 13450 60  0000 C CNN
	1    25100 13450
	0    1    1    0   
$EndComp
Wire Wire Line
	25950 13450 25100 13450
$Comp
L R R5
U 1 1 566B8EEA
P 12050 7100
F 0 "R5" V 12130 7100 50  0000 C CNN
F 1 "0" V 12050 7100 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 11980 7100 30  0001 C CNN
F 3 "" H 12050 7100 30  0000 C CNN
	1    12050 7100
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR039
U 1 1 566B916A
P 12050 6900
F 0 "#PWR039" H 12050 6750 50  0001 C CNN
F 1 "+3V3" H 12050 7040 50  0000 C CNN
F 2 "" H 12050 6900 60  0000 C CNN
F 3 "" H 12050 6900 60  0000 C CNN
	1    12050 6900
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR040
U 1 1 566B91DC
P 12050 7300
F 0 "#PWR040" H 12050 7150 50  0001 C CNN
F 1 "+3V3" H 12050 7440 50  0000 C CNN
F 2 "" H 12050 7300 60  0000 C CNN
F 3 "" H 12050 7300 60  0000 C CNN
	1    12050 7300
	-1   0    0    1   
$EndComp
Wire Wire Line
	12050 6900 12050 6950
Wire Wire Line
	12050 7250 12050 7300
NoConn ~ 20650 8900
NoConn ~ 20650 9000
NoConn ~ 20650 9100
NoConn ~ 20650 9200
NoConn ~ 20650 10600
NoConn ~ 20650 10900
NoConn ~ 20650 11200
NoConn ~ 20650 11300
NoConn ~ 20650 11400
NoConn ~ 20650 11700
NoConn ~ 16700 11700
NoConn ~ 16700 11600
NoConn ~ 16700 11400
NoConn ~ 16700 11100
NoConn ~ 16700 11000
NoConn ~ 16700 10900
NoConn ~ 16700 10800
NoConn ~ 16700 10700
NoConn ~ 16700 10600
NoConn ~ 16700 10000
NoConn ~ 16700 9100
$EndSCHEMATC