#!/usr/bin/python
# -*- coding: utf-8 -*-
#

#ESP01 - via Serial leads to kernel panic. especially if ESP01 is trying to send message to pi while pi is booting or shutting down.
#Hence, communicating with ESP32 via UART will also be ruled out. Best bet is using ESP NOW. WHich means - a secondary networking interface will be needed.
'''
	ESP32 Usable GPIO Pins
	Usable Output Pins	Notes
	GPIO 2	OK, but connected to LED on some boards.
	GPIO 4	Safe for output.
	GPIO 5	Safe, often used for SPI SS (can be used normally).
	GPIO 12	Safe for output. Be careful if using strapping pins.
	GPIO 13	Safe for output.
	GPIO 14	Safe for output.
	GPIO 15	Safe for output.
	GPIO 18	Safe for SPI and normal output.
	GPIO 19	Safe for SPI and normal output.
	GPIO 21	Safe for output.
	GPIO 22	Safe for output.
	GPIO 23	Safe for output.
	GPIO 25	Safe for output (DAC available).
	GPIO 26	Safe for output (DAC available).
	GPIO 27	Safe for output.
	GPIO 32	Safe for output (RTC capable).
	GPIO 33	Safe for output (RTC capable).
	
	Usable Input Pins	Notes
	GPIO 2	Can read input, but connected to on-board LED on some boards.
	GPIO 4	Safe for input.
	GPIO 5	Safe for input.
	GPIO 12	Safe for input, but avoid pulling HIGH at boot (strapping pin).
	GPIO 13	Safe for input.
	GPIO 14	Safe for input.
	GPIO 15	Safe for input.
	GPIO 16	Safe for input.
	GPIO 17	Safe for input.
	GPIO 18	Safe for input.
	GPIO 19	Safe for input.
	GPIO 21	Safe for input.
	GPIO 22	Safe for input.
	GPIO 23	Safe for input.
	GPIO 25	Safe for input.
	GPIO 26	Safe for input.
	GPIO 27	Safe for input.
	GPIO 32	Safe for input (RTC capable).
	GPIO 33	Safe for input (RTC capable).
	GPIO 34	Input only (cannot be output).
	GPIO 35	Input only (cannot be output).
	GPIO 36	Input only (cannot be output).
	GPIO 39	Input only (cannot be output).
ESP32PLCHUB programmed with
#define TOTAL_O_PINS 13  // Number of output pins
#define TOTAL_I_PINS 5  // Number of input pins

// Define the GPIO pins you want to control
int outputPins[TOTAL_O_PINS] = {4, 5, 16, 17, 18, 19, 21, 22, 23, 26,27, 32, 33 };
int inputPins[TOTAL_I_PINS] = {13,14,25,36,39};

GPIO	High (1)	Low (0)
4	0x09	0x08
5	0x0B	0x0A
16	0x21	0x20
17	0x23	0x22
18	0x25	0x24
19	0x27	0x26
21	0x2B	0x2A
22	0x2D	0x2C
23	0x2F	0x2E
26	0x35	0x34
27	0x37	0x36
32	0x41	0x40
33	0x43	0x42


	'''


import time
import os
import shutil
import datetime 
import smbus
import math
import signal
import socket
from threading import Thread
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
import binascii
import struct

import serial
from collections import deque
from math import sqrt
import subprocess

mylocationdir="/home/pi/Downloads/sensor-server/"


#### GLOBAL VARIABLES ######
TCP_IP=''
WS_PORT= 8000
TCP_PORT=9999

running_flag = True #used for graceful shutdown 
watchdog_flag = False#used for watchdog

#global commandQ
commandQ=[]

# global value for plotchart
mydict ={}


# Configuration (migrating to sensor-server)
SERIAL_PORT = '/dev/serial0'  # Replace with your serial port
BAUD_RATE = 115200
MAC_ADDRESS_LENGTH = 6
PACKET_ID_LENGTH = 2
HEADER_LENGTH = MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH + 3  # MAC + Packet ID + Total Packets, Sequence, Payload Length
FOOTER_LENGTH = 2  # Checksum length
PACKET_DELIMITER = '\x0D\x0A'  # Delimiter: 0D 0A
DATA_TYPE_HEARTBEAT = '\xFF'
DATA_TYPE_ADC0 = '\x10'
DATA_TYPE_ADC1 = '\x11'
# Path for the Unix Domain Socket
ESP_UDS_PATH = "/tmp/raw_socket_uds_esp"
LORA_UDS_PATH = "/tmp/raw_socket_uds_lora"

# ESP32 MAC address for identification
ESP32_MAC_ADDR = '\x58\xBF\x25\x82\x8E\xD8'  # Replace with the actual MAC address
BROADCAST_MAC_ADDR = '\xFF\xFF\xFF\xFF\xFF\xFF'


# Set to track received Packet IDs
PACKET_ID_TRACKER = deque(maxlen=50)  # FIFO queue with a max size of 50

# Dictionary to store segmented packets
pending_segments = {}

# Initialize a global variable to track LED state
esp32_status_led_state = False  # False means OFF, True means ON
LoRa_status_led_state = False  # False means OFF, True means ON

#global Switch for using ESP01 via serial vs Direct Pi Zero Wifi Packet injection to connect with ESP nodes
ESP01 = False # True means using Serial, False means using raw socket packet injection

# Create a Unix domain socket to receive data from the C program
esp_uds_socket = None
lora_uds_socket = None

#Create a global Serial device
ser = None





#MAC Addresses of sensornodes # loads from settings file or user can input from webinterface for the first run
sensortotankattachmentdict={}
# Define the dictionary with actuator addresses
actuatoraddressdict = {
	"ACTUATOR_1_ADDRESS": '\xAA',
	"ACTUATOR_2_ADDRESS": '\xBB',
	"ACTUATOR_3_ADDRESS": '\xCC'
}

# Global dictionary to track the STATUS LED state of each actuator
actuator_status_led_state = {
	"ACTUATOR_1": False,  # False means OFF, True means ON
	"ACTUATOR_2": False,
	"ACTUATOR_3": False
}

ACTUATOR_GPIO_OUTPUT_MAP = {
	"SWTANK1": (1, 8),
	"SWTANK1RELAY": (1, 7),
	"STATUS_LED1": (1, 4),
	"SWTANKB": (2, 5),
	"SWTANKBRELAY": (2, 6),
	"STATUS_LEDB": (2, 8)
}

ACTUATOR_STATUS_MAP = {
	"STATUSTANK1": (1, 0),
	"STATUSTANK2": (1, 1),
	"STATUSTANK3": (2, 0)
}

#WebSocket OPCODES
STREAM = 0x0
TEXT = 0x1
BINARY = 0x2
CLOSE = 0x8
PING = 0x9
PONG = 0xA
#WebSocket clients
clients = []
 
# ESP32 GPIO mapping dictionary
GPIO_OUTPUT_MAP = {
	"SWSTARTPB": 23,
	"SWSTOPPB": 22,
	"SWTANK": 4,
	"SWTANKRELAY": 23,
	"ESP32_STATUS_LED": 5
}
GPIO_INPUT_MAP = {
	"STATUSMODE": 13,
	#"STATUSTANK1": 14,
	#"STATUSTANK2": 36,
}

# Fixed sequence of ESP32 input GPIOs corresponding to bits in the received byte
INPUT_PINS_SEQUENCE = [13, 14, 25, 36, 39]

STATUSMODE = False
STATUSTANK1 = False
STATUSTANK2 = False

#STATUSMODE=23 # Computer vs Human (High = Human). THis is a hardware switch on the board
#STATUSTANK1=21 
#STATUSTANK2=22 
#SWSTARTPB=13 # High = start PB pressed
#SWSTOPPB=14# High = stop PB pressed
#SWTANK=15
#SWTANKRELAY=27 #Cuts off the 12V supply to actuator. Actuator stays in position. 

SENSOR_PACKET_LENGTH=16 # Non-ESP32 packets longer than this size are discarded
ACTUATOR_NODE_PACKET_LENGTH=2 # Tank Actuator ATMEGA sneds 2 bytes, 1 for Address and 1 for status

#SOFTWARE MODE
SOFTMODE="Auto" # Turn off motor if AC voltage low or high, motor current too high, 
#tank levels high, switch tanks. Turn on Motor if Tank level low. 

#define initial AC Voltage and current
ACVOLTAGE=110
MOTORCURRENT=0.756
#Expose the voltage and current midrail ADC values (aka offset) and ratios to user via settings so that it can be calibrated
#V_RATIO=110.0/140.0 # ADC Value of 140 for RMS AC Voltage  of 110 V
#I_RATIO=0.21/17.5 # ADC Value of 17.5 for RMS AC Current of 0.21 A
offsetV=529.0
offsetI=512.0
VoltCalibrate=110.0
VadcValue=140.0
CurrentCalibrate=0.21
CadcValue=17.5
#V_RATIO=VoltCalibrate/VadcValue
#I_RATIO=CurrentCalibrate/CadcValue
#Turn off motor if voltage too high or too low
HVLVL=350 ##CHANGE##
LVLVL=110

LCD_REFRESH_INTERVAL = 1.5 # LCD numbers change every this many seconds
#Turn off motor if current draw is too high
HMCURR=7 # At the start of the motor - current draw is going to be high

#Added on 2/19/18 - MONCURR is only used in "backup" motor status detection in myanalogread. If motor is drawing more than this much current - motor is considered to be on.
MONCURR=19 # Jan 2022 - was 0.9 - changed it to 19 for debugging

#Set Max and initial Tank Level ADC reads
MAXT1=100.0#MM Values. how many Centimeters from HCSR04 when the tank 1 is full. 
MAXT2=100.0 # decimal point necessary so that this is treated as float when calculations are done

MINT1=1990.0 # Adjust max and min for each sensor, Min level is basically offset
MINT2=1990.0 # Tanklevel in % = (sensorvalue-min)/(max-min)*100

T1LLVL=75 # less than this % value - switch tank or motor on
T1HLVL=95

T2LLVL=75 # less than this % value - switch tank or motor on
T2HLVL=95

TANK1LEVEL=111 # Define Tank level variable - initial value
TANK2LEVEL=111

TANK1AVERAGINGLIST=[85.0,85.0,85.0,85.0,85.0] # use the running average of the list to determine whether or not to turn on the motor. Not using it for turning the motor off - one value higher than set enough.
TANK2AVERAGINGLIST=[85.0,85.0,85.0,85.0,85.0]

myTANK1AVERAGELEVEL = sum(TANK1AVERAGINGLIST)/len(TANK1AVERAGINGLIST)
myTANK2AVERAGELEVEL = sum(TANK2AVERAGINGLIST)/len(TANK2AVERAGINGLIST)

timebetweensensordata = 20 # Time in seconds - if the same sensor sends data with less than this much time gap from previous data - ignore that data
SENSORTIMEOUT=120 # Time in seconds - for which if the sensor hasn't posted data - it will be considered to be "Down"

WATERDRAWTIMELIMIT=90 # Time is seconds before watchdog is triggerred - if no increase in either tank levels and motor has been on for this long.
MOTORONTIMELIMIT=8*60 # 8 minutes maximum time for motor to be on in Auto mode regardless of tank level.

#SENSOR1TIME=time.time()-SENSORTIMEOUT # Define initial update time for the sensors#
#SENSOR2TIME=time.time()-SENSORTIMEOUT# 60 seconds prior to start of the script - so that - if sensor is down at the runtime-auto mode won't progress
#2/19/18 updates - global sensor status - start with assumption that sensors are down.
#3-6-18 - due to sensor being down - even if saved softmode is auto - it switched to manual. hence start with sensors being up.
SENSOR1TIME=time.time()-timebetweensensordata+5# OR datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
SENSOR2TIME=time.time()-timebetweensensordata+5# OR datetime.now().strftime("%Y-%m-%d %H:%M:%S")
#SO in the beginning both sensors are assumed to be down - as soon as a sensor reading is received - it will be set to True 
IsSENSOR1UP=False
IsSENSOR2UP=False
#This is used if SOFTMODE=Manual and client wants to shutoff the motor after filling the tanks
MANUALMODESHUTOFF="false"
TANK1MANUALHIGHLEVEL=85
TANK2MANUALHIGHLEVEL=85
#1/19/18 For calibration purposes - need to send raw values (can back calculate on client - but this is cleaner)
raw_RMS_Current_ADC=999
raw_RMS_Voltage_ADC=999
raw_Sensor_1_Reading=999
raw_Sensor_2_Reading=999
#Defining global battery levels
battery1level=5000
battery2level=5000
#Defining global temp levels
TANK1temp=10
TANK2temp=10
#Arrays to hold rawADC values
sampleVArray=[]
sampleIArray=[]
#MCP3008 Analog pins - Hardwired
inPinV=0
inPinI=7
#Defining global variables
MODE="DEFINE"
ACPOWER="OFF"
MOTOR="OFF"
TANK="DEFINE"

#Define Season
SUMMER=True
HASMOTORBEENONTODAY = False
MOTORONTIMESTAMP=time.time()

TANK1TIMEDFULL = False
TANK2TIMEDFULL = False

TANK1FILLINGSTARTTIME = time.time()
TANK2FILLINGSTARTTIME = time.time()


def get_ip_address(ifname):
	try:
		output = subprocess.check_output("ifconfig {}".format(ifname), shell=True).decode()
		for line in output.split("\n"):
			if "inet " in line:
				return line.split()[1]  # Extracts IP address
		return "Disconnected"
	except Exception as e:
		error_handler(get_ip_address.__name__, str(e))
	return "Disconnected"



def is_connected():
	"""Check internet connectivity by connecting to a known server."""
	try:
		socket.create_connection(("8.8.8.8", 53), timeout=2)
		return True
	except OSError:
		return False


#Read initial GPIO status
def init_status():
	#since we are MODIFYING global variables - need to use "global" keyword
	global MODE
	global ACPOWER
	global MOTOR
	global TANK
	global SUMMER
	TODAY = datetime.datetime.today()
	if (STATUSMODE):
		MODE="HUMAN"
	else:
		MODE="COMPUTER"
	#1/7/18 - With optocoupler - logic is reversed - On signal, output goes to ground
	if (STATUSTANK1):
		TANK="Tank 1"
	elif (STATUSTANK2):
		TANK="Tank 2"
	else:
		TANK="undefined"
	Month = TODAY.month
	if (3<Month<9):
		SUMMER=True
	else:
		SUMMER=False
#Function called by change in STATUSMODE updated by heartbeat data from ESP32
def modeswitch():
	global MODE
	if (STATUSMODE):
		MODE="HUMAN"
	else:
		MODE="COMPUTER"
	sendchangedstatus("MODE="+MODE)

#Function called by change in STATUSTANK1 and STATUSTANK2 updated by heartbeat data from ESP32
def tankswitch():
	global TANK
	if (STATUSTANK1):
		TANK="Tank 1"
	elif (STATUSTANK2):
		TANK="Tank 2"
	else:
		TANK="undefined"
	sendchangedstatus("TANK="+TANK)
#Function used to send the changed statuses to the client via Websocket
def sendchangedstatus(mymsg):
	try:
		for ws in clients:
			ws.sendMessage(u'STATUS#'+mymsg)
		activity_handler(mymsg)
	except Exception as e:
		error_handler(sendchangedstatus.__name__,str(e))
		pass
## Commands are handled in the order they are received - this function is invoked by commandthread
def commandhandler(command):
	global SOFTMODE
	global MOTORONTIMESTAMP
	try:
		#global variable declaration needed if modifying global variables - not needed if just using global variables
		if (command.split("=")[0]=="MOTOR"):
			if (ACPOWER=="ON"):# Execute motor commands only if ACPOWER is ON
				if (command.split("=")[1]=="ON"):
					if (MOTOR=="OFF"):
						#Build Safety - Stable voltage for 5 seconds(with in 20 volts), initial current draw
						initialACVOLTAGE=ACVOLTAGE
						time.sleep(5) # THIS WILL BLOCK THE SCRIPT EXECUTION FOR 5 SECONDS
						finalACVOLTAGE=ACVOLTAGE
						if ((HVLVL>finalACVOLTAGE>LVLVL) and ((finalACVOLTAGE-initialACVOLTAGE)<abs(20))):
							ESP32send("SWSTARTPB","HIGH") 
							time.sleep(3)
							activity_handler("Motor On")
							if (MOTOR=="ON"): #wait 3 seconds for Motor status to change to ON
								ESP32send("SWSTARTPB","LOW") # then release the STARTPB 
								#Check motor current draw and turn off motor if current > limit
								if (MOTORCURRENT>HMCURR):
									#Motor is ON but current is high - turn off the Motor
									ESP32send("SWSTOPPB","HIGH") #Press STOP PB
									sendchangedstatus("ERROR=MOTORHIGHCURRENT")
									error_handler(commandhandler.__name__,"MOTORHIGHCURRENT")
									#In case of any error while running in "SOFTMODE=Auto" , switch to SOFTMODE=Manual and notify the connected clients
									#3-10-18 - TODO - if the motor is ON and in Auto mode - any error - will switch the mode to manual and motor will never turn off since the user doesn't know that motor is on and mode is now manual.
									if (SOFTMODE=="Auto"):
										SOFTMODE="Manual"
										sendchangedstatus("SOFTMODE="+SOFTMODE)
									time.sleep(3) # Wait for 3 seconds
									if (MOTOR=="OFF"): # If motor turned off
										ESP32send("SWSTOPPB","LOW") # Release STOP PB
								#Motor has been turned ON successfully. Set the motoroontimestamp
								MOTORONTIMESTAMP=time.time()
								if (TANK=="Tank 1"):
									TANK1FILLINGSTARTTIME = time.time()
								elif (TANK=="Tank 2"):
									TANK2FILLINGSTARTTIME = time.time()
							else:#Motor didn't start,
								ESP32send("SWSTARTPB","LOW") #  Release START PB and send Error
								sendchangedstatus("ERROR=MOTORSTART")
								error_handler(commandhandler.__name__,"MOTORSTART")
								#In case of any error while running in "SOFTMODE=Auto" , switch to SOFTMODE=Manual and notify the connected clients
								#3-10-18 - TODO - if the motor is ON and in Auto mode - any error - will switch the mode to manual and motor will never turn off since the user doesn't know that motor is on and mode is now manual.
								if (SOFTMODE=="Auto"):
									SOFTMODE="Manual"
									sendchangedstatus("SOFTMODE="+SOFTMODE)
						else:# Doesn't meet voltage criteria - unstable or too low/too high
							sendchangedstatus("ERROR=MOTORNOTSTABLEVOLTAGE")
							error_handler(commandhandler.__name__,"MOTORNOTSTABLEVOLTAGE")
							#In case of any error while running in "SOFTMODE=Auto" , switch to SOFTMODE=Manual and notify the connected clients
							#3-10-18 - TODO - if the motor is ON and in Auto mode - any error - will switch the mode to manual and motor will never turn off since the user doesn't know that motor is on and mode is now manual.
							if (SOFTMODE=="Auto"):
								SOFTMODE="Manual"
								sendchangedstatus("SOFTMODE="+SOFTMODE)
				if (command.split("=")[1]=="OFF"):
					if (MOTOR=="ON"):
						ESP32send("SWSTOPPB","HIGH") #Press STOP PB
						time.sleep(3) # Wait for 3 seconds
						if (MOTOR=="OFF"): # If Motor turned off
							ESP32send("SWSTOPPB","LOW") # Release STOP PB
							activity_handler("Motor Off")
						else: # Motor didn't stop
							ESP32send("SWSTOPPB","LOW") # Release STOP PB and send error
							sendchangedstatus("ERROR=MOTORSTOP")
							error_handler(commandhandler.__name__,"MOTORSTOP")
							#In case of any error while running in "SOFTMODE=Auto" , switch to SOFTMODE=Manual and notify the connected clients
							#3-10-18 - TODO - if the motor is ON and in Auto mode - any error - will switch the mode to manual and motor will never turn off since the user doesn't know that motor is on and mode is now manual.
							if (SOFTMODE=="Auto"):
								SOFTMODE="Manual"
								sendchangedstatus("SOFTMODE="+SOFTMODE)
							#TURN OFF A MASTER POWER SWITCH TO PROTECT THE MOTOR
		if (command.split("=")[0]=="TANK"):
			if (command.split("=")[1]=="Tank 2"):
				if (TANK=="Tank 1"):
					ESP32send("SWTANK","HIGH")
					#send_msg_to_LoRaNode('/xAA,/xCC')
					if (MOTOR=="ON"):
						TANK1FILLINGSTARTTIME = time.time()
					#activity_handler("Tank 2") #Using the statuschange of Tank instead to capture human mode actions
			if (command.split("=")[1]=="Tank 1"):
				if (TANK=="Tank 2"):
					ESP32send("SWTANK","LOW")
					if (MOTOR=="ON"):
						TANK2FILLINGSTARTTIME = time.time()
					#activity_handler("Tank 1") #Using the statuschange of Tank instead to capture human mode actions
		if (command.split("=")[0]=="dMOTOR"):
			if (ACPOWER=="ON"):# Execute motor commands only if ACPOWER is ON
				if (command.split("=")[1]=="ON"):
					if (MOTOR=="OFF"):
						#Build Safety - Stable voltage for 5 seconds(with in 20 volts), initial current draw
						initialACVOLTAGE=ACVOLTAGE
						time.sleep(5)
						finalACVOLTAGE=ACVOLTAGE
						if ((HVLVL>finalACVOLTAGE>LVLVL) and ((finalACVOLTAGE-initialACVOLTAGE)<abs(20))):
							ESP32send("SWSTARTPB","HIGH")
						else:# Doesn't meet voltage criteria - unstable or too low/too high
							sendchangedstatus("ERROR=MOTORNOTSTABLEVOLTAGE")
				if (command.split("=")[1]=="OFF"):
					if (MOTOR=="ON"):
						ESP32send("SWSTARTPB","LOW")
	except Exception as e:
		error_handler(commandhandler.__name__,str(e))
		pass
###SENSOR VALUES ARE RECEIVED, SAVED and UPDATED by THIS FUNCTION
def sensor_data_handler(data):
	try:
		global TANK1LEVEL
		global TANK2LEVEL
		global SENSOR1TIME
		global SENSOR2TIME
		global raw_Sensor_1_Reading
		global raw_Sensor_2_Reading
		global battery1level
		global battery2level
		global TANK1temp
		global TANK2temp
		global myTANK1AVERAGELEVEL
		global myTANK2AVERAGELEVEL
		if (data):
			request = data
			currtime=time.time()
			strcurrtime = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(currtime))
			macaddrofdata = binascii.hexlify(request[0:6])
			if (macaddrofdata==sensortotankattachmentdict['T1MAC']):
				#check if we just received data from the same macaddr
				if((currtime-SENSOR1TIME)>timebetweensensordata):
					SENSOR1TIME=currtime
					#round(float((y-x)/(z-x)*100),1) - float to 1 decimal
					myTANK1LEVEL=round((((MINT1-float(struct.unpack('h',request[6:8])[0]))/(MINT1-MAXT1))*100),1)
					if (myTANK1LEVEL>0):
						TANK1LEVEL=myTANK1LEVEL
						TANK1AVERAGINGLIST.pop(0)
						TANK1AVERAGINGLIST.append(myTANK1LEVEL)
					#TANK1LEVEL=sensorvalue.split(':')[1]+"%"# For sending RAW CM OUTPUT TO CLIENT FOR CALIBRATION
					raw_Sensor_1_Reading=struct.unpack('h',request[6:8])[0]
					battery1level=struct.unpack('h',request[10:12])[0]
					myTANK1temp = struct.unpack('h',request[8:10])[0]
					if (myTANK1temp<70):
						TANK1temp=myTANK1temp
					message2=u'SensorData#'+strcurrtime+u'|T1MAC|'+str(TANK1LEVEL)+u'|'+str(battery1level)+u'|'+str(TANK1temp)
					message1=strcurrtime+'|T1MAC|'+str(TANK1LEVEL)+'|'+str(battery1level)+'|'+str(TANK1temp)
					for ws in clients:
						ws.sendMessage(message2)
					savesensordatatofile(message1)
			elif (macaddrofdata==sensortotankattachmentdict['T2MAC']):
				if((currtime-SENSOR2TIME)>timebetweensensordata):
					SENSOR2TIME=currtime
					#round(float((y-x)/(z-x)*100),1) - float to 1 decimal
					myTANK2LEVEL=round((((MINT2-float(struct.unpack('h',request[6:8])[0]))/(MINT2-MAXT2))*100),1)
					if (myTANK2LEVEL>0):
						TANK2LEVEL=myTANK2LEVEL
						TANK2AVERAGINGLIST.pop(0)
						TANK2AVERAGINGLIST.append(myTANK2LEVEL)
					raw_Sensor_2_Reading=struct.unpack('h',request[6:8])[0]
					battery2level=struct.unpack('h',request[10:12])[0]
					myTANK2temp = struct.unpack('h',request[8:10])[0]
					if (myTANK2temp<70):
						TANK2temp=myTANK2temp
					message2=u'SensorData#'+strcurrtime+u'|T2MAC|'+str(TANK2LEVEL)+u'|'+str(battery2level)+u'|'+str(TANK2temp)
					message1=strcurrtime+'|T2MAC|'+str(TANK2LEVEL)+'|'+str(battery2level)+'|'+str(TANK2temp)
					for ws in clients:
						ws.sendMessage(message2)
					savesensordatatofile(message1)
			else:
				#MAC address of the received data doesn't match any nodes we know of so just send this data to the client console
				message2=u'SensorData#'+strcurrtime+u'|'+macaddrofdata+'|'+str(struct.unpack('h',request[6:8])[0])+u'|'+str(struct.unpack('h',request[10:12])[0])+u'|'+str(struct.unpack('h',request[8:10])[0])
				for ws in clients:
					ws.sendMessage(message2)
			if (len(request)>16):
				nofchunks=len(request)/16
				i=0
				while i < nofchunks:
					if (binascii.hexlify(request[0+i*16:6+i*16])!=macaddrofdata):
						if (binascii.hexlify(request[0+i*16:6+i*16])==sensortotankattachmentdict['T1MAC']):
							#check if we just received data from the same macaddr
							if((currtime-SENSOR1TIME)>timebetweensensordata):
								SENSOR1TIME=currtime
								#round(float((y-x)/(z-x)*100),1) - float to 1 decimal
								myTANK1LEVEL=round((((MINT1-float(struct.unpack('h',request[6+i*16:8+i*16])[0]))/(MINT1-MAXT1))*100),1)
								if (myTANK1LEVEL>0):
									TANK1LEVEL=myTANK1LEVEL
									TANK1AVERAGINGLIST.pop(0)
									TANK1AVERAGINGLIST.append(myTANK1LEVEL)
								raw_Sensor_1_Reading=struct.unpack('h',request[6+i*16:8+i*16])[0]
								battery1level=struct.unpack('h',request[10+i*16:12+i*16])[0]
								myTANK1temp = struct.unpack('h',request[8+i*16:10+i*16])[0]
								if (myTANK1temp<70):
									TANK1temp=myTANK1temp
								message2=u'SensorData#'+strcurrtime+u'|T1MAC|'+str(TANK1LEVEL)+u'|'+str(battery1level)+u'|'+str(TANK1temp)
								message1=strcurrtime+'|T1MAC|'+str(TANK1LEVEL)+'|'+str(battery1level)+'|'+str(TANK1temp)
								for ws in clients:
									ws.sendMessage(message2)
								savesensordatatofile(message1)
						elif (binascii.hexlify(request[0+i*16:6+i*16])==sensortotankattachmentdict['T2MAC']):
							if((currtime-SENSOR2TIME)>timebetweensensordata):
								SENSOR2TIME=currtime
								#round(float((y-x)/(z-x)*100),1) - float to 1 decimal
								myTANK2LEVEL=round((((MINT2-float(struct.unpack('h',request[6+i*16:8+i*16])[0]))/(MINT2-MAXT2))*100),1)
								if (myTANK2LEVEL>0):
									TANK2LEVEL=myTANK2LEVEL
									TANK2AVERAGINGLIST.pop(0)
									TANK2AVERAGINGLIST.append(myTANK2LEVEL)
								raw_Sensor_2_Reading=struct.unpack('h',request[6+i*16:8+i*16])[0]
								battery2level=struct.unpack('h',request[10+i*16:12+i*16])[0]
								myTANK2temp = struct.unpack('h',request[8+i*16:10+i*16])[0]
								if (myTANK2temp<70):
									TANK2temp=myTANK2temp
								message2=u'SensorData#'+strcurrtime+u'|T2MAC|'+str(TANK2LEVEL)+u'|'+str(battery2level)+u'|'+str(TANK2temp)
								message1=strcurrtime+'|T2MAC|'+str(TANK2LEVEL)+'|'+str(battery2level)+'|'+str(TANK2temp)
								for ws in clients:
									ws.sendMessage(message2)
								savesensordatatofile(message1)
						else:
							#MAC address of the received data doesn't match any nodes we know of so just send this data to the client console
							message2=u'SensorData#'+strcurrtime+u'|'+binascii.hexlify(request[0+i*16:6+i*16])+'|'+str(struct.unpack('h',request[6+i*16:8+i*16])[0])+u'|'+str(struct.unpack('h',request[10+i*16:12+i*16])[0])+u'|'+str(struct.unpack('h',request[8+i*16:10+i*16])[0])
							for ws in clients:
								ws.sendMessage(message2)
						break
					i+=1
		myTANK1AVERAGELEVEL = sum(TANK1AVERAGINGLIST)/len(TANK1AVERAGINGLIST)
		myTANK2AVERAGELEVEL = sum(TANK2AVERAGINGLIST)/len(TANK2AVERAGINGLIST)
	except Exception as e:
		error_handler(sensor_data_handler.__name__,str(e))
		pass
def savesensordatatofile(formattedsensordata):
	try:
		fobj = open(mylocationdir+"sensordata"+time.strftime("%Y-%m-%d",time.localtime()), 'a+')
		fobj.write(formattedsensordata)
		fobj.write('\n')
		fobj.close()
	except Exception as e:
		error_handler(savesensordatatofile.__name__,str(e))
		pass
## Sending the raw ADC values to the client - to plot- and use it to calibrate
def send_raw_adc(param):
	try:
		global sampleVArray
		global sampleIArray
		#myPin=inPinV # default pin is voltage pin
		if param=="VOLTAGE":
			return sampleVArray
		#	myPin=inPinV
		elif param == "CURRENT":
			return sampleIArray
	except Exception as e:
		error_handler(send_raw_adc.__name__,str(e))
		pass

##This function loads and saves the voltage and current calibration settings
def calibrationhandler(vals,operation):
	try:
		global offsetI
		global offsetV
		global VadcValue
		global CadcValue
		global CurrentCalibrate
		global VoltCalibrate
		if (operation=="bakup"):
			#Create Bakup of existing calibration
			#offsetI=512,offsetV=529,Vcal=110,Vadc=140,Ccal=0.21,Cadc=17.5
			sobj = open(mylocationdir+"ADCcalibrationbakup", 'w')
			sobj.write("offsetI="+str(offsetI)+",")
			sobj.write("offsetV="+str(offsetV)+",")
			sobj.write("Vcal="+str(VoltCalibrate)+",")
			sobj.write("Vadc="+str(VadcValue)+",")
			sobj.write("Ccal="+str(CurrentCalibrate)+",")
			sobj.write("Cadc="+str(CadcValue))
			sobj.close()
		if (operation=="save"):
			if not ((vals=="null") or (vals=="")):
				mylist=vals.split(",")
				del mylist[-1] #delete the last comma
				valsdict={}
				for eachval in mylist: #vals get passed on as offsetI=512,offsetV=529,Vcal=110,Vadc=140,Ccal=0.21,Cadc=17.5  - this split converts it in to a list
					valsdict[eachval.split("=")[0]]=eachval.split("=")[1]
				#offsetI=512,offsetV=529,Vcal=110,Vadc=140,Ccal=0.21,Cadc=17.5
				#Set the values for current instance of the script
				offsetI=float(valsdict['offsetI'])
				offsetV=float(valsdict['offsetV'])
				VoltCalibrate=float(valsdict['Vcal'])
				VadcValue=float(valsdict['Vadc'])
				CurrentCalibrate=float(valsdict['Ccal'])
				CadcValue=float(valsdict['Cadc'])
			#Save the values for next run
			sobj = open(mylocationdir+"ADCcalibration", 'w')
			sobj.write("offsetI="+str(offsetI)+",")
			sobj.write("offsetV="+str(offsetV)+",")
			sobj.write("Vcal="+str(VoltCalibrate)+",")
			sobj.write("Vadc="+str(VadcValue)+",")
			sobj.write("Ccal="+str(CurrentCalibrate)+",")
			sobj.write("Cadc="+str(CadcValue))
			sobj.close()
		if (operation=="load"):
			sobj = open(mylocationdir+"ADCcalibration", 'r')
			valsdata=sobj.readline()
			valsdict={}
			#offsetI=512,offsetV=529,Vcal=110,Vadc=140,Ccal=0.21,Cadc=17.5
			for eachval in valsdata.split(","): 
				valsdict[eachval.split("=")[0]]=eachval.split("=")[1]
			#load the values from the file for current instance of the script
			offsetI=float(valsdict['offsetI'])
			offsetV=float(valsdict['offsetV'])
			VoltCalibrate=float(valsdict['Vcal'])
			VadcValue=float(valsdict['Vadc'])
			CurrentCalibrate=float(valsdict['Ccal'])
			CadcValue=float(valsdict['Cadc'])
			sobj.close()
		if (operation=="revert"):
			sobj = open(mylocationdir+"ADCcalibrationbakup", 'r')
			valsdata=sobj.readline()
			valsdict={}
			for eachval in valsdata.split(","): 
				valsdict[eachval.split("=")[0]]=eachval.split("=")[1]
			#load the values from the file for current instance of the script
			offsetI=float(valsdict['offsetI'])
			offsetV=float(valsdict['offsetV'])
			VoltCalibrate=float(valsdict['Vcal'])
			VadcValue=float(valsdict['Vadc'])
			CurrentCalibrate=float(valsdict['Ccal'])
			CadcValue=float(valsdict['Cadc'])
			sobj.close()
	except Exception as e:
		error_handler(calibrationhandler.__name__,str(e))
		pass
##This function loads and saves the general settings (tank high/low etc)
def settingshandler(settings,operation):
	try:
		global SOFTMODE
		global HVLVL
		global HMCURR
		global T1HLVL
		global T1LLVL
		global T2HLVL
		global T2LLVL
		global LVLVL
		global MAXT1
		global MAXT2
		global MINT1
		global MINT2
		global MONCURR
		global sensortotankattachmentdict
		if (operation=="bakup"):
			#Create Bakup of existing settings
			sobj = open(mylocationdir+"sensorsettingsbakup", 'w')
			sobj.write("SOFTMODE="+str(SOFTMODE)+"\n")
			sobj.write("HVLVL="+str(HVLVL)+"\n")
			sobj.write("HMCURR="+str(HMCURR)+"\n")
			sobj.write("T1HLVL="+str(T1HLVL)+"\n")
			sobj.write("T1LLVL="+str(T1LLVL)+"\n")
			sobj.write("T2HLVL="+str(T2HLVL)+"\n")
			sobj.write("T2LLVL="+str(T2LLVL)+"\n")
			sobj.write("LVLVL="+str(LVLVL)+"\n")
			sobj.write("MAXT1="+str(MAXT1)+"\n")
			sobj.write("MAXT2="+str(MAXT2)+"\n")
			sobj.write("MINT1="+str(MINT1)+"\n")
			sobj.write("MINT2="+str(MINT2)+"\n")
			sobj.write("T1MAC="+sensortotankattachmentdict['T1MAC']+"\n")
			sobj.write("T2MAC="+sensortotankattachmentdict['T2MAC']+"\n")
			sobj.write("MONCURR="+str(MONCURR)+"\n")
			sobj.close()
		if (operation=="savemacaddr"):
			#Save current settings including current mac addr
			sobj = open(mylocationdir+"sensorsettings", 'w')
			sobj.write("SOFTMODE="+str(SOFTMODE)+"\n")
			sobj.write("HVLVL="+str(HVLVL)+"\n")
			sobj.write("HMCURR="+str(HMCURR)+"\n")
			sobj.write("T1HLVL="+str(T1HLVL)+"\n")
			sobj.write("T1LLVL="+str(T1LLVL)+"\n")
			sobj.write("T2HLVL="+str(T2HLVL)+"\n")
			sobj.write("T2LLVL="+str(T2LLVL)+"\n")
			sobj.write("LVLVL="+str(LVLVL)+"\n")
			sobj.write("MAXT1="+str(MAXT1)+"\n")
			sobj.write("MAXT2="+str(MAXT2)+"\n")
			sobj.write("MINT1="+str(MINT1)+"\n")
			sobj.write("MINT2="+str(MINT2)+"\n")
			sobj.write("T1MAC="+sensortotankattachmentdict['T1MAC']+"\n")
			sobj.write("T2MAC="+sensortotankattachmentdict['T2MAC']+"\n")
			sobj.write("MONCURR="+str(MONCURR)+"\n")
			sobj.close()
		if (operation=="save"):
			if not ((settings=="null") or (settings=="")):
				mylist=settings.split(",")
				del mylist[-1] #delete the last comma
				settingsdict={}
				for eachsetting in mylist: #settings get passed on as HVLVL=350,LVLVL=170,T1HLVL=95,T1LLVL=75,MAXT1=590.0,MINT1=199.0,T2HLVL=95,T2LLVL=75,MAXT2=590.0,MINT2=199.0,HMCURR=7 - this split converts it in to a list
					settingsdict[eachsetting.split("=")[0]]=eachsetting.split("=")[1]
				#Set the values for current instance of the script
				HVLVL=float(settingsdict['HVLVL'])
				HMCURR=float(settingsdict['HMCURR'])
				T1HLVL=float(settingsdict['T1HLVL'])
				T1LLVL=float(settingsdict['T1LLVL'])
				T2HLVL=float(settingsdict['T2HLVL'])
				T2LLVL=float(settingsdict['T2LLVL'])
				LVLVL=float(settingsdict['LVLVL'])
				MAXT1=float(settingsdict['MAXT1'])
				MAXT2=float(settingsdict['MAXT2'])
				MINT1=float(settingsdict['MINT1'])
				MINT2=float(settingsdict['MINT2'])
				MONCURR=float(settingsdict['MONCURR'])
			#Save the values for next run
			sobj = open(mylocationdir+"sensorsettings", 'w')
			sobj.write("SOFTMODE="+str(SOFTMODE)+"\n")
			sobj.write("HVLVL="+str(HVLVL)+"\n")
			sobj.write("HMCURR="+str(HMCURR)+"\n")
			sobj.write("T1HLVL="+str(T1HLVL)+"\n")
			sobj.write("T1LLVL="+str(T1LLVL)+"\n")
			sobj.write("T2HLVL="+str(T2HLVL)+"\n")
			sobj.write("T2LLVL="+str(T2LLVL)+"\n")
			sobj.write("LVLVL="+str(LVLVL)+"\n")
			sobj.write("MAXT1="+str(MAXT1)+"\n")
			sobj.write("MAXT2="+str(MAXT2)+"\n")
			sobj.write("MINT1="+str(MINT1)+"\n")
			sobj.write("MINT2="+str(MINT2)+"\n")
			sobj.write("T1MAC="+sensortotankattachmentdict['T1MAC']+"\n")
			sobj.write("T2MAC="+sensortotankattachmentdict['T2MAC']+"\n")
			sobj.write("MONCURR="+str(MONCURR)+"\n")
			sobj.close()
		if (operation=="load"):
			settingsdict={}
			try:
				with open(mylocationdir+"sensorsettings", 'r') as sobj:
					for settingsdata in sobj:
						settingsdict[settingsdata.split("=")[0]]=settingsdata.split("=")[1].rstrip('\n')
				sobj.close()
			except IndexError:
				pass
			#load the values from the file for current instance of the script
			SOFTMODE=str(settingsdict['SOFTMODE'])
			HVLVL=float(settingsdict['HVLVL'])
			HMCURR=float(settingsdict['HMCURR'])
			T1HLVL=float(settingsdict['T1HLVL'])
			T1LLVL=float(settingsdict['T1LLVL'])
			T2HLVL=float(settingsdict['T2HLVL'])
			T2LLVL=float(settingsdict['T2LLVL'])
			LVLVL=float(settingsdict['LVLVL'])
			MAXT1=float(settingsdict['MAXT1'])
			MAXT2=float(settingsdict['MAXT2'])
			MINT1=float(settingsdict['MINT1'])
			MINT2=float(settingsdict['MINT2'])
			sensortotankattachmentdict['T1MAC']=settingsdict['T1MAC']
			sensortotankattachmentdict['T2MAC']=settingsdict['T2MAC']
			MONCURR=float(settingsdict['MONCURR'])
		if (operation=="revert"):
			settingsdict={}
			try:
				with open(mylocationdir+"sensorsettingsbakup", 'r') as sobj:
					for settingsdata in sobj:
						settingsdict[settingsdata.split("=")[0]]=settingsdata.split("=")[1].rstrip('\n')
				sobj.close()
			except IndexError:
				pass
			#load the values from the file for current instance of the script
			HVLVL=float(settingsdict['HVLVL'])
			HMCURR=float(settingsdict['HMCURR'])
			T1HLVL=float(settingsdict['T1HLVL'])
			T1LLVL=float(settingsdict['T1LLVL'])
			T2HLVL=float(settingsdict['T2HLVL'])
			T2LLVL=float(settingsdict['T2LLVL'])
			LVLVL=float(settingsdict['LVLVL'])
			MAXT1=float(settingsdict['MAXT1'])
			MAXT2=float(settingsdict['MAXT2'])
			MINT1=float(settingsdict['MINT1'])
			MINT2=float(settingsdict['MINT2'])
			sensortotankattachmentdict['T1MAC']=settingsdict['T1MAC']
			sensortotankattachmentdict['T2MAC']=settingsdict['T2MAC']
			MONCURR=float(settingsdict['MONCURR'])
	except Exception as e:
		error_handler(settingshandler.__name__,str(e))
		pass
def handlemacaddresschange(data):
	#MACADDR#T1MAC=5ccf7f1754d1
	try:
		global sensortotankattachmentdict
		sensortotankattachmentdict[data.split("=")[0]]=data.split("=")[1]
		settingshandler("null","savemacaddr")
	except Exception as e:
		error_handler(handlemacaddresschange.__name__,str(e))
		pass
##This handles the "Manual mode"
def manualmodehandler(data):
	#Client sends data as either MANUAL#MANUALMODESHUTOFF=false,TANK1MANUALHIGHLEVEL=87,TANK2MANUALHIGHLEVEL=100
	# or MANUAL-REQUEST
	global MOTOR
	global MANUALMODESHUTOFF
	global TANK1MANUALHIGHLEVEL
	global TANK2MANUALHIGHLEVEL
	if (data=="REQUEST"):
		#2/19/18 -If the Motor is off - disable the manualmodeshutoff - to prevent starting the motor if tank levels are higher than set levels
		if (MOTOR=="OFF"):
			MANUALMODESHUTOFF="false"
		pass
	else:
		datalist=data.split(",")
		MANUALMODESHUTOFF=datalist[0].split("=")[1]
		TANK1MANUALHIGHLEVEL=float(datalist[1].split("=")[1])
		TANK2MANUALHIGHLEVEL=float(datalist[2].split("=")[1])
###MAIN WEBSOCKET HANDLER - handle received messages, connecting and disconnecting clients
class SimpleChat(WebSocket):
	def handleMessage(self):
		global SOFTMODE
		global HVLVL
		global HMCURR
		global T1HLVL
		global T1LLVL
		global T2HLVL
		global T2LLVL
		global LVLVL
		global MAXT1
		global MAXT2
		global MINT1
		global MINT2
		try:
			#if client != self:
			#client.sendMessage(self.address[0] + u' - ' + self.data)
			if (self.data.split("#")[0]=="COMMAND"): #commands sent to server as COMMAND#MOTOR=ON
				commandQ.append(self.data.split("#")[1])
			elif (self.data.split("#")[0]=="STOREDDATA"): #Send last hour data on request as opposed to at the time of initial connection - improves UX. Send request from client as STOREDDATA-3600 for last 3600 seconds data 
				#self.sendMessage(u'StoredData#'+str(plotcharts("./sensordata",(time.time()-int(self.data.split("#")[1])),time.time(),"P","p")))
				try:
					print "Working on it"
					#self.sendMessage(u'StoredData-'+str(plotcharts("./sensordata",(time.time()-360),time.time(),"b")))
					#self.sendMessage(u'StoredData-'+str(plotcharts(self.data.split("#")[1],float(self.data.split("#")[2]),float(self.data.split("#")[3]),str(self.data.split("#")[4]))))
				except Exception as e:
					error_handler("STOREDDATA",str(e))
					pass
			elif (self.data.split("#")[0]=="SOFTMODE"): #changing software mode SOFTMODE
				SOFTMODE=self.data.split("#")[1]
				for client in clients:
					client.sendMessage(u'STATUS#SOFTMODE='+SOFTMODE) # Send change in mode to all connected clients
				activity_handler("SOFTMODE="+SOFTMODE)
			elif (self.data.split("#")[0]=="REQSETTINGS"): #Requesting software configurable settings 
				self.sendMessage(u'raw#ADC='+str(raw_Sensor_1_Reading)+u','+str(raw_Sensor_2_Reading))
				self.sendMessage(u'SETTINGS#HVLVL='+str(HVLVL)+u',LVLVL='+str(LVLVL)+u',T1HLVL='+str(T1HLVL)+u',T1LLVL='+str(T1LLVL)+u',MAXT1='+str(MAXT1)+u',MINT1='+str(MINT1)+u',T2HLVL='+str(T2HLVL)+u',T2LLVL='+str(T2LLVL)+u',MAXT2='+str(MAXT2)+u',MINT2='+str(MINT2)+u',HMCURR='+str(HMCURR)+u',MONCURR='+str(MONCURR)) # Send change in mode to all connected clients
			elif (self.data.split("#")[0]=="REQCAL"):
				self.sendMessage(u'raw#RMS='+str(raw_RMS_Current_ADC)+u','+str(raw_RMS_Voltage_ADC))
				self.sendMessage(u'CAL#offsetI='+str(offsetI)+u',offsetV='+str(offsetV)+u',Vcal='+str(VoltCalibrate)+u',Vadc='+str(VadcValue)+u',Ccal='+str(CurrentCalibrate)+u',Cadc='+str(CadcValue)) # Send change in mode to all connected clients
			elif (self.data.split("#")[0]=="SAVE"): #sending software configurable settings 
				settingshandler("null","bakup")
				settingshandler(self.data.split("#")[1],"save")
				self.sendMessage(u'SETTINGS#SAVED')
			elif (self.data.split("#")[0]=="SAVECAL"): #sending software configurable settings 
				calibrationhandler("null","bakup")
				calibrationhandler(self.data.split("#")[1],"save")
				self.sendMessage(u'CAL#SAVED')
			elif (self.data.split("#")[0]=="REVERTSETTINGS"): #sending software configurable settings 
				settingshandler("null","revert")
				self.sendMessage(u'SETTINGS#HVLVL='+str(HVLVL)+u',LVLVL='+str(LVLVL)+u',T1HLVL='+str(T1HLVL)+u',T1LLVL='+str(T1LLVL)+u',MAXT1='+str(MAXT1)+u',MINT1='+str(MINT1)+u',T2HLVL='+str(T2HLVL)+u',T2LLVL='+str(T2LLVL)+u',MAXT2='+str(MAXT2)+u',MINT2='+str(MINT2)+u',HMCURR='+str(HMCURR)+u',MONCURR='+str(MONCURR)) # Send change in mode to all connected clients
			elif (self.data.split("#")[0]=="REVERTCAL"): #sending software configurable settings 
				calibrationhandler("null","revert")
				#offsetI=512,offsetV=529,Vcal=110,Vadc=140,Ccal=0.21,Cadc-17.5
				self.sendMessage(u'CAL#offsetI='+str(offsetI)+u',offsetV='+str(offsetV)+u',Vcal='+str(VoltCalibrate)+u',Vadc='+str(VadcValue)+u',Ccal='+str(CurrentCalibrate)+u',Cadc='+str(CadcValue)) # Send change in mode to all connected clients
			elif (self.data.split("#")[0]=="MANUAL"): #sending software configurable settings 
				manualmodehandler(self.data.split("#")[1])
				for client in clients:
					client.sendMessage(u'MANUAL#MANUALMODESHUTOFF='+MANUALMODESHUTOFF+u',TANK1MANUALHIGHLEVEL='+str(TANK1MANUALHIGHLEVEL)+u',TANK2MANUALHIGHLEVEL='+str(TANK2MANUALHIGHLEVEL))
			elif (self.data.split("#")[0]=="RAWADC"):# Request for RawADC as RAWADC-VOLTAGE or RAWADC-CURRENT
				self.sendMessage(u'RawADC#'+str(send_raw_adc(str(self.data.split("#")[1]))))
			elif (self.data.split("#")[0]=="MACADDR"):# Sending updated mac addresses
				handlemacaddresschange(self.data.split("#")[1])
				for client in clients:
					client.sendMessage(u'TANKtoMACADDR#T1MAC='+sensortotankattachmentdict['T1MAC']+'|T2MAC='+sensortotankattachmentdict['T2MAC'])
			else:
				for client in clients:
					if client != self:
						client.sendMessage(self.address[0] + u' # ' + self.data)
		except Exception as e:
			error_handler(handleMessage.__name__,str(e))
			pass
	def handleConnected(self):
		global SOFTMODE
		global MODE
		global ACPOWER
		global MOTOR
		global TANK	
		try:
			clients.append(self)
			self.sendMessage(u'STATUS#ACPOWER='+ACPOWER+u',MOTOR='+MOTOR+u',TANK='+TANK+u',MODE='+MODE+u',SOFTMODE='+SOFTMODE)
			self.sendMessage(u'SensorData#'+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR1TIME))+u'|T1MAC|'+str(TANK1LEVEL)+u'|'+str(battery1level)+u'|'+str(TANK1temp))
			self.sendMessage(u'SensorData#'+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR2TIME))+u'|T2MAC|'+str(TANK2LEVEL)+u'|'+str(battery2level)+u'|'+str(TANK2temp))
			self.sendMessage(u'TANKtoMACADDR#T1MAC='+sensortotankattachmentdict['T1MAC']+'|T2MAC='+sensortotankattachmentdict['T2MAC'])
			'''
			if not (IsSENSOR1UP):
				sendchangedstatus("SENSORDOWN=1,SENSORTIME="+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR1TIME)))
			if not (IsSENSOR2UP):
				sendchangedstatus("SENSORDOWN=2,SENSORTIME="+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR2TIME)))
			'''
		except Exception as e:
			error_handler(handleConnected.__name__,str(e))
			pass
	def handleClose(self):
		try:
			clients.remove(self)
		except ValueError:
			pass
#This will reassess the given param (voltage, current, sensorvalue, sensorstatus etc after the timeout period and take appropriate action
def watchdoghandler(param,timeout):
	global watchdog_flag
	if (param=="voltage"):
		#Watchdog triggered by Voltage
		while watchdog_flag:
			#First sleep for timeout period
			time.sleep(timeout)
			#then reassess the Voltage
			if ((ACVOLTAGE>HVLVL) or (ACVOLTAGE<LVLVL)):
				#SInce Voltage watchdog will only be triggerred if Motor is on and now after timeout period - Voltage is still High or low - let's turn off the motor
				commandQ.append("MOTOR=OFF")
				#If SOFTMODE is auto then switch it to manual - Let the client do it - on receiving the error message
				#Send an error message to 
				sendchangedstatus("ERROR=MOTORNOTSTABLEVOLTAGE")
				error_handler(watchdoghandler.__name__,"MOTORNOTSTABLEVOLTAGE")
				watchdog_flag=False
			else:
				#Voltage is stable so leave the motor running and clear the watchdog flag
				watchdog_flag=False
	if (param=="current"):
		#watchdog triggerred by current
		while watchdog_flag:
			#First sleep for timeout period
			time.sleep(timeout)
			#Then reassess the current draw
			if (MOTORCURRENT>HMCURR):
				commandQ.append("MOTOR=OFF")
				sendchangedstatus("ERROR=MOTORHIGHCURRENT")
				error_handler(watchdoghandler.__name__,"MOTORHIGHCURRENT")
				watchdog_flag=False
			else:
				#Current draw is stable so leave the motor running and clear the watchdog flag
				watchdog_flag=False
	if (param=="NoWaterDraw"):
		#watchdog triggerred by NoWaterDraw - motor on but tanks not filling up
		while watchdog_flag:
			#First sleep for timeout period (wait for one sensor value)
			time.sleep(timeout)
			#Then reassess the tank levels
			if not ((TANK1LEVEL>myTANK1AVERAGELEVEL) or (TANK2LEVEL>myTANK2AVERAGELEVEL)):
				commandQ.append("MOTOR=OFF")
				sendchangedstatus("ERROR=NOWATERDRAW")
				error_handler(watchdoghandler.__name__,"NOWATERDRAW")
				watchdog_flag=False
			else:
				#Tanks are actually filling up, clear the watchdog flag.
				watchdog_flag=False

### This will save the errors in a log file - this file can be requested by the client and will open on the client as a pop up
def error_handler(calling_function_name,data):
	try:
		mystring="Error"
		if hasattr (data,'message'):
			mystring=data.message
		if hasattr (data,'value'):
			mystring=mystring+data.value
		print calling_function_name
		print mystring
		print data
		fobj = open(mylocationdir+"errorlog"+time.strftime("%Y-%m-%d",time.localtime()), 'a+')
		fobj.write(time.strftime("%Y-%m-%d %H:%M:%S",time.localtime()))
		fobj.write(",")
		fobj.write(calling_function_name)
		fobj.write(",")
		fobj.write(data)
		fobj.write(",")
		fobj.write(mystring)
		fobj.write('\n')
		fobj.write('**')
		fobj.close()
	except Exception as e:
		print(e)
### This will save activity (motor-on/off for now) in the errorlog (will create an activity log file in future).
### log the software mode and hardware mode. Cannot motor on from "human" hardware mode but can motor off and hence logging
def activity_handler(activity_name):
	try:
		fobj = open(mylocationdir+"errorlog"+time.strftime("%Y-%m-%d",time.localtime()), 'a+')
		fobj.write(time.strftime("%Y-%m-%d %H:%M:%S",time.localtime()))
		fobj.write(",")
		fobj.write(SOFTMODE)
		fobj.write(",")
		fobj.write(activity_name)
		fobj.write(",")
		fobj.write(MODE)
		fobj.write('\n')
		fobj.write('**')
		fobj.close()
	except Exception as e:
		error_handler(activity_handler.__name__, str(e))



# Circular buffer implementation
class CircularBuffer:
	def __init__(self, size):
		self.size = size
		self.buffer = [0] * size
		self.index = 0

	def add(self, value):
		self.buffer[self.index] = value
		self.index = (self.index + 1) % self.size

	def get_data(self):
		return self.buffer

# Initialize buffers and RMS variables for each channel
BUFFER_SIZE = 50
adc_channel_0 = CircularBuffer(BUFFER_SIZE)
adc_channel_1 = CircularBuffer(BUFFER_SIZE)
rms_channel_0 = 0
rms_channel_1 = 0


def calculate_checksum(data):
	"""Calculate the checksum for the given data."""
	return sum(map(ord, data)) & 0xFFFF


def is_duplicate_packet(packet_id):
	"""
	Check if a packet ID is a duplicate.
	If not a duplicate, add it to the tracker.
	"""
	if packet_id in PACKET_ID_TRACKER:
		return True
	PACKET_ID_TRACKER.append(packet_id)
	return False


def validate_data(data):
	"""
	Validate the received data based on checksum and other criteria.
	"""
	# Print raw packet in hex
	#print_packet_hex(data)
	if len(data) < HEADER_LENGTH + FOOTER_LENGTH:
		return False, "Invalid data length."

	# Extract header components
	mac_addr = data[:MAC_ADDRESS_LENGTH]
	packet_id = struct.unpack('<H', data[MAC_ADDRESS_LENGTH:MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH])[0]
	total_packets, sequence, payload_length = struct.unpack('BBB', data[MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH:HEADER_LENGTH])
	payload_length = int(payload_length)
	# Define the position where the checksum starts (header + payload length byte)
	checksum_position = HEADER_LENGTH + payload_length

	# Validate checksum
	payload = data[HEADER_LENGTH:checksum_position]  # Payload is between the header and checksum
	checksum_received = struct.unpack('<H', data[checksum_position:checksum_position + FOOTER_LENGTH])[0]
	checksum_calculated = calculate_checksum(data[:checksum_position])  # Include header + payload for checksum calculation
	if checksum_received != checksum_calculated:
		return False, "Checksum mismatch."

	return True, {
		"mac_addr": mac_addr,
		"packet_id": packet_id,
		"total_packets": total_packets,
		"sequence": sequence,
		"payload_length": payload_length,
		"payload": payload,
	}


def process_data_esp32(packets):
	"""
	Process ESP32 data packets, handling segmented and non-segmented data.
	"""
	for packet in packets:
		packet_id = packet["packet_id"]
		total_packets = packet["total_packets"]
		sequence = packet["sequence"]
		payload = packet["payload"]
		
		# Calculate the range of expected packet IDs
		expected_packet_ids = set(packet_id + i for i in range(-sequence + 1, total_packets - sequence + 1))
		
		# Unique key for grouping packets
		key = tuple(sorted(expected_packet_ids))

		if total_packets == 1:
			# Non-segmented data
			data_type = payload[0]  # First byte is the data type
			call_handler(data_type, payload)
		else:
			# Segmented data
			if key not in pending_segments:
				pending_segments[key] = {
					"total_packets": total_packets,
					"received": {},
					"start_time": time.time(),
				}

			# Add packet to the received dictionary
			pending_segments[key]["received"][sequence] = payload

			# Check if all packets are received
			if len(pending_segments[key]["received"]) == total_packets:
				# Reassemble packets in sequence order
				reassembled_payload = b''.join(
					pending_segments[key]["received"][i] for i in range(1, total_packets + 1)
				)

				# Determine data type from the first byte of the first packet
				data_type = reassembled_payload[0]
				call_handler(data_type, reassembled_payload)

				# Remove entry from pending_segments
				del pending_segments[key]
			else:
				# Handle timeout for incomplete segments
				current_time = time.time()
				if current_time - pending_segments[key]["start_time"] > 5.0:  # 5-second timeout
					# Concatenate received packets
					partial_payload = b''.join(
						pending_segments[key]["received"].get(i, b'') for i in range(1, total_packets + 1)
					)
					print("Timeout for segmented data: Missing packets for key {}. Proceeding with partial data.".format(key))

					# Determine data type from partial data
					data_type = partial_payload[0] if partial_payload else None
					if data_type:
						call_handler(data_type, partial_payload)

					# Remove entry from pending_segments
					del pending_segments[key]

def call_handler(data_type, payload):
	"""
	Calls the appropriate handler based on the data type.
	"""
	if data_type == DATA_TYPE_HEARTBEAT:
		process_esp32_heartbeat(payload)
	elif data_type in [DATA_TYPE_ADC0, DATA_TYPE_ADC1]:
		process_esp32_adc_data(payload)
	else:
		process_esp32_unknown_data_type(payload)
def process_esp32_heartbeat(payload):
	global raw_RMS_Voltage_ADC
	global raw_RMS_Current_ADC
	global ACVOLTAGE
	global MOTORCURRENT
	global sampleVArray
	global sampleIArray
	global VoltCalibrate
	global VadcValue
	global CurrentCalibrate
	global CadcValue
	
	#print("Heartbeat from ESP32")
	
	# Remove the first byte (data type)
	payload = payload[1:]

	# Extract ADC channel 0 and channel 1 data
	adc_data_0 = payload[:100]  # First 100 bytes
	adc_data_1 = payload[100:200]  # Next 100 bytes
	padding = payload[200:]  # Remaining bytes (padding)

	# Convert raw ADC data into 16-bit integers
	sampleVArray = [struct.unpack('<H', adc_data_0[i:i + 2])[0] for i in range(0, len(adc_data_0), 2)]
	sampleIArray = [struct.unpack('<H', adc_data_1[i:i + 2])[0] for i in range(0, len(adc_data_1), 2)]
	# Update circular buffers
	for sample in sampleVArray:
		adc_channel_0.add(sample)
	for sample in sampleIArray:
		adc_channel_1.add(sample)

	# Calculate RMS for each channel
	raw_RMS_Voltage_ADC = sqrt(sum(x**2 for x in adc_channel_0.get_data()) / BUFFER_SIZE)
	raw_RMS_Current_ADC = sqrt(sum(x**2 for x in adc_channel_1.get_data()) / BUFFER_SIZE)

	# Pass padding to the status bits handler
	handlestatusbits(padding)


	
	ACVOLTAGE = round((VoltCalibrate/VadcValue)*raw_RMS_Voltage_ADC) # root of the mean of the squared values.
	MOTORCURRENT = round((CurrentCalibrate/CadcValue)*raw_RMS_Current_ADC)

	#############################################
	# BACK UP AC/MOTOR STATUS DETECTION SECTION # 
	#############################################
	#ALSO USEFUL IN TESTING THE AUTOTHREAD with MOTOR instead of dMOTOR.
	#Use a separate light switch and bulb-set the current sensor to sense this bulb current draw - manually turn on the bulb when SWSTARTPB relay is active and manually turn off the bulb when SWSTOPPB is active
	global ACPOWER
	global MOTOR
	global MONCURR
	#If Voltage >LOW VOLTAGE - AC present
	if (ACVOLTAGE>LVLVL):
		if (ACPOWER=="OFF"):
			ACPOWER="ON"
			sendchangedstatus("ACPOWER="+ACPOWER)
	else:
		if (ACPOWER=="ON"):
			ACPOWER="OFF"
			sendchangedstatus("ACPOWER="+ACPOWER)
	#If Motor drawing more than MONCURR  - MOTOR is ON
	if (MOTORCURRENT>MONCURR):
		if (MOTOR=="OFF"):
			MOTOR="ON"
			sendchangedstatus("MOTOR="+MOTOR)
	else:
		if(MOTOR=="ON"):
			MOTOR="OFF"
			sendchangedstatus("MOTOR="+MOTOR)
	try:
		for ws in clients:
			ws.sendMessage(u'POWERDATA#ACVOLTAGE='+str(ACVOLTAGE)+u',MOTORCURRENT='+str(MOTORCURRENT))
	except Exception as e:
		error_handler(process_esp32_heartbeat.__name__, str(e))
	
#############################################
	# Print or log the results
	#print("Channel 0 RMS: {:.2f}".format(rms_channel_0))
	#print("Channel 1 RMS: {:.2f}".format(rms_channel_1))
	#print("Channel 0 Data: {}".format(adc_channel_0.get_data()))
	#print("Channel 1 Data: {}".format(adc_channel_1.get_data()))

def handlestatusbits(padding):
	global esp32_status_led_state  # Use the global state variable
	# Process padding bytes
	#print("Handling status bits")

	# Toggle LED state
	if esp32_status_led_state:
		#send_msg_to_ESP32(ESP32_MAC_ADDR+LED_OFF)
		#send_msg_to_ESP32(BROADCAST_MAC_ADDR+LED_OFF)
		ESP32send("ESP32_STATUS_LED","LOW")
	else:
		ESP32send("ESP32_STATUS_LED","HIGH")
		#send_msg_to_ESP32(BROADCAST_MAC_ADDR+LED_ON)
		#send_msg_to_ESP32(ESP32_MAC_ADDR+LED_ON)
	# Update the LED state
	esp32_status_led_state = not esp32_status_led_state
	#First byte of padding has ESP32 Input GPIOs statuses.
	global STATUSMODE, STATUSTANK1, STATUSTANK2

	if not isinstance(padding, (str, bytearray)) or len(padding) < 1:
		print("Error: Invalid padding data")
		return

	input_byte = ord(padding[0])  # Extract first byte (Python 2.7 compatibility)

	# Extract bit values based on fixed sequence
	gpio_states = {}
	for idx, gpio in enumerate(INPUT_PINS_SEQUENCE):
		state = (input_byte >> idx) & 1  # Extract bit
		gpio_states[gpio] = bool(state)  # Store state in dictionary

	# Apply GPIO_INPUT_MAP to set global variables
	for var_name, mapped_gpio in GPIO_INPUT_MAP.items():
		if mapped_gpio in gpio_states:
			globals()[var_name] = gpio_states[mapped_gpio]

	# Print updated GPIO states (Python 2 compatible)
	updated_states = {key: globals()[key] for key in GPIO_INPUT_MAP}
	modeswitch()
	tankswitch()
	#print("Updated Input GPIO States:", updated_states)

def process_esp32_adc_data(payload):
	print("ADC data")
def process_esp32_unknown_data_type(payload):
	print("unknown data type")
def process_data_other_node(payload):
	"""
	Process data for other sensor nodes (different format).
	"""
	print("Processed data from other node: {}".format(payload))


def print_packet_hex(data):
	"""
	Print received packet data in hexadecimal format.
	"""
	print("Received Packet (Hex):", " ".join("{:02X}".format(ord(byte) if isinstance(byte, str) else byte) for byte in data))

def ESP32send(GPIO, STATUS):
	"""
	Convert GPIO name and status to a single-byte message and send it.
	"""
	if GPIO not in GPIO_OUTPUT_MAP:
		print("Error: Invalid GPIO name '{}'".format(GPIO))
		return

	gpio_number = GPIO_OUTPUT_MAP[GPIO]

	if not (0 <= gpio_number <= 39):  # Ensure GPIO fits within 6 bits
		print("Error: GPIO number {} out of range (0-39)".format(gpio_number))
		return

	state_bit = 1 if STATUS == "HIGH" else 0 if STATUS == "LOW" else None
	if state_bit is None:
		print("Error: Invalid status '{}', expected 'HIGH' or 'LOW'".format(STATUS))
		return

	# Encode into a single byte: (GPIO << 1) | state_bit
	encoded_byte = (gpio_number << 1) | state_bit

	# Ensure we don’t overlap with reserved commands
	if encoded_byte >= 0xA0:
		print("Error: Encoded byte {} conflicts with reserved commands!".format(encoded_byte))
		return

	# Send the encoded byte
	send_msg_to_ESP32(BROADCAST_MAC_ADDR + chr(encoded_byte))


def send_msg_to_ESP32(msg):
	if ESP01:
		try:
			ser.write(msg)
		except serial.SerialException as e:
			error_handler(send_msg_to_ESP32.__name__, str(e))
	else:
		try:
			esp_uds_socket.send(msg)
		except socket.error as e:
			error_handler(send_msg_to_ESP32.__name__, str(e))

def LoRasend(GPIO, STATUS):
	print("LoRasend called for GPIO: " + GPIO)
	if GPIO in ACTUATOR_GPIO_OUTPUT_MAP:
		actuator_number, gpio_pin = ACTUATOR_GPIO_OUTPUT_MAP[GPIO]
		print("GPIO in ACTUATOR_GPIO_OUTPUT_MAP")
		address = actuatoraddressdict.get("ACTUATOR_{}_ADDRESS".format(actuator_number))
		if not address:
			print("Unknown actuator number:", actuator_number)
			return

		cmd_value = gpio_pin * 10 + (1 if STATUS == "HIGH" else 0)
		msg = address + chr(cmd_value)

		print("Sending to LoRa:", repr(msg))
		send_msg_to_LoRaNode(msg)

#LoRasend("SWTANK1", "HIGH")  # Sends '\xAA\x51' (81 in decimal)
#LoRasend("SWTANKB", "LOW")   # Sends '\xBB\x50' (80 in decimal)

def send_msg_to_LoRaNode(msg):
	try:
		lora_uds_socket.send(msg)
		print(msg)
	except socket.error as e:
		error_handler(send_msg_to_LoRaNode.__name__, str(e))
def receive_data_from_c_program():
	global esp_uds_socket, running_flag
	data = b''
	try:
		esp_uds_socket.settimeout(1.0)  # Set timeout to allow checking running_flag
		while running_flag:
			try:
				data = esp_uds_socket.recv(2048)
				if data:
					handlepacket(data[63:]) # Raw socket packets have 63 bytes of header compared with packets from Serial
					#print_packet_hex(data)
				time.sleep(0.1)
			except socket.timeout:
				pass  # Ignore timeout, just loop again to check running_flag
	except Exception as e:
		error_handler(receive_data_from_c_program.__name__, str(e))
	finally:
		try:
			esp_uds_socket.close()
		except NameError:
			pass
		print("Exiting receive_data_from_c_program")

def receive_data_from_c_plus_program():
	global lora_uds_socket, running_flag
	data = b''
	try:
		lora_uds_socket.settimeout(1.0)  # Set timeout to allow checking running_flag
		while running_flag:
			try:
				data = lora_uds_socket.recv(2048)
				if data:
					handlepacket(data)
					#print_packet_hex(data)
				time.sleep(0.1)
			except socket.timeout:
				pass  # Ignore timeout, just loop again to check running_flag
	except Exception as e:
		error_handler(receive_data_from_c_plus_program.__name__, str(e))
	finally:
		try:
			lora_uds_socket.close()
		except NameError:
			pass
		print("Exiting receive_data_from_c_plus_program")



def handlepacket(packet):
	"""
	Handles the processing of a received packet.
	Processes ESP32 packets and hands over non-ESP32 packets for further handling.
	"""
	global STATUSTANK1, STATUSTANK2
	if len(packet) == 2:
		first_byte = packet[0]
		if first_byte in actuatoraddressdict.values():
			print("Packet matches an actuator address: 0x{:02X}".format(ord(first_byte)))

			# Identify which actuator sent the packet
			actuator_name = None
			actuator_status_LED =None
			for key, value in actuatoraddressdict.items():
				if value == first_byte:
					actuator_name = key.replace("_ADDRESS", "")  # Extract actuator name
					break

			if actuator_name:
				# Toggle the corresponding STATUS LED
				actuator_status_led_state[actuator_name] = not actuator_status_led_state[actuator_name]

				# Send updated LED status via LoRa
				led_status = "HIGH" if actuator_status_led_state[actuator_name] else "LOW"
				actuator_status_LED=actuator_name.replace("ACTUATOR_","LED")
				LoRasend("STATUS_{}".format(actuator_status_LED), led_status)

				print("Toggled STATUS LED of {} to {}".format(actuator_name, led_status))

			# Process second byte as one's complement of status bits
			raw_status = ord(packet[1])
			decoded_status = ~raw_status & 0xFF  # Convert back from one's complement

			for status_name, (actuator_id, bit_pos) in ACTUATOR_STATUS_MAP.items():
				if actuatoraddressdict.get("ACTUATOR_{}_ADDRESS".format(actuator_id)) == first_byte:
					is_active = bool(decoded_status & (1 << bit_pos))
					print("{} = {}".format(status_name, is_active))
					tankswitch()
		else:
			print("Packet does not match any known actuator address.")

	# Ensure the packet has at least 6 bytes for MAC address check
	elif len(packet) < MAC_ADDRESS_LENGTH:
		print("Incomplete packet received. Skipping.")
		return  # Exit function for incomplete packet
		
	else:
		# Extract the MAC address
		mac_addr = packet[:MAC_ADDRESS_LENGTH]

		if mac_addr == ESP32_MAC_ADDR:
			#print("ESP32 packet detected. MAC: {}".format(mac_addr.encode("hex").upper()))

			# Extract Packet ID
			try:
				packet_id = struct.unpack('<H', packet[MAC_ADDRESS_LENGTH:MAC_ADDRESS_LENGTH + PACKET_ID_LENGTH])[0]

				# Check for duplicate Packet ID
				if is_duplicate_packet(packet_id):
					print("Duplicate packet detected. Packet ID: {}".format(packet_id))
					return  # Exit function for duplicate packet

				# Validate the data
				is_valid, result = validate_data(packet)

				if not is_valid:
					print("Invalid ESP32 data: {}".format(result))
					return  # Exit function for invalid data

				# Process valid ESP32 packet
				process_data_esp32([result])

			except struct.error as e:
				error_handler(handlepacket.__name__, str(e))
		elif len(packet) == SENSOR_PACKET_LENGTH:
			sensor_data_handler(packet)
		else:
			# Hand over non-ESP32 data
			#print("Non-ESP32 MAC detected: {}".format(mac_addr.encode("hex").upper()))
			#process_data_other_node(packet)
			print_packet_hex(packet)
			print("Random node in the area transmitting data on same frequency")
			return #Exit function for random packet

def receive_data_from_serial():
	"""Main function to handle serial communication."""
	global running_flag, ser
	buffer = b''  

	try:
		with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
			print("Listening on {} at {} baud rate.".format(SERIAL_PORT, BAUD_RATE))

			while running_flag:
				# Read incoming data
				if ser.in_waiting > 0:
					buffer += ser.read(ser.in_waiting)

					# Split buffer into packets based on delimiter
					while PACKET_DELIMITER in buffer:
						#print("Packet received via Serial")
						packet, buffer = buffer.split(PACKET_DELIMITER, 1)
						handlepacket(packet)
					time.sleep(0.1)  # Small delay to prevent busy-waiting

	except serial.SerialException as e:
		error_handler(receive_data_from_serial.__name__, str(e))
	finally:
		try:
			ser.close()
		except NameError:
			pass
		print("Exiting receive_data_from_serial")



######################################
#LCD screen Defining functions
######################################
# Define some device parameters
#I2C_ADDR  = 0x27 # I2C device address # Jan 2022 - device marked 2 - address = 0x3F . i2cdetect -l , i2cdetect -y 1
I2C_ADDR  = 0x3F
LCD_WIDTH = 16   # Maximum characters per line

# Define some device constants
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command

LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line

LCD_BACKLIGHT  = 0x08  # On
#LCD_BACKLIGHT = 0x00  # Off

ENABLE = 0b00000100 # Enable bit

# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface
#bus = smbus.SMBus(0)  # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1

def lcd_init():
	# Initialise display
	lcd_byte(0x33,LCD_CMD) # 110011 Initialise
	lcd_byte(0x32,LCD_CMD) # 110010 Initialise
	lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
	lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off 
	lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
	lcd_byte(0x01,LCD_CMD) # 000001 Clear display
	time.sleep(E_DELAY)

def lcd_byte(bits, mode):
	# Send byte to data pins
	# bits = the data
	# mode = 1 for data
	#        0 for command

	bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
	bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT

	# High bits
	bus.write_byte(I2C_ADDR, bits_high)
	lcd_toggle_enable(bits_high)

	# Low bits
	bus.write_byte(I2C_ADDR, bits_low)
	lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
	# Toggle enable
	time.sleep(E_DELAY)
	bus.write_byte(I2C_ADDR, (bits | ENABLE))
	time.sleep(E_PULSE)
	bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
	time.sleep(E_DELAY)

def lcd_string(message,line):
	# Send string to display

	message = message.ljust(LCD_WIDTH," ")

	lcd_byte(line, LCD_CMD)

	for i in range(LCD_WIDTH):
		lcd_byte(ord(message[i]),LCD_CHR)
##########################################


#This is used to gracefully exit in incase of SIGINT/SIGTERM (ctrl+c)
#All infinite loops - whether in the thread or with in a function being called from a thread must constantly check for running_flag
class GracefulKiller:
	kill_now = False
	def __init__(self):
		signal.signal(signal.SIGINT, self.exit_gracefully)
		signal.signal(signal.SIGTERM, self.exit_gracefully)

	def exit_gracefully(self, signum, frame):
		print("Received signal, exiting gracefully...")
		global running_flag
		running_flag = False
		self.kill_now = True

#Thread # 1
def LoRaReceiverthread(): 
	try:
		print("Python LoRaReceiverthread called")
		try:
			print("Python LORA_UDS_PATH Unix socket opened successfully")
			receive_data_from_c_plus_program()
		except socket.error as e:
			error_handler(LoRaReceiverthread.__name__,str(e))
	except Exception as e:
		error_handler(LoRaReceiverthread.__name__,str(e))
		pass
	finally:
		print("Exiting LoRaReceiverthread")

#Thread # 2
#was HTTP Server
def esp32handlerthread():
	try:
		print("Python esp32handlerthread called")
		if ESP01:
			try:
				ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
				print("Receiving data from Serial port")
				receive_data_from_serial()
			except serial.SerialException as e:
				error_handler(esp32handlerthread.__name__,str(e))
		else:
			try:
				print("Python ESP_UDS_PATH unix socket opened successfully")
				receive_data_from_c_program()
			except socket.error as e:
				error_handler(esp32handlerthread.__name__,str(e))
	except Exception as e:
		error_handler(esp32handlerthread.__name__,str(e))
		pass
	finally:
		print("Exiting esp32handlerthread")
#Thread # 3
def websocketservarthread():
	"""Python WebSocket server thread."""
	global running_flag
	print("Python websocket-server thread started")

	try:
		server = SimpleWebSocketServer('', WS_PORT, SimpleChat)
		websocketservarthread.server = server  # Store reference for external shutdown
		
		while running_flag:  # Run while the flag is True
			#server.serveonce()  # Process one request at a time BUT not available on currently installed version of this lib
			server.serveforever() # This is an infinite loop of it's own and it wouldn't respect running_flag
			time.sleep(0.1)  # Prevent high CPU usage
			
	except Exception as e:
		error_handler(websocketservarthread.__name__, str(e))

	finally:
		try:
			server.close()  # Properly close WebSocket server
		except:
			pass
		print("Exiting websocketservarthread")


#Thread # 4
def analogreadthread(): 
	print("Python analogreadthread called")
	try:
		while running_flag:
			#analogread(40,4000) # Range - V 106-123 I - 0.23 - 0.27
			#myanalogread(100) # 100 ms gives approximately 6 fullwaves for 60Hz powersupply
			try:
				for ws in clients:
					ws.sendMessage(u'POWERDATA#ACVOLTAGE='+str(ACVOLTAGE)+u',MOTORCURRENT='+str(MOTORCURRENT))
			except Exception as e:
				error_handler(analogreadthread.__name__, str(e))
			time.sleep(1)  # Ensure sleep isn't blocking the exit process
	except KeyboardInterrupt:
		pass
	finally:
		print("Exiting analogreadthread")
#Thread # 5
def commandthread():
	try:
		print("Python commandthread called")
		while running_flag:
			#if there is a command in the Q - pop it and send it to commandhandler
			if (len(commandQ)>0):
				commandhandler(commandQ.pop(0)) #commandQ.pop() serves commands from the last to first, commandQ.pop(0) serves from first to last
			time.sleep(1)
	except KeyboardInterrupt:
		pass
	finally:
		print("Exiting commandthread")
	
#Thread # 6
def autothread():
	try:
		print("Python autothread called")
		global TANK1FILLINGSTARTTIME
		global TANK2FILLINGSTARTTIME
		global TANK1TIMEDFULL
		global TANK2TIMESFULL
		global HASMOTORBEENONTODAY
		global MANUALMODESHUTOFF # Only global variable being modified
		#Currently using dMOTOR for debugging - change to make it work on the real system.
		#To do - due to sensor unreliability - may be wait for 2 consecutive readings before triggering start/stop
		while running_flag:
			#First check is AC POWER is present
			if (ACPOWER=="ON"):
				# This is the hardware MODE switch - set to human - no auto shut off, no auto tank switch
				# No auto start. Can still "remotely" turn on or turn off the motor
				# Won't be able to switch tanks
				if (MODE=="COMPUTER"):
					#If SMART MODE is ON - renamed it as SOFTMODE = Auto
					if (SOFTMODE=="Auto"): # Separate from computer (instead of using AND logic) in order to be able to keep some basic functions
						#separate, if needed. Such as - keep turning off when Tanks are full separate from Turning on when empty.
						#Is MOTOR On?
						if (MOTOR=="ON"):
							#Turn off motor if voltage too high or too low # 2-19-18 - being handled by watchdog thread
							#if ((ACVOLTAGE>HVLVL) or (ACVOLTAGE<LVLVL)):
							#	commandQ.append("dMOTOR=OFF")
								#Turn off motor if current draw is too high
								# At the start of the motor - current draw is going to be high
								#If autothread sees that high current- it will send a motor off command
								#So this might create a futile loop
								# Why not just do the current check when Motor is starting and not in the autothread
								# Then - there will no "Monitoring" of current draw
							#if (MOTORCURRENT>HMCURR):
								#commandQ.append("dMOTOR=OFF")
							#Check if both SENSORS are up (TODO : What if one sensor is up and the other one is down??)
							if ((IsSENSOR1UP) and (IsSENSOR2UP)):
								#If both Tanks > 95% - turn off the motor
								if (((TANK1LEVEL>T1HLVL) and (TANK2LEVEL>T2HLVL)) or ((TANK1LEVEL>98) or (TANK2LEVEL>98))):
									commandQ.append("MOTOR=OFF")
									HASMOTORBEENONTODAY = True
									#What if both tanks are full and we want to just get water downstairs?
									# set to manual mode and do it..
									#If tank 1  >  95% but tank 2 < 95%, Switch to tank 2
								elif (TANK =="Tank 1"):
									if(TANK1LEVEL>T1HLVL):
										commandQ.append("TANK=Tank 2")
								#If tank 1 >95% and Tank 2 < 75% then switch to tank2 AND vice versa
								elif (TANK == "Tank 2"):
									if(TANK2LEVEL>T2HLVL):
										commandQ.append("TANK=Tank 1")
							#SENSORS are DOWN but MODE=AUTO and MOTOR=ON  
							else:
								#Either one or both of the sensors are down
								#check how long the motor has been on
								if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT*2):
									commandQ.append("MOTOR=OFF")
									HASMOTORBEENONTODAY = True
								else:
									if (TANK =="Tank 1"):
										if IsSENSOR1UP:
											if(TANK1LEVEL>T1HLVL):
												TANK1TIMEDFULL=True
												if TANK2TIMEDFULL:
													HASMOTORBEENONTODAY = True
													commandQ.append("MOTOR=OFF")
												else:
													commandQ.append("TANK=Tank 2")
										else:
											if ((time.time()-TANK1FILLINGSTARTTIME)>MOTORONTIMELIMIT):
												TANK1TIMEDFULL=True
												if TANK2TIMEDFULL:
													HASMOTORBEENONTODAY = True
													commandQ.append("MOTOR=OFF")
												else:
													commandQ.append("TANK=Tank 2")
									elif (TANK =="Tank 2"):
										if IsSENSOR2UP:
											if(TANK2LEVEL>T2HLVL):
												TANK2TIMEDFULL=True
												if TANK1TIMEDFULL:
													HASMOTORBEENONTODAY = True
													commandQ.append("MOTOR=OFF")
												else:
													commandQ.append("TANK=Tank 1")
										else:
											if ((time.time()-TANK2FILLINGSTARTTIME)>MOTORONTIMELIMIT):
												TANK2TIMEDFULL=True
												if TANK1TIMEDFULL:
													HASMOTORBEENONTODAY = True
													commandQ.append("MOTOR=OFF")
												else:
													commandQ.append("TANK=Tank 1")
						#Motor is not ON
						else: 
							TODAY = datetime.datetime.today()
							#Sensors are UP?
							if ((IsSENSOR1UP) and (IsSENSOR2UP)):
								#If it is summer (April to August) - regardless of tank levels - turn on the motor at 2 am else follow level based 
								if SUMMER:
									if (1<TODAY.hour<3) and not HASMOTORBEENONTODAY:
										#Check which tank needs water and then start the motor.
										if ((myTANK1AVERAGELEVEL<T1HLVL) or (myTANK2AVERAGELEVEL<T2HLVL)) and not ((TANK1LEVEL>98) or (TANK2LEVEL>98)):
											if(myTANK2AVERAGELEVEL<myTANK1AVERAGELEVEL):
												commandQ.append("TANK=Tank 2")
											else:
												commandQ.append("TANK=Tank 1")
											commandQ.append("MOTOR=ON")
								else:
									#Not SUMMER - WHEN either Tank level goes below set low level - Switch to that Tank - send motor on command
									if (myTANK1AVERAGELEVEL<T1LLVL):
										commandQ.append("TANK=Tank 1")
										commandQ.append("MOTOR=ON")
									elif(myTANK2AVERAGELEVEL<T2LLVL):
										commandQ.append("TANK=Tank 2")
										commandQ.append("MOTOR=ON")
							else:
								#Either one or both of the sensors are down
								#So doesn't matter if it is Summer or not. Check if one of the sensors is up or both are down
								if (1<TODAY.hour<3) and not HASMOTORBEENONTODAY:
									if IsSENSOR1UP:
										if (myTANK1AVERAGELEVEL<T1HLVL):
											commandQ.append("TANK=Tank 1")
										else:
											commandQ.append("TANK=Tank 2")
									elif IsSENSOR2UP:
										if (myTANK2AVERAGELEVEL<T2HLVL):
											commandQ.append("TANK=Tank 2")
										else:
											commandQ.append("TANK=Tank 1")
									#Neither sensors are up so doesn't matter which tank is the switch on
									#Start the motor at 2 am and Motor has not been on.
									commandQ.append("MOTOR=ON")
					#Software mode or SMART MODE is set to Manual
					if (SOFTMODE=="Manual"):
						#Does the user want motor to be shutoff when tanks are full?
						if(MANUALMODESHUTOFF=="true"):
							if(MOTOR=="ON"):
								if ((IsSENSOR1UP) and (IsSENSOR2UP)):
									#If either Tank > set level - turn off the motor
									#Can write code to auto switch the tanks but since 
									#in manual Mode - let the user do it
									#Whichever tank is filling - when it reaches the set level
									#motor will shut off
									#2/19/18 - let's switch the tanks and when both are full - shut off the Motor
									if (TANK=="Tank 1"):
										if (TANK1LEVEL>TANK1MANUALHIGHLEVEL): 
											if (TANK2LEVEL>TANK2MANUALHIGHLEVEL):
												commandQ.append("MOTOR=OFF")
												MANUALMODESHUTOFF="false"
											else:
												commandQ.append("TANK=Tank 2")
									elif (TANK=="Tank 2"):
										if (TANK2LEVEL>TANK2MANUALHIGHLEVEL):
											if (TANK1LEVEL>TANK1MANUALHIGHLEVEL):
												commandQ.append("MOTOR=OFF")
												MANUALMODESHUTOFF="false"
											else:
												commandQ.append("TANK=Tank 1")
			time.sleep(15) # Evaluate @ twice the frequency of sensor update - in this case every 15 seconds
	except KeyboardInterrupt:
		pass
	finally:
		print("Exiting autothread")
	
def autothread2023():
	#NOT in USE#
	#Sensor #2 destroyed by hanuman jee. Running Auto on Sensor 1 and timed filling (10 minutes each tank) only.
	try:
		global HASMOTORBEENONTODAY
		global MANUALMODESHUTOFF # Only global variable being modified
		#Currently using dMOTOR for debugging - change to make it work on the real system.
		#To do - due to sensor unreliability - may be wait for 2 consecutive readings before triggering start/stop
		while running_flag:
			#First check is AC POWER is present
			if (ACPOWER=="ON"):
				# This is the hardware MODE switch - set to human - no auto shut off, no auto tank switch
				# No auto start. Can still "remotely" turn on or turn off the motor
				# Won't be able to switch tanks
				if (MODE=="COMPUTER"):
					#If SMART MODE is ON - renamed it as SOFTMODE = Auto
					if (SOFTMODE=="Auto"): # Separate from computer (instead of using AND logic) in order to be able to keep some basic functions
						#separate, if needed. Such as - keep turning off when Tanks are full separate from Turning on when empty.
						#Is MOTOR On?
						if (MOTOR=="ON"):
							#Check if motor has been on for longer than per tank set time limit
							if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT):
								# Since starting the motor from Tank 2 - if it is still on Tank 2 - switch to Tank 1 (unless Tank 1 is full)
								if (TANK == "Tank 2"):
									if IsSENSOR1UP:
										#If Tank 1 > high level %  turn off the motor (Weird logic because Tank1 fills up even when Tank is set to Tank 2)
										if ((TANK1LEVEL>T1HLVL)):
											commandQ.append("MOTOR=OFF")
											HASMOTORBEENONTODAY = True
										else:
											#Tank 1 isn't full yet so switch to Tank 1
											commandQ.append("TANK=Tank 1")
									else:
										#Sensor 1 is down - so switch to Tank 1 and fill up for fixed number of minutes.
										commandQ.append("TANK=Tank 1")
								else:
									#Tank is Tank 1
									if IsSENSOR1UP:
										#If Tank 1 > high level %  turn off the motor
										if ((TANK1LEVEL>T1HLVL)):
											commandQ.append("MOTOR=OFF")
											HASMOTORBEENONTODAY = True
									else:
										#Sensor 1 is down - check how long the motor has been on.
										if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT*2):
											commandQ.append("MOTOR=OFF")
											HASMOTORBEENONTODAY = True
						#Motor is not ON
						else: 
							TODAY = datetime.datetime.today()
							#Turn on Motor every night at 2 am and fill tanks for 10 min each.
							if (1<TODAY.hour<3) and not HASMOTORBEENONTODAY:
								print "Time is 2 O clock and Motor has not been on today"
								#Switch to Tank 2 (Summer 2023 - on switching to Tank2 - water goes in both Tank 1 and 2.
								commandQ.append("TANK=Tank 2")
								print "Tank 2 switched"
								commandQ.append("MOTOR=ON")
								print "Motor ON"
								
					#Software mode or SMART MODE is set to Manual
					if (SOFTMODE=="Manual"):
						#Does the user want motor to be shutoff when tanks are full?
						if(MANUALMODESHUTOFF=="true"):
							if (MOTOR=="ON"):
								#Check if motor has been on for longer than per tank set time limit
								if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT):
									# Since starting the motor from Tank 2 - if it is still on Tank 2 - switch to Tank 1 (unless Tank 1 is full)
									if (TANK == "Tank 2"):
										if IsSENSOR1UP:
											#If Tank 1 > high level %  turn off the motor
											if ((TANK1LEVEL>T1HLVL)):
												commandQ.append("MOTOR=OFF")
												HASMOTORBEENONTODAY = True
										else:
											#Sensor 1 is down - so switch to Tank 1 and fill up for fixed number of minutes.
											commandQ.append("TANK=Tank 1")
									else:
										#Tank is Tank 1
										if IsSENSOR1UP:
											#If Tank 1 > high level %  turn off the motor
											if ((TANK1LEVEL>T1HLVL)):
												commandQ.append("MOTOR=OFF")
												HASMOTORBEENONTODAY = True
										else:
											#Sensor 1 is down - check how long the motor has been on.
											if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT*2):
												commandQ.append("MOTOR=OFF")
												HASMOTORBEENONTODAY = True
			time.sleep(15) # Evaluate @ twice the frequency of sensor update - in this case every 15 seconds
	except KeyboardInterrupt:
		pass
		
def autothread2023a():
	#NOT IN USE#
	#Sensor #2 destroyed by hanuman jee. Running Auto on Sensor 1 and timed filling (10 minutes each tank) only.
	#Vineet fixed Sensor - June 2023. Both sensors up and running. Another issue detected - if tank is full - level doesn't increase - resulting in NOWATERDRAW error - and motor keeps turning on in loop
	try:
		global HASMOTORBEENONTODAY
		global MANUALMODESHUTOFF # Only global variable being modified
		#Currently using dMOTOR for debugging - change to make it work on the real system.
		#To do - due to sensor unreliability - may be wait for 2 consecutive readings before triggering start/stop
		while running_flag:
			#First check is AC POWER is present
			if (ACPOWER=="ON"):
				# This is the hardware MODE switch - set to human - no auto shut off, no auto tank switch
				# No auto start. Can still "remotely" turn on or turn off the motor
				# Won't be able to switch tanks
				if (MODE=="COMPUTER"):
					#If SMART MODE is ON - renamed it as SOFTMODE = Auto
					if (SOFTMODE=="Auto"): # Separate from computer (instead of using AND logic) in order to be able to keep some basic functions
						#separate, if needed. Such as - keep turning off when Tanks are full separate from Turning on when empty.
						#Is MOTOR On?
						if (MOTOR=="ON"):
							#Check if motor has been on for longer than per tank set time limit
							if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT):
								# Since starting the motor from Tank 2 - if it is still on Tank 2 - switch to Tank 1 (unless Tank 1 is full)
								if (TANK == "Tank 2"):
									if IsSENSOR1UP:
										#If Tank 1 > high level %  turn off the motor (Weird logic because Tank1 fills up even when Tank is set to Tank 2)
										if ((TANK1LEVEL>T1HLVL)):
											commandQ.append("MOTOR=OFF")
											HASMOTORBEENONTODAY = True
										else:
											#Tank 1 isn't full yet so switch to Tank 1
											commandQ.append("TANK=Tank 1")
									else:
										#Sensor 1 is down - so switch to Tank 1 and fill up for fixed number of minutes.
										commandQ.append("TANK=Tank 1")
								else:
									#Tank is Tank 1
									if IsSENSOR1UP:
										#If Tank 1 > high level %  turn off the motor
										if ((TANK1LEVEL>T1HLVL)):
											commandQ.append("MOTOR=OFF")
											HASMOTORBEENONTODAY = True
									else:
										#Sensor 1 is down - check how long the motor has been on.
										if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT*2):
											commandQ.append("MOTOR=OFF")
											HASMOTORBEENONTODAY = True
						#Motor is not ON
						else: 
							TODAY = datetime.datetime.today()
							#Turn on Motor every night at 2 am and fill tanks for 10 min each.
							if (1<TODAY.hour<3) and not HASMOTORBEENONTODAY:
								#Switch to Tank 2 (Summer 2023 - on switching to Tank2 - water goes in both Tank 1 and 2.
								#Vineet fixed the valve (End switch was not working. Problem solved. Water doesnt go to both tanks anymore.
								commandQ.append("TANK=Tank 2")
								print "Tank 2 switched"
								commandQ.append("MOTOR=ON")
								print "Motor ON"
								
					#Software mode or SMART MODE is set to Manual
					if (SOFTMODE=="Manual"):
						#Does the user want motor to be shutoff when tanks are full?
						if(MANUALMODESHUTOFF=="true"):
							if (MOTOR=="ON"):
								#Check if motor has been on for longer than per tank set time limit
								if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT):
									# Since starting the motor from Tank 2 - if it is still on Tank 2 - switch to Tank 1 (unless Tank 1 is full)
									if (TANK == "Tank 2"):
										if IsSENSOR1UP:
											#If Tank 1 > high level %  turn off the motor
											if ((TANK1LEVEL>T1HLVL)):
												commandQ.append("MOTOR=OFF")
												HASMOTORBEENONTODAY = True
										else:
											#Sensor 1 is down - so switch to Tank 1 and fill up for fixed number of minutes.
											commandQ.append("TANK=Tank 1")
									else:
										#Tank is Tank 1
										if IsSENSOR1UP:
											#If Tank 1 > high level %  turn off the motor
											if ((TANK1LEVEL>T1HLVL)):
												commandQ.append("MOTOR=OFF")
												HASMOTORBEENONTODAY = True
										else:
											#Sensor 1 is down - check how long the motor has been on.
											if ((time.time()-MOTORONTIMESTAMP)>MOTORONTIMELIMIT*2):
												commandQ.append("MOTOR=OFF")
												HASMOTORBEENONTODAY = True
			time.sleep(15) # Evaluate @ twice the frequency of sensor update - in this case every 15 seconds
	except KeyboardInterrupt:
		pass
#Thread # 7
def watchdogthread():
	try:
		print("Python watchdogthread called")
		global IsSENSOR1UP
		global IsSENSOR2UP
		global watchdog_flag
		# If in COMPUTER mode - Keep an eye on Motor Current, Supply Voltage and status of the sensors. Trigger a watchdog sequence with a timeout if something is not right
		while running_flag:
			if(MODE=="COMPUTER"):
				# when Motor is on - look at motor current draw and motor voltage
				if(MOTOR=="ON"):
					if ((ACVOLTAGE>HVLVL) or (ACVOLTAGE<LVLVL)):
						#Voltage too high/too low - set the watchdog flag and let the watchdog reassess the voltage after timeout (in seconds) period
						#If voltage still low or high - turn off the motor, switch the SOFTMODE to Manual if it was in Auto
						#Log an error with timestamp
						if not watchdog_flag:
							watchdog_flag=True
							watchdoghandler("voltage",10)
					if (MOTORCURRENT>HMCURR):
						#Motor drawing too much current??
						if not watchdog_flag:
							watchdog_flag=True
							watchdoghandler("current",5)
					if (TANK == "Tank 1"):
						if ((time.time()-TANK1FILLINGSTARTTIME)>WATERDRAWTIMELIMIT):
						#MOTOR has been ON for more than a minute - check if either tank levels have changed. If not - turn off motor
							if not (TANK1LEVEL>myTANK1AVERAGELEVEL):
							#MOTOR ON but tanks not filling up. No WATER DRAW - send it to the watchdog and wait for one more sensor value (40 seconds)
							##### PROBLEM SCENARIO IF TANK IS FULL### THE LEVEL WILL NOT?CAN NOT RISE### 
								if not watchdog_flag:
									watchdog_flag=True
									watchdoghandler("NoWaterDraw",40)
					if (TANK == "Tank 2"):
						if ((time.time()-TANK2FILLINGSTARTTIME)>WATERDRAWTIMELIMIT):
						#MOTOR has been ON for more than a minute - check if either tank levels have changed. If not - turn off motor
							if not (TANK2LEVEL>myTANK2AVERAGELEVEL):
							#MOTOR ON but tanks not filling up. No WATER DRAW - send it to the watchdog and wait for one more sensor value (40 seconds)
							##### PROBLEM SCENARIO IF TANK IS FULL### THE LEVEL WILL NOT?CAN NOT RISE### 
								if not watchdog_flag:
									watchdog_flag=True
									watchdoghandler("NoWaterDraw",40)
					
			#Irrespective of Motor on/off or Mode - monitor the status and accuracy of sensors 
			# First - check if the sensors have n't sent data in last SENSORTIMEOUT seconds
			# If either sensor is down - notify clients, log into errorlog
			if (time.time()-SENSOR1TIME>SENSORTIMEOUT): #Sensor 1 timed out
				if (IsSENSOR1UP):
					IsSENSOR1UP=False
					#sendchangedstatus("SENSORDOWN=1,SENSORTIME="+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR1TIME)))
					activity_handler("Watchdogthread - SENSORDOWN=1")
			else: #Sensor 1 not timed out
				if not (IsSENSOR1UP):
					IsSENSOR1UP=True
					sendchangedstatus("SENSORUP=1")
					activity_handler("Watchdogthread- SENSORUP=1")
			if (time.time()-SENSOR2TIME>SENSORTIMEOUT): #Sensor 2 timed out
				if (IsSENSOR2UP):
					IsSENSOR2UP=False
					#sendchangedstatus("SENSORDOWN=2,SENSORTIME="+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR2TIME)))
					activity_handler("Watchdogthread- SENSORDOWN=2")
			else: #Sensor 2 not timed out
				if not (IsSENSOR2UP):
					IsSENSOR2UP=True
					sendchangedstatus("SENSORUP=2")
					activity_handler("Watchdogthread- SENSORUP=2")
			#Then - let's look at the last two sensor readings - if difference is > SENSORACCURACY - mark that sensor as Down so if the autothread is evaluating - will see it as down and act accrordingly. 
			#appears challenging to implement - will need synchronization of autothread with sensor times.
			#Watch for increase in that tank's water level (if water being used at the same time - level might not rise).
			time.sleep(1)
	except KeyboardInterrupt:
		pass
	finally:
		print("Exiting watchdogthread")

#Thread # 8 
def lcdtickerthread():
	"""Thread for updating the LCD screen at fixed intervals."""
	global running_flag
	print("Python lcdticker thread started")
	#initialize the LCD screen
	try:
		print("call for lcd_init")
		lcd_init() # March 2022 - Remote - Getting I/O error after a reboot so disabling the LCD altogether
	except Exception as e:
		error_handler(lcdtickerthread.__name__, str(e))
		print("Error initializing LCD screen, exiting LCD Ticker Thread")
		return
	while running_flag:
		time.sleep(1)
		try:
			#print("Python lcdticker called")
			lcd_string("MODE = " + MODE, LCD_LINE_1)
			lcd_string("AC POWER = " + ACPOWER, LCD_LINE_2)
			time.sleep(LCD_REFRESH_INTERVAL)

			lcd_string("MOTOR = " + MOTOR, LCD_LINE_1)
			lcd_string("TANK = " + TANK, LCD_LINE_2)
			time.sleep(LCD_REFRESH_INTERVAL)

			if MOTOR == "ON":
				lcd_string("Motor V = " + str(ACVOLTAGE), LCD_LINE_1)
				lcd_string("Current(A) = " + str(MOTORCURRENT), LCD_LINE_2)
				time.sleep(LCD_REFRESH_INTERVAL)

			if IsSENSOR1UP:
				lcd_string("TANK 1 Level = ", LCD_LINE_1)
				lcd_string(str(TANK1LEVEL) + "%", LCD_LINE_2)
				time.sleep(LCD_REFRESH_INTERVAL)
			else:
				lcd_string("SENSOR 1 DOWN", LCD_LINE_1)
				lcd_string(time.strftime('%d-%b %H:%M:%S', time.localtime(SENSOR1TIME)), LCD_LINE_2)
				time.sleep(LCD_REFRESH_INTERVAL)
			
			if IsSENSOR2UP:
				lcd_string("TANK 2 Level = ", LCD_LINE_1)
				lcd_string(str(TANK2LEVEL) + "%", LCD_LINE_2)
				time.sleep(LCD_REFRESH_INTERVAL)
			else:
				lcd_string("SENSOR 2 DOWN", LCD_LINE_1)
				lcd_string(time.strftime('%d-%b %H:%M:%S', time.localtime(SENSOR2TIME)), LCD_LINE_2)
				time.sleep(LCD_REFRESH_INTERVAL)
			# **Check and display network status**
			if is_connected():
				network_status = get_ip_address("wlan1")
			else:
				network_status = "Disconnected"

			lcd_string("NETWORK STATUS", LCD_LINE_1)
			lcd_string(network_status, LCD_LINE_2)
			time.sleep(LCD_REFRESH_INTERVAL)
		except Exception as e:
			error_handler(lcdtickerthread.__name__, str(e))
			print("Exiting due to Error in lcdticker thread:", e)
			return
		finally:
			#Wipe the LCD screen
			lcd_byte(0x01, LCD_CMD)
			#print("Exiting lcdtickerthread")


	

if __name__ == '__main__':
	try:#Load the settings
		print "Starting sensor-server script \n"
		print "Date Time is :" 
		print datetime.datetime.today()
		settingshandler("null","load")
		calibrationhandler("null","load")
		if not ESP01:
			mysendrecv_proc=subprocess.Popen(["./mysendrecv.o", "mon0"])
			print("Python mysendrecv C called")
			time.sleep(2)
			esp_uds_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
			esp_uds_socket.connect(ESP_UDS_PATH)
		lora_proc=subprocess.Popen(["./LoRaDuplexWithRecvCallBack"])
		print("Python LoRaDuplexWithRecvCallBack C++ called")
		time.sleep(2)
		lora_uds_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
		lora_uds_socket.connect(LORA_UDS_PATH)

		init_status()
		t1=Thread(target=LoRaReceiverthread) 
		t2=Thread(target=esp32handlerthread)
		t3=Thread(target=websocketservarthread)
		#t4=Thread(target=analogreadthread)
		t5=Thread(target=commandthread)
		t6=Thread(target=autothread)
		t7=Thread(target=watchdogthread)
		t8=Thread(target=lcdtickerthread)
		#Daemon - means threads will exit when the main thread exits
		t1.daemon=True
		t2.daemon=True
		t3.daemon=True
		#t4.daemon=True
		t5.daemon=True
		t6.daemon=True
		t7.daemon=True
		t8.daemon=True
		#Start the threads 
		t1.start()
		t2.start()
		t3.start()
		#t4.start()
		t5.start()
		t6.start()
		t7.start()
		t8.start()
		#For killing gracefully
		killer = GracefulKiller()
		while True:
			if killer.kill_now:
				#save the settings
				settingshandler("null","save")
				calibrationhandler("null","save")
				# Send termination signal to subprocesses
				if not ESP01:
					mysendrecv_proc.terminate()
				lora_proc.terminate()
				#Join means wait for the threads to exit
				t1.join() #LoRaReceiverthread - has runningflag in its function
				t2.join() #esp32handlerthread - has runningflag in its function
				#t4.join() #analogreadthread - has runningflag
				t5.join() #commandthread - has runningflag
				t6.join()	#autothread - has runningflag
				t7.join() # watchdogthread - has runningflag
				t8.join()#lcdtickerthread - has runningflag
				if not ESP01:
					mysendrecv_proc.wait()
				lora_proc.wait()
				websocketservarthread.server.close()
				t3.join() #websocketservarthread - has runningflag but has serveforever() infinite loop which runs independent.hence calling server.close
				print "exited gracefully"
				break
			time.sleep(1) # without it CPU runs as fast as possible.
	except KeyboardInterrupt:
		#Signal is caught by graceful killer class
		pass

