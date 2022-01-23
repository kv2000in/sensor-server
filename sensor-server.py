#!/usr/bin/python
# -*- coding: utf-8 -*-
#

#Changelog 1/21/2018 - added battery voltage display
#changelog 2/10/2018 - cleanup - removed unused functions
#Changelog 2/18/2018 - only one websocket client connection at a time
#Changelog 2/19/2018 - Adding "backup" way of detecting AC (via voltmeter) and Motor (via Current meter) statuses - in case the Chinese AC detection board doesn't work.
#						Back up methods are commented out for now.
#						Added init_status - to consolidate all the global status variables
#						Adjusted manual mode shut off routine - now fill both tanks and then turn off the motor when टंकी भर जाये ता माेटर बँद? is checked
#						Added a watchdog thread - which basically monitors for High Low Voltage and High Motor current, Sensors staying active etc.
#						Added a logfile to log all the errors with timestamps- switch from Auto to Manual SOFTMODE - in case of an error
#						Modified the HTTP handler - to serve anyfile in the directory, also serving files as objects instead of reading and writing line by line
#						To do - Watch for increase in tank's water level (if water being used at the same time - level might not rise).
#ToDO - 3/5/18 - display sensor values as soon as user webpage is displayed, figure out why webserver got stuck, fix sensor up/down UI, fix manual mode shutdown
#Changelog 3/6/18 - added - send battery level when user connects, changed all relative paths to absolute paths, change the LCD ticker logic to fix simultaneous "sensor 1 down and Tank1 level" messages
#Changelog #3-7-18 - Using Apache2 HTTP server instead of python server. Allowed 2 simultaneous websocket connections
#Changelog 3-8-18 - Changed MONCURR=0.9 - . 0.1 pe when Power is switched on - falsely detects motor on
#Changelog 3-9-18 - Vineet reported inability to connect to websocket server, apache sever running. Multiple restarts attempted. error msg on client side was "connection refused" suggesting that python script might not be running. 
					#We tried changing the NUMofWebSocketClientsAllowed=2 to 10 but this was not the problem since the problem related to this leads to an error like "websocket closed before handshake was complete". Meaning - if the 
					#NUMofWebSocketClientsAllowed is set to 2 and 3rd client is trying to connect - that one is closed. Also - on the client side "onblur - websocket.close()" routine was added to prevent "hanging' of the server script
					#if the client goes to sleep with active websocket connection. This didn't seem to be the problem either. "Connection refused" suggests that python script was exiting after starting - due to some error OR the script was
					#not starting at all. Start of the script was tested multiple times - even after installation of apache. It tunrs out that this problem was fixed when sensor2 came online. 
					#
					#{{{{ 
					#Tank 2 overflow - sensor destroyed. To prevent - 
					#even with the bigger overflow hole - i think water will still reach the sensors and sensor 1 is also going to be destroyed sooner or later. Will need to come up with a better way to install these sensors so that these are 
					#never in contact with water - no matter what. 
					#Overflow happened when in auto mode - tank2 kept on filling and motor wasn't turned off.  Motor was started because one/both of the tanks were below low set level. It switched from tank1 to tank2 automatically when tank1 
					#reached its high set level then it turned off when tank2 was above its LOW set level. Restarted again when Tank2 went below the low set level (from 40 to 39) and then it didn't stop.
					#If there were an error encountered -auto mode should have been switched to manual and if motor was on - it should have been switched off (will have to double check this). If someone switches from Auto mode to Manual mode 
					#while the motor is on - motor will not stop.
					#}}}}
					#
					#SO - when the sensor 2 was down - python script failed or atleast the websocket server was not running. As of 3-10-18 - Both sensors were online, both tanks had bigger and may be more than one over flow holes , script was running.
					#WILL NEED TO IMPLEMENT TIMER BASED SAFETY (takes x mins to fill tank 1 from level y to level z - turn off the motor after that amount of time has passed.
					#Check the sensor down logic.
					# Turn off the motor - in case of an error in auto mode.
					# ALSO - We are not using the optocouplers for detection of AC status and Motor running status. We are using the voltmeter and currentmeter "Backup" method since it needed less hardware to implement.
#Jan 2022 - todo: - change host ip on sensor nodes and extenders to 192.168.1.110
#LCD i2c address - see comments in lcd section i2c address 0x27 vs 0x3F
#Removed limit on websocket clients - disconnection probably due to errors rather than number of clients



import time
import os
import shutil
import datetime 
import RPi.GPIO as GPIO
import smbus
import math
import signal
import socket
from threading import Thread
import BaseHTTPServer
from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
import binascii
import struct

mylocationdir="/home/pi/Downloads/sensor-server/"
# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

#### GLOBAL VARIABLES ######
TCP_IP=''
HTTP_PORT = 62246
WS_PORT= 8000
TCP_PORT=9999

running_flag = True #used for graceful shutdown 
watchdog_flag = False#used for watchdog

#global commandQ
commandQ=[]

# global value for plotchart
mydict ={}

#MAC Addresses of sensornodes # loads from settings file or user can input from webinterface for the first run
sensortotankattachmentdict={}
#WebSocket OPCODES
STREAM = 0x0
TEXT = 0x1
BINARY = 0x2
CLOSE = 0x8
PING = 0x9
PONG = 0xA
#WebSocket clients
clients = []
# Define GPIO Pins - these are hard wired - changing these will require respldering the appropriate pins
GPIO.setmode(GPIO.BCM)
STATUSMODE=5 # Computer vs Human (High = Human). THis is a hardware switch on the board
STATUSTANK1=20 # Low = Tank1 actuator valve in on position  #Jan 2022 - this was 20 which conflicted with STATUSAC (? wasn't a problem earlier-see comments below)
STATUSTANK2=21 # Low = Tank2 actuator valve in on position   #Jan 2022 - this was 21 which conflicted with STATUSMOTOR (? wasn't a problem earlier-see comments below)
SWSTARTPB=13 # High = start PB pressed
SWSTOPPB=19# High = stop PB pressed
SWTANK=26  # High = Tank1, Low = Tank2
#PIN # 6 connected to 4th relay switch - on Boot - Pin 6 is high and the relay is on ### 2nd board - switched this pin to GPIO 18
GPIO.setup(6,GPIO.OUT)
GPIO.output(6,GPIO.LOW)
#Set switches as GPIO.OUT and statuses as GPIO.IN
# with gpio register pulled down - it is sitting at 0V. when connected to 3.3V via a 10k external resistor (as in the opto) - small current flows through the 10k resistor - causing the voltage at collector to be 2.7 V - which is not detected as logical HIGH by GPIO Input
# Pull up will set it to 3.3V so no current will flow.
GPIO.setup(STATUSMODE, GPIO.IN, pull_up_down = GPIO.PUD_DOWN) # NO esistor between 3.3V and this pin so keep it pulled down
GPIO.setup(STATUSTANK1, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(STATUSTANK2, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.setup(SWSTARTPB, GPIO.OUT)
GPIO.output(SWSTARTPB,GPIO.LOW)
GPIO.setup(SWSTOPPB, GPIO.OUT)
GPIO.output(SWSTOPPB,GPIO.LOW)
GPIO.setup(SWTANK, GPIO.OUT)
GPIO.output(SWTANK,GPIO.LOW)
 
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

#Turn off motor if current draw is too high
HMCURR=7 # At the start of the motor - current draw is going to be high

#Added on 2/19/18 - MONCURR is only used in "backup" motor status detection in myanalogread. If motor is drawing more than this much current - motor is considered to be on.
MONCURR=19 # Jan 2022 - was 0.9 - changed it to 19 for debugging

#Set Max and initial Tank Level ADC reads
MAXT1=10.0#CM Values. how many Centimeters from HCSR04 when the tank 1 is full. 
MAXT2=10.0 # decimal point necessary so that this is treated as float when calculations are done

MINT1=199.0 # Adjust max and min for each sensor, Min level is basically offset
MINT2=199.0 # Tanklevel in % = (sensorvalue-min)/(max-min)*100

T1LLVL=75 # less than this % value - switch tank or motor on
T1HLVL=95

T2LLVL=75 # less than this % value - switch tank or motor on
T2HLVL=95

TANK1LEVEL=999 # Define Tank level variable - initial value
TANK2LEVEL=999

timebetweensensordata = 20 # Time in seconds - if the same sensor sends data with less than this much time gap from previous data - ignore that data
SENSORTIMEOUT=120 # Time in seconds - for which if the sensor hasn't posted data - it will be considered to be "Down"

#SENSOR1TIME=time.time()-SENSORTIMEOUT # Define initial update time for the sensors#
#SENSOR2TIME=time.time()-SENSORTIMEOUT# 60 seconds prior to start of the script - so that - if sensor is down at the runtime-auto mode won't progress
#2/19/18 updates - global sensor status - start with assumption that sensors are down.
#3-6-18 - due to sensor being down - even if saved softmode is auto - it switched to manual. hence start with sensors being up.
SENSOR1TIME=time.time()-timebetweensensordata+5# OR datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
SENSOR2TIME=time.time()-timebetweensensordata+5# OR datetime.now().strftime("%Y-%m-%d %H:%M:%S")
#SO in the beginning both sensors are assumed to be down - as soon as a sensor reading is received - it will be set to True 
IsSENSOR1UP=True
IsSENSOR2UP=True
#This is used if SOFTMODE=Manual and client wants to shutoff the motor after filling the tanks
MANUALMODESHUTOFF="false"
TANK1MANUALHIGHLEVEL=100
TANK2MANUALHIGHLEVEL=100
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



#Read initial GPIO status
def init_status():
	#since we are MODIFYING global variables - need to use "global" keyword
	global MODE
	global ACPOWER
	global MOTOR
	global TANK
	if (GPIO.input(STATUSMODE)==1):
		MODE="HUMAN"
	else:
		MODE="COMPUTER"
	#1/7/18 - With optocoupler - logic is reversed - On signal, output goes to ground
	if ((GPIO.input(STATUSTANK1)==0)&(GPIO.input(STATUSTANK2)==1)):
		TANK="Tank 1"
	elif ((GPIO.input(STATUSTANK1)==1)&(GPIO.input(STATUSTANK2)==0)):
		TANK="Tank 2"
	else:
		TANK="undefined"
#Function called by change in STATUSMODE GPIO
def modeswitch(channel):
	global MODE
	if (GPIO.input(STATUSMODE)==1):
		MODE="HUMAN"
	else:
		MODE="COMPUTER"
	sendchangedstatus("MODE="+MODE)

#Function called by change in STATUSTANK1 and STATUSTANK2 GPIOs
def tankswitch(channel):
	global TANK
	#1/7/18 - With optocoupler - logic is reversed - On signal, output goes to ground
	if ((GPIO.input(STATUSTANK1)==0)&(GPIO.input(STATUSTANK2)==1)):
		TANK="Tank 1"
	elif ((GPIO.input(STATUSTANK1)==1)&(GPIO.input(STATUSTANK2)==0)):
		TANK="Tank 2"
	else:
		TANK="undefined"
	sendchangedstatus("TANK="+TANK)
#Function used to send the changed statuses to the client via Websocket
def sendchangedstatus(mymsg):
	try:
		for ws in clients:
			ws.sendMessage(u'STATUS#'+mymsg)
	except Exception as e:
		error_handler(sendchangedstatus.__name__,e)
		pass
#Adding bouncetime leads to undefined state in case of fast switching
#October 2019 - added RC circuit on hardware board to deal with signal bounce
#GPIO.add_event_detect(STATUSMODE, GPIO.BOTH, callback=modeswitch, bouncetime=30)
GPIO.add_event_detect(STATUSMODE, GPIO.BOTH, callback=modeswitch)
GPIO.add_event_detect(STATUSTANK1, GPIO.BOTH, callback=tankswitch)
GPIO.add_event_detect(STATUSTANK2, GPIO.BOTH, callback=tankswitch)

#Define the object for ADC read functions
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(0, 1,20000)) # (0,0) was being used by NRF24L01 # Decreased the max clock speed to 50KHz to get more accurate readings (Range is 10 Khz to 1.35 MHz).

## Commands are handled in the order they are received - this function is invoked by commandthread
def commandhandler(command):
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
							GPIO.output(SWSTARTPB,GPIO.HIGH) # Active low relays - GPIO HIGH turns on the relay by forward biasing the base of the NPN transistor - which brings the collector (Relay IN pin) to ground
							time.sleep(3)
							if (MOTOR=="ON"): #wait 3 seconds for Motor status to change to ON
								GPIO.output(SWSTARTPB,GPIO.LOW) # then release the STARTPB 
								#Check motor current draw and turn off motor if current > limit
								if (MOTORCURRENT>HMCURR):
									#Motor is ON but current is high - turn off the Motor
									GPIO.output(SWSTOPPB,GPIO.HIGH) #Press STOP PB
									sendchangedstatus("ERROR=MOTORHIGHCURRENT")
									error_handler(commandhandler.__name__,"MOTORHIGHCURRENT")
									time.sleep(3) # Wait for 3 seconds
									if (MOTOR=="OFF"): # If motor turned off
										GPIO.output(SWSTOPPB,GPIO.LOW) # Release STOP PB
							else:#Motor didn't start,
								GPIO.output(SWSTARTPB,GPIO.LOW) #  Release START PB and send Error
								sendchangedstatus("ERROR=MOTORSTART")
								error_handler(commandhandler.__name__,"MOTORSTART")
						else:# Doesn't meet voltage criteria - unstable or too low/too high
							sendchangedstatus("ERROR=MOTORNOTSTABLEVOLTAGE")
							error_handler(commandhandler.__name__,"MOTORNOTSTABLEVOLTAGE")
				if (command.split("=")[1]=="OFF"):
					if (MOTOR=="ON"):
						GPIO.output(SWSTOPPB,GPIO.HIGH) #Press STOP PB
						time.sleep(3) # Wait for 3 seconds
						if (MOTOR=="OFF"): # If Motor turned off
							GPIO.output(SWSTOPPB,GPIO.LOW) # Release STOP PB
						else: # Motor didn't stop
							GPIO.output(SWSTOPPB,GPIO.LOW) # Release STOP PB and send error
							sendchangedstatus("ERROR=MOTORSTOP")
							error_handler(commandhandler.__name__,"MOTORSTOP")
							#TURN OFF A MASTER POWER SWITCH TO PROTECT THE MOTOR
		if (command.split("=")[0]=="TANK"):
			if (command.split("=")[1]=="Tank 2"):
				if (TANK=="Tank 1"):
					GPIO.output(SWTANK,GPIO.LOW)
			if (command.split("=")[1]=="Tank 1"):
				if (TANK=="Tank 2"):
					GPIO.output(SWTANK,GPIO.HIGH)
					
		if (command.split("=")[0]=="dMOTOR"):
			if (ACPOWER=="ON"):# Execute motor commands only if ACPOWER is ON
				if (command.split("=")[1]=="ON"):
					if (MOTOR=="OFF"):
						#Build Safety - Stable voltage for 5 seconds(with in 20 volts), initial current draw
						initialACVOLTAGE=ACVOLTAGE
						time.sleep(5)
						finalACVOLTAGE=ACVOLTAGE
						if ((HVLVL>finalACVOLTAGE>LVLVL) and ((finalACVOLTAGE-initialACVOLTAGE)<abs(20))):
							GPIO.output(SWSTARTPB,GPIO.HIGH)
						else:# Doesn't meet voltage criteria - unstable or too low/too high
							sendchangedstatus("ERROR=MOTORNOTSTABLEVOLTAGE")
				if (command.split("=")[1]=="OFF"):
					if (MOTOR=="ON"):
						GPIO.output(SWSTARTPB,GPIO.LOW)
		'''
		if (command.split("=")[0]=="dMOTOR"):#Debugging mode - just one relay - MOTOR ON - high, Motor OFF - low
			if (command.split("=")[1]=="ON"):
				if (MOTOR=="OFF"):
					GPIO.output(SWSTARTPB,GPIO.HIGH)
			if (command.split("=")[1]=="OFF"):
				if (MOTOR=="ON"):
					GPIO.output(SWSTARTPB,GPIO.LOW)
		'''
	except Exception as e:
		error_handler(commandhandler.__name__,e)
		pass
###SENSOR VALUES ARE RECEIVED, SAVED and UPDATED by THIS FUNCTION
def worker_sensorthread(client_socket):
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
		if (client_socket):
			request = client_socket.recv(512)
			#Data arrives as multiples of 16 bytes - 6 bytes of MAC Addr, 2 bytes Short Int/'h' Distance, 2 bytes temp and 2 bytes battery voltage, 4 bytes zeroes padding
			#initially had it as 16 instead of 512. If data comes directly from sensors via tcp - always 16 bytes. if it comes via ESP-NOW nodes - it could be multiples of 16 due to re-transmissions.
			client_socket.close()
			currtime=time.time()
			strcurrtime = time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(currtime))
			macaddrofdata = binascii.hexlify(request[0:6])
			if (macaddrofdata==sensortotankattachmentdict['T1MAC']):
				#check if we just received data from the same macaddr
				if((currtime-SENSOR1TIME)>timebetweensensordata):
					SENSOR1TIME=currtime
					#round(float((y-x)/(z-x)*100),1) - float to 1 decimal
					TANK1LEVEL=round((((MINT1-float(struct.unpack('h',request[6:8])[0]))/(MINT1-MAXT1))*100),1)
					#TANK1LEVEL=sensorvalue.split(':')[1]+"%"# For sending RAW CM OUTPUT TO CLIENT FOR CALIBRATION
					raw_Sensor_1_Reading=struct.unpack('h',request[6:8])[0]
					battery1level=struct.unpack('h',request[10:12])[0]
					TANK1temp = struct.unpack('h',request[8:10])[0]
					message2=u'SensorData#'+strcurrtime+u'|T1MAC|'+str(TANK1LEVEL)+u'|'+str(battery1level)+u'|'+str(TANK1temp)
					message1=strcurrtime+'|T1MAC|'+str(TANK1LEVEL)+'|'+str(battery1level)+'|'+str(TANK1temp)
					for ws in clients:
						ws.sendMessage(message2)
					savesensordatatofile(message1)
			elif (macaddrofdata==sensortotankattachmentdict['T2MAC']):
				if((currtime-SENSOR2TIME)>timebetweensensordata):
					SENSOR2TIME=currtime
					#round(float((y-x)/(z-x)*100),1) - float to 1 decimal
					TANK2LEVEL=round((((MINT2-float(struct.unpack('h',request[6:8])[0]))/(MINT2-MAXT2))*100),1)
					raw_Sensor_2_Reading=struct.unpack('h',request[6:8])[0]
					battery2level=struct.unpack('h',request[10:12])[0]
					TANK2temp = struct.unpack('h',request[8:10])[0]
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
								TANK1LEVEL=round((((MINT1-float(struct.unpack('h',request[6+i*16:8+i*16])[0]))/(MINT1-MAXT1))*100),1)
								raw_Sensor_1_Reading=struct.unpack('h',request[6+i*16:8+i*16])[0]
								battery1level=struct.unpack('h',request[10+i*16:12+i*16])[0]
								TANK1temp = struct.unpack('h',request[8+i*16:10+i*16])[0]
								message2=u'SensorData#'+strcurrtime+u'|T1MAC|'+str(TANK1LEVEL)+u'|'+str(battery1level)+u'|'+str(TANK1temp)
								message1=strcurrtime+'|T1MAC|'+str(TANK1LEVEL)+'|'+str(battery1level)+'|'+str(TANK1temp)
								for ws in clients:
									ws.sendMessage(message2)
								savesensordatatofile(message1)
						elif (binascii.hexlify(request[0+i*16:6+i*16])==sensortotankattachmentdict['T2MAC']):
							if((currtime-SENSOR2TIME)>timebetweensensordata):
								SENSOR2TIME=currtime
								#round(float((y-x)/(z-x)*100),1) - float to 1 decimal
								TANK2LEVEL=round((((MINT2-float(struct.unpack('h',request[6+i*16:8+i*16])[0]))/(MINT2-MAXT2))*100),1)
								raw_Sensor_2_Reading=struct.unpack('h',request[6+i*16:8+i*16])[0]
								battery2level=struct.unpack('h',request[10+i*16:12+i*16])[0]
								TANK2temp = struct.unpack('h',request[8+i*16:10+i*16])[0]
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
	except Exception as e:
		error_handler(worker_sensorthread.__name__,e)
		pass
def savesensordatatofile(formattedsensordata):
	try:
		fobj = open(mylocationdir+"sensordata"+time.strftime("%Y-%m-%d",time.localtime()), 'a+')
		fobj.write(formattedsensordata)
		fobj.write('\n')
		fobj.close()
	except Exception as e:
		error_handler(savesensordatatofile.__name__,e)
		pass
#Request for plot will come as which sensor, starttime, stoptime.
#open the file with name sensordata+startdate name, read lines, seek until time is >time from starttime
#read line by line - if sensor matches the sensor requested - push the time and value data in the list
#reached EOF but time>stoptime - open next file sensordata+startdate+1 name and start reading lines.
# uniformly date time format yyyy-mm-dd hh:mm:ss seems to work
#request comes as STOREDDATA#2021-02-26 09:47:52#2021-02-26 10:47:52#t#a#A
#return data as "t$2021-02-20 21:04:33|-1.09,2021-02-20 21:04:41|-1.08,"
def sendstoreddata(data):
	vars=[]
	mytempdataholdingdict={} #get all the data in a list and send it to javascript in chunks of "numberofdatapoints" length
	mystarttimedatetimeformat=datetime.strptime(data.split("#")[1],"%Y-%m-%d %H:%M:%S")
	myendtimedatetimeformat=datetime.strptime(data.split("#")[2],"%Y-%m-%d %H:%M:%S")
	mystartdate=mystarttimedatetimeformat.date()
	vars=data.split('#')[3:]
	for sensorid in vars:
		mytempdataholdingdict[sensorid]=""
	try:
		while (myendtimedatetimeformat.date()>=mystartdate):
			if os.path.exists(mylocationdir+"sensordata"+str(mystartdate)):
				with open(mylocationdir+"sensordata"+str(mystartdate),"r") as fobj:
					for myline in fobj:
						myline=myline.strip()
						mystoreddatetime=myline.split("|")[0]
						mystoredsensor=myline.split("|")[1]
						mystoredsensorvalue=myline.split("|")[2]
						if (datetime.strptime(mystoreddatetime,"%Y-%m-%d %H:%M:%S")>mystarttimedatetimeformat):
							for sensorid in vars:
								if (mystoredsensor==sensorid):
									mytempdataholdingdict[sensorid]+=mystoreddatetime+"|"+mystoredsensorvalue+","
			mystartdate+=timedelta(days=1)
		for sensorid in vars:
			mymessagetosend = sensorid+"#"+mytempdataholdingdict[sensorid]
			for ws in clients:
				ws.sendMessage(u'StoredData#'+mymessagetosend.strip(","))
	except Exception as e:
		error_handler(sendstoreddata.__name__,e)
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
		error_handler(send_raw_adc.__name__,e)
		pass
##Analogread is the function from EMONPI example - currently not in use
def analogread(crossings,timeouttime): #(20,2000) in emonpi example
	#Using myanalogread instead of analogread - simpler - no phase difference calculation - just instantaneous V and I measurement
	global ACVOLTAGE
	global MOTORCURRENT
	global inPinV
	global inPinI
	global offsetV
	global offsetI
	global VoltCalibrate
	global VadcValue
	global CurrentCalibrate
	global CadcValue
	ADC_COUNTS=1024
	'''
	SupplyVoltage=3300 #mcp3008 supply voltage
	VCAL=110 # Volt for highest ADC
	ICAL=1.5
	'''
	'''
	# These offset values are true if the bias voltage is mid supply rail
	#i.e. 2.5 V for a 5 V ADC and 1.65 V for a 3.3 V ADC
	# in our case for AC Volt - at current settings mid point is around ADC value of 422
	# for Current mid point is ADC=505 ### NOW I KNOW WHY NOT 512 - Voltage divider source resistance too high - ADC pin presents a "load" to the voltage divider
	#Changed the resistors to 4.7k instead of 47k and 470k.
	#VCC 5.12V gives midrail 2.56V - going through 4.68k/9.36k divider gives 1.707V = 529 read by ADC with V ref of 3.3V
	offsetV = ADC_COUNTS>>1 #Divide ADC_COUNTS by 2 - for 1024 - result is 512
	offsetI = ADC_COUNTS>>1
	
	'''
	crossCount = 0 #Used to measure number of times threshold is crossed.
	numberOfSamples = 0#This is now incremented
	'''
	//Reset accumulators
	'''
	sumV = 0
	sumI = 0
	sumP = 0
	'''
	//-------------------------------------------------------------------------------------------------------------------------
	// 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
	//-------------------------------------------------------------------------------------------------------------------------
	'''
	st=False#an indicator to exit the while loop
	starttime = time.time()*1000 # millis()-start makes sure it doesnt get stuck in the loop if there is an error.
	while(st==False):#the while loop
		startV = mcp.read_adc(inPinV)#using the voltage waveform
		'''
		#See above - since ADC isn't exactly mid rail - I am substituting with actually Midpoint ADC
		if ((startV < (ADC_COUNTS*0.55)) and (startV > (ADC_COUNTS*0.45))):
			st=True#check its within range
		'''
		if ((startV < 400) and (startV > 450)):
			st=True#check its within range
		if ((time.time()*1000-starttime)>timeouttime):
			st=True
	'''
	//-------------------------------------------------------------------------------------------------------------------------
	// 2) Main measurement loop
	//-------------------------------------------------------------------------------------------------------------------------
	'''
	starttime = time.time()*1000
	while ((crossCount < crossings)and((time.time()*1000-starttime)<timeouttime)):
		numberOfSamples+=1#Count number of times looped.
		#lastFilteredV = filteredV#Used for delay/phase compensation
		'''
		//-----------------------------------------------------------------------------
		// A) Read in raw voltage and current samples
		//-----------------------------------------------------------------------------
		'''
		sampleV = mcp.read_adc(inPinV)#Read in raw voltage signal
		sampleI = mcp.read_adc(inPinI)#Read in raw current signal
		'''
		//-----------------------------------------------------------------------------
		// B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
		//     then subtract this - signal is now centred on 0 counts.
		//-----------------------------------------------------------------------------
		'''
		offsetV = offsetV + ((sampleV-offsetV)/1024)
		filteredV = sampleV - offsetV
		offsetI = offsetI + ((sampleI-offsetI)/1024)
		filteredI = sampleI - offsetI
		'''
		//-----------------------------------------------------------------------------
		// C) Root-mean-square method voltage
		//-----------------------------------------------------------------------------
		'''
		sqV= filteredV * filteredV	#1) square voltage values
		sumV += sqV	#2) sum
		'''
		//-----------------------------------------------------------------------------
		// D) Root-mean-square method current
		//-----------------------------------------------------------------------------
		'''
		sqI = filteredI * filteredI	#1) square current values
		sumI += sqI	#2) sum
		'''
		//-----------------------------------------------------------------------------
		// E) Phase calibration
		//-----------------------------------------------------------------------------
		'''
		#phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV)
		'''
		//-----------------------------------------------------------------------------
		// F) Instantaneous power calc
		//-----------------------------------------------------------------------------
		'''
		#instP = phaseShiftedV * filteredI	#Instantaneous Power
		#sumP +=instP	#Sum
		'''
		//-----------------------------------------------------------------------------
		// G) Find the number of times the voltage has crossed the initial voltage
		//    - every 2 crosses we will have sampled 1 wavelength
		//    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
		//-----------------------------------------------------------------------------
		'''
		#lastVCross = checkVCross
		if(sampleV > startV):
			checkVCross = True
		else:
			checkVCross = False
		if(numberOfSamples==1):
			checkVCross = False
		if(checkVCross):
			crossCount+=1
	'''
	//-------------------------------------------------------------------------------------------------------------------------
	// 3) Post loop calculations
	//-------------------------------------------------------------------------------------------------------------------------
	//Calculation of the root of the mean of the voltage and current squared (rms)
	//Calibration coefficients applied.
	'''
	''' To figure out the sampling frequency
	
	timediff=time.time()*1000-starttime
	print numberOfSamples
	print timediff
	print (numberOfSamples/timediff)*1000
	'''
	'''
	#ADC of 140 ~ AC V RMS of 110 => each ADC = 110/140
	#ADC of 17.5 ~ AC I RMS of 0.21 A
	'''
	'''
	#double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
	V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS))
	'''
	#V_RATIO=110.0/140.0
	#ACVOLTAGE = V_RATIO*math.sqrt(sumV / numberOfSamples)
	ACVOLTAGE = (VoltCalibrate/VadcValue)*math.sqrt(sumV / numberOfSamples)
	'''
	#double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
	I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS))
	'''
	#I_RATIO= 0.21/17.5
	#MOTORCURRENT = I_RATIO*math.sqrt(sumI / numberOfSamples)
	MOTORCURRENT = (CurrentCalibrate/CadcValue)*math.sqrt(sumI / numberOfSamples)
	'''
	//Calculation power values
	realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
	apparentPower = Vrms * Irms;
	powerFactor=realPower / apparentPower;
	'''
	#print ACVOLTAGE
	'''//--------------------------------------------------------------------------------------
	'''
##THIS FUNCTION READs the analog ADC values (voltage and current)
def myanalogread(timeout):
	try:
		global ACVOLTAGE
		global MOTORCURRENT
		global sampleVArray
		global sampleIArray
		global VoltCalibrate
		global VadcValue
		global CurrentCalibrate
		global CadcValue
		global offsetI
		global offsetV
		global raw_RMS_Voltage_ADC
		global raw_RMS_Current_ADC
		VArray=[]
		IArray=[]
		sumV=0
		sumI=0
		numberOfSamples=0
		#V_RATIO=110.0/140.0 # ADC Value of 140 for RMS AC Voltage  of 110 V
		#I_RATIO=0.21/17.5 # ADC Value of 17.5 for RMS AC Current of 0.21 A
		#offsetV=529 #see Emon pi function above for explanation
		#offsetI=512
		starttime=time.time()*1000 #(time in milliseconds)
		#timeout= 4000
		while ((time.time()*1000-starttime)<timeout): # Collect samples for the duration specified by timeout time.
			#Bitbanging using python is slower than spidev using python
			sampleV=mcp.read_adc(inPinV)
			sampleI=mcp.read_adc(inPinI)
			filteredV = sampleV - offsetV
			filteredI = sampleI - offsetI
			sqV= filteredV * filteredV	#1) square voltage values
			sumV += sqV	#2) sum
			sqI = filteredI * filteredI	#1) square current values
			sumI += sqI	#2) sum
			numberOfSamples+=1
			VArray.append(sampleV)
			IArray.append(sampleI)
			#round(float((y-x)/(z-x)*100),1) - float to 1 decimal
		#ACVOLTAGE = round((V_RATIO*math.sqrt(sumV / numberOfSamples)),1) # root of the mean of the squared values.
		ACVOLTAGE = round(((VoltCalibrate/VadcValue)*math.sqrt(sumV / numberOfSamples)),1) # root of the mean of the squared values.
		raw_RMS_Voltage_ADC=round(math.sqrt(sumV/numberOfSamples),1)
		'''
		print numberOfSamples
		print sumV
		print sumV/numberOfSamples
		print math.sqrt(sumV / numberOfSamples)
		print V_RATIO*math.sqrt(sumV / numberOfSamples)
		print sumI
		'''
		#MOTORCURRENT = round((I_RATIO*math.sqrt(sumI / numberOfSamples)),2)
		MOTORCURRENT = round(((CurrentCalibrate/CadcValue)*math.sqrt(sumI / numberOfSamples)),2)
		raw_RMS_Current_ADC=round(math.sqrt(sumI/numberOfSamples),1)
		sampleVArray=VArray
		sampleIArray=IArray
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
		#############################################
	except Exception as e:
		error_handler(myanalogread.__name__,e)
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
		error_handler(calibrationhandler.__name__,e)
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
		error_handler(settingshandler.__name__,e)
		pass
def handlemacaddresschange(data):
	#MACADDR#T1MAC=5ccf7f1754d1
	try:
		global sensortotankattachmentdict
		sensortotankattachmentdict[data.split("=")[0]]=data.split("=")[1]
		settingshandler("null","savemacaddr")
	except Exception as e:
		error_handler(handlemacaddresschange.__name__,e)
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
					error_handler("STOREDDATA",e)
					pass
			elif (self.data.split("#")[0]=="SOFTMODE"): #changing software mode SOFTMODE
				SOFTMODE=self.data.split("#")[1]
				for client in clients:
					client.sendMessage(u'STATUS#SOFTMODE='+SOFTMODE) # Send change in mode to all connected clients
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
			error_handler(handleMessage.__name__,e)
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
			if not (IsSENSOR1UP):
				sendchangedstatus("SENSORDOWN=1,SENSORTIME="+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR1TIME)))
			if not (IsSENSOR2UP):
				sendchangedstatus("SENSORDOWN=2,SENSORTIME="+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR2TIME)))
		except Exception as e:
			error_handler(handleConnected.__name__,e)
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
		global SOFTMODE
		#In case of any error while running in "SOFTMODE=Auto" , switch to SOFTMODE=Manual and notify the connected clients
		#3-10-18 - TODO - if the motor is ON and in Auto mode - any error - will switch the mode to manual and motor will never turn off since the user doesn't know that motor is on and mode is now manual.
		if (SOFTMODE=="Auto"):
			SOFTMODE="Manual"
			sendchangedstatus("SOFTMODE="+SOFTMODE)
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




##### This updates the LCD screen##############
def lcdticker():
	lcd_string("MODE = "+MODE,LCD_LINE_1)
	lcd_string("AC POWER = "+ACPOWER,LCD_LINE_2)
	time.sleep(1.5)
	lcd_string("MOTOR = "+MOTOR,LCD_LINE_1)
	lcd_string("TANK = "+TANK,LCD_LINE_2)
	time.sleep(1.5)
	if (MOTOR=="ON"):
		lcd_string("Motor V = "+str(ACVOLTAGE),LCD_LINE_1)
		lcd_string("Current(A)= "+str(MOTORCURRENT),LCD_LINE_2)
		time.sleep(1.5)
	if (running_flag):
		if (IsSENSOR1UP):
			lcd_string("TANK 1 Level = ",LCD_LINE_1)
			lcd_string(str(TANK1LEVEL)+"%",LCD_LINE_2)
			time.sleep(1.5)
		else:
			lcd_string("SENSOR 1 DOWN",LCD_LINE_1)
			lcd_string(time.strftime('%d-%b %H:%M:%S', time.localtime(SENSOR1TIME)),LCD_LINE_2)
			time.sleep(1.5)
	if (running_flag):
		if (IsSENSOR2UP):
			lcd_string("TANK 2 Level = ",LCD_LINE_1)
			lcd_string(str(TANK2LEVEL)+"%",LCD_LINE_2)
			time.sleep(1.5)
		else:
			lcd_string("SENSOR 2 DOWN",LCD_LINE_1)
			lcd_string(time.strftime('%d-%b %H:%M:%S', time.localtime(SENSOR2TIME)),LCD_LINE_2)
			time.sleep(1.5)
#######################################################

#This is used to gracefully exit in incase of SIGINT/SIGTERM (ctrl+c)
class GracefulKiller:
	kill_now = False
	def __init__(self):
		signal.signal(signal.SIGINT, self.exit_gracefully)
		signal.signal(signal.SIGTERM, self.exit_gracefully)

	def exit_gracefully(self,signum, frame):
		self.kill_now = True
		print "SIGNAL received"
		global running_flag
		running_flag=False
#Thread # 1
def TCPserverthread(): 
	try:
		#ESP8266 sends sensor data to Raspberry Pi using TCP Server Client 
		#TCP Server for sensor
		TCPserverthread.srvr = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		#Fixes address already in use error due to socket being in "TIME_WAIT" stae. allow reusing of socket address
		TCPserverthread.srvr.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
		TCPserverthread.srvr.bind((TCP_IP, TCP_PORT))
		TCPserverthread.srvr.listen(5)
		#print 'Listening on {}:{}'.format(TCP_IP, TCP_PORT)
		while running_flag:
			client_sock, address = TCPserverthread.srvr.accept()
			#print 'Accepted connection from {}:{}'.format(address[0], address[1])
			client_handler = Thread(
				target=worker_sensorthread,
				args=(client_sock,)  # without comma you'd get a... TypeError: handle_client_connection() argument after * must be a sequence, not _socketobject
			)
			client_handler.start()
	except Exception as e:
		error_handler(TCPserverthread.__name__,e)
		pass
#Thread # 2
#was HTTP Server
#Thread # 3
def websocketservarthread(): 
	try:
		websocketservarthread.server = SimpleWebSocketServer('', WS_PORT, SimpleChat)
		websocketservarthread.server.serveforever()
		#print time.asctime(), "Websocket Server Starts - %s:%s" % (TCP_IP, WS_PORT)
	except Exception as e:
		error_handler(websocketservarthread.__name__,e.message)
		pass
#Thread # 4
def analogreadthread(): 
	try:
		while running_flag:
			#analogread(40,4000) # Range - V 106-123 I - 0.23 - 0.27
			myanalogread(100) # 100 ms gives approximately 6 fullwaves for 60Hz powersupply
			try:
				for ws in clients:
					ws.sendMessage(u'POWERDATA#ACVOLTAGE='+str(ACVOLTAGE)+u',MOTORCURRENT='+str(MOTORCURRENT))
					#ws._sendMessage(False, TEXT, message) 
			except Exception as e:
				if hasattr(e, 'message'):
					print(e.message)
				else:
					print(e)
			time.sleep(1) # 100 ms sample everysecond
	except KeyboardInterrupt:
		pass
#Thread # 5
def commandthread():
	try:
		while running_flag:
			#if there is a command in the Q - pop it and send it to commandhandler
			if (len(commandQ)>0):
				commandhandler(commandQ.pop(0)) #commandQ.pop() serves commands from the last to first, commandQ.pop(0) serves from first to last
			time.sleep(1)
	except KeyboardInterrupt:
		pass
#Thread # 6
def autothread():
	try:
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
						#		commandQ.append("dMOTOR=OFF")
							#Check if both SENSORS are up (TODO : What if one sensor is up and the other one is down??)
							if ((IsSENSOR1UP) and (IsSENSOR2UP)):
								#If both Tanks > 95% - turn off the motor
								if ((TANK1LEVEL>T1HLVL) and (TANK2LEVEL>T2HLVL)):
									commandQ.append("MOTOR=OFF")
									#What if both tanks are full and we want to just get water downstairs?
									# set to manual mode and do it..
								#If tank 1 >95% and tank 2 <95%
								elif ((TANK1LEVEL>T1HLVL) and (TANK2LEVEL<T2HLVL)):
									if(TANK2LEVEL<T2LLVL):
										commandQ.append("TANK=Tank 2")
									else:
										commandQ.append("MOTOR=OFF")
								#If tank 1 >95% and Tank 2 < 75% then switch to tank2 AND vice versa
								elif ((TANK2LEVEL>T2HLVL) and (TANK1LEVEL<T1HLVL)):
									if(TANK1LEVEL<T1LLVL):
										commandQ.append("TANK=Tank 1")
									else:
										commandQ.append("MOTOR=OFF")
							#SENSORS are DOWN but MODE=AUTO and MOTOR=ON  : Turn off the Motor
							else:
								commandQ.append("MOTOR=OFF")
						#Motor is not ON
						else: 
							#Sensors are UP?
							if ((IsSENSOR1UP) and (IsSENSOR2UP)):
								#WHEN either Tank level goes below set low level - Switch to that Tank - send motor on command
								if (TANK1LEVEL<T1LLVL):
									commandQ.append("TANK=Tank 1")
									commandQ.append("MOTOR=ON")
								elif(TANK2LEVEL<T2LLVL):
									commandQ.append("TANK=Tank 2")
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
#Thread # 7
def watchdogthread():
	try:
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
			#Irrespective of Motor on/off or Mode - monitor the status and accuracy of sensors 
			# First - check if the sensors have n't sent data in last SENSORTIMEOUT seconds
			# If either sensor is down - notify clients, log into errorlog
			if (time.time()-SENSOR1TIME>SENSORTIMEOUT): #Sensor 1 timed out
				if (IsSENSOR1UP):
					IsSENSOR1UP=False
					sendchangedstatus("SENSORDOWN=1,SENSORTIME="+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR1TIME)))
					error_handler("Watchdogthread","SENSORDOWN=1")
			else: #Sensor 1 not timed out
				if not (IsSENSOR1UP):
					IsSENSOR1UP=True
					sendchangedstatus("SENSORUP=1")
			if (time.time()-SENSOR2TIME>SENSORTIMEOUT): #Sensor 2 timed out
				if (IsSENSOR2UP):
					IsSENSOR2UP=False
					sendchangedstatus("SENSORDOWN=2,SENSORTIME="+time.strftime("%Y-%m-%d %H:%M:%S",time.localtime(SENSOR2TIME)))
					error_handler("Watchdogthread","SENSORDOWN=2")
			else: #Sensor 2 not timed out
				if not (IsSENSOR2UP):
					IsSENSOR2UP=True
					sendchangedstatus("SENSORUP=2")
			#Then - let's look at the last two sensor readings - if difference is > SENSORACCURACY - mark that sensor as Down so if the autothread is evaluating - will see it as down and act accrordingly. 
			#appears challenging to implement - will need synchronization of autothread with sensor times.
			#Watch for increase in that tank's water level (if water being used at the same time - level might not rise).
			time.sleep(1)
	except KeyboardInterrupt:
		pass

if __name__ == '__main__':
	#server_class = BaseHTTPServer.HTTPServer
	#httpd = server_class((TCP_IP, HTTP_PORT), MyHandler)
	#print time.asctime(), "Server Starts - %s:%s" % (TCP_IP, HTTP_PORT)
	try:#Load the settings
		settingshandler("null","load")
		calibrationhandler("null","load")
		#initialize the LCD screen
		lcd_init()
		#Check initial GPIO statuses - added 2/19/18
		init_status()
		t1=Thread(target=TCPserverthread)  
		t3=Thread(target=websocketservarthread)
		t4=Thread(target=analogreadthread)
		t5=Thread(target=commandthread)
		t6=Thread(target=autothread)
		t7=Thread(target=watchdogthread)
		#Daemon - means threads will exit when the main thread exits
		t1.daemon=True
		t3.daemon=True
		t4.daemon=True
		t5.daemon=True
		t6.daemon=True
		t7.daemon=True
		#Start the threads 
		t1.start()
		t3.start()
		t4.start()
		t5.start()
		t6.start()
		t7.start()
		#For killing gracefully
		killer = GracefulKiller()
		while True:
			lcdticker()
			time.sleep(1) # Makes a huge difference in CPU usage - without >95%, with <10%
			if killer.kill_now:
				#clean the GPIO
				GPIO.cleanup()
				#save the settings
				settingshandler("null","save")
				calibrationhandler("null","save")
				#Wipe the LCD screen
				#lcd_byte(0x01, LCD_CMD)
				#Join means wait for the threads to exit
				TCPserverthread.srvr.shutdown(socket.SHUT_RDWR)
				TCPserverthread.srvr.close()
				t1.join #TCPserverthread - has runningflag
				print "TCP server closed"
				websocketservarthread.server.close()
				t3.join #websocketservarthread
				print "websocket closed"
				t4.join #analogreadthread - has runningflag
				t5.join #commandthread - has runningflag
				t6.join	#autothread - has runningflag
				t7.join # watchdogthread - has runningflag
				print "exited gracefully"
				break
	except KeyboardInterrupt:
		#Signal is caught by graceful killer class
		pass


'''
From Adafruit library :
    def __init__(self, port, device, max_speed_hz=500000):
        """Initialize an SPI device using the SPIdev interface.  Port and device
        identify the device, for example the device /dev/spidev1.0 would be port
        1 and device 0.

'''
'''
#### These are not in use functions#####
def sighandler(signum, frame):
	print 'signal handler called with signal: %s ' % signum
	global running_flag
	running_flag = False
	sys.exit() # make sure you add this so the main thread exits as well.

if __name__ == '__main__':
	main(sys.argv)
	while 1:  # this will force your main thread to live until you terminate it.
		time.sleep(1) 

def main(argv=None):
	signal.signal(signal.SIGTERM, sighandler) # so we can handle kill gracefully
	signal.signal(signal.SIGINT, sighandler) # so we can handle ctrl-c
	try:
		Thread(target=main_loop, args=()).start()
	except Exception, reason:
		print reason
'''
'''
### Main function from LCD screen example###
def main():
	# Main program block

	# Initialise display
	lcd_init()

	while True:

	# Send some test
		lcd_string("         <",LCD_LINE_1)
		lcd_string("        <",LCD_LINE_2)

		time.sleep(3)
  
		# Send some more text
		lcd_string(">",LCD_LINE_1)
		lcd_string("> ",LCD_LINE_2)

		time.sleep(3)

if __name__ == '__main__':

	try:
		main()
	except KeyboardInterrupt:
		pass
	finally:
		lcd_byte(0x01, LCD_CMD)

'''

'''
#### Example code for GPIO detection####
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
GPIO.setup(24, GPIO.IN, pull_up_down = GPIO.PUD_UP)
def printFunction(channel):
print(?Button 1 pressed!?)
print(?Note how the bouncetime affects the button press?)
GPIO.add_event_detect(23, GPIO.RISING, callback=printFunction, bouncetime=300)
while True:
GPIO.wait_for_edge(24, GPIO.FALLING)
print(?Button 2 Pressed?)
GPIO.wait_for_edge(24, GPIO.RISING)
print(?Button 2 Released?)
GPIO.cleanup()
	If needed to remove function
GPIO.remove_event_detect(23)
'''

	#except KeyboardInterrupt:
		# It never reaches the thread
	#	pass
		#print "websocket received Ctrl+C"
		#websocketservarthread.server.close()
		#print time.asctime(), "Server Stops - %s:%s" % (TCP_IP, WS_PORT)
