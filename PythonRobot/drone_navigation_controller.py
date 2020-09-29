# /etc/init.d/dronekitserver.py
### BEGIN INIT INFO
# Provides:          dronekitserver.py
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Start daemon at boot time
# Description:       Enable service provided by daemon.
### END INIT INFO

from websocket_server import WebsocketServer
from Tkinter import *
import math
import time
import thread
import socket
import select
import struct
import random
import sys, glob # for listing serial ports.
import os  # to command the mp3 and wav player omxplayer*

# IRobot Controller
from dronecontroller.idrone_controller import IDroneController

##########################################################################
##########################################################################
#---------GLOBAL VARIABLES FOR ROBOT STATE
##########################################################################
##########################################################################

STATE_DISCONNECTED = -2
STATE_IDLE = -1
STATE_ROTATE_TO_ANGLE = 0
STATE_GO_TO_TARGET = 1
STATE_FREE_CONTROL = 2

g_distanceFromCenter = 0.05
g_targetAngle = 100001
g_reportReachedTarget = False
g_displayOneTimeMessage = False
g_previousTimeNow = 0
g_timeAcumToRunAction = 0
g_isRedWaypoint = True
g_stateRoombaTarget = STATE_IDLE
g_currentAngle = 0
g_currentIterations = 0
g_checkWhatColor = True
g_error_range = 5
g_accuracy_angle = 25
g_iterator_surpass = 5
g_last_direction_rotation = True
g_sign_diference_last_rotation = 0
g_targetLatitude = 0
g_targetLongitude = 0

def initilitzationSystem():
	global STATE_DISCONNECTED
	global g_distanceFromCenter
	global g_targetAngle
	global g_targetLatitude
	global g_targetLongitude
	global g_reportReachedTarget
	global g_displayOneTimeMessage
	global g_previousTimeNow
	global g_timeAcumToRunAction
	global g_isRedWaypoint
	global g_stateRoombaTarget
	global g_currentAngle
	global g_currentIterations
	global g_checkWhatColor
	global g_error_range
	global g_accuracy_angle
	g_distanceFromCenter = 0.05
	g_targetAngle = 100001
	g_reportReachedTarget = False
	g_displayOneTimeMessage = False
	g_previousTimeNow = 0
	g_timeAcumToRunAction = 0
	g_isRedWaypoint = True
	changeStateRobot(STATE_DISCONNECTED, 100001)
	g_currentAngle = 0
	g_currentIterations = 0
	g_checkWhatColor = True
	g_error_range = 15
	g_accuracy_angle = 25
	g_targetLatitude = 0
	g_targetLongitude = 0

def changeStateRobot(newState, angleGoal, latitude, longitude):
	global g_targetAngle
	global g_targetLatitude
	global g_targetLongitude
	global g_previousTimeNow
	global g_currentIterations
	global g_timeAcumToRunAction
	global g_displayOneTimeMessage
	global g_stateRoombaTarget
	global g_error_range
	global g_iterator_surpass
	global STATE_DISCONNECTED
	global STATE_IDLE
	global STATE_ROTATE_TO_ANGLE
	global STATE_GO_TO_TARGET
	global STATE_FREE_CONTROL
	g_stateRoombaTarget = newState
	g_currentIterations = 0
	g_previousTimeNow = 0
	g_timeAcumToRunAction = 0
	g_displayOneTimeMessage = True
	if g_stateRoombaTarget == STATE_IDLE:
		g_targetAngle = 100001
		print ('**CHANGE STATE: STATE_IDLE')
	if g_stateRoombaTarget == STATE_ROTATE_TO_ANGLE:
		#NAVIGATION	-	RASPBERRY
		#0+180			-	200/180
		#270+180%360	-	110/90
		#180+180%360	-	20/0
		#90+180%360		- 	300/270
		g_targetAngle = angleGoal
		g_targetLatitude = latitude
		g_targetLatitude = longitude
		output = "**CHANGE STATE: ANGLE=%d, LATITUDE=%d, LONGITUDE=%d" % (g_targetAngle, g_targetLatitude, g_targetLatitude)
		g_iterator_surpass = 7
		print (output)
	if g_stateRoombaTarget == STATE_GO_TO_TARGET:
		g_targetAngle = 100001
		print ('**CHANGE STATE: STATE_GO_TO_TARGET')
	if g_stateRoombaTarget == STATE_FREE_CONTROL:
		g_targetAngle = 100001
		print ('**CHANGE STATE: STATE_FREE_CONTROL')
	if g_stateRoombaTarget == STATE_DISCONNECTED:
		g_targetAngle = 100001
		print ('**CHANGE STATE: STATE_DISCONNECTED-------')

def calculateDeltaTime():
	global g_previousTimeNow
	global g_timeAcumToRunAction
	localdeltatime = int(round(time.time() * 1000))
	g_timeAcumToRunAction = g_timeAcumToRunAction + localdeltatime - g_previousTimeNow
	g_previousTimeNow = localdeltatime

def displayOneTimeMessage(messageDisplay):
	global g_displayOneTimeMessage
	if g_displayOneTimeMessage:
		g_displayOneTimeMessage = False
		print(messageDisplay)


##########################################################################
##########################################################################
#---------Start my Python Robot Controller Program below here!------------
#---------Change the code below
##########################################################################
##########################################################################

def new_client(client, server):
		global irobotcontroller
		print("Client connected")
		irobotcontroller.connect_vehicle("/dev/ttyAMA0", 57600)
		changeStateRobot(STATE_IDLE, 100001)
		server.send_message_to_all("Client connected")
def message_received(client, server, message):
		global irobotcontroller
		global g_distanceFromCenter
		global g_targetAngle
		global g_targetLatitude
		global g_targetLongitude
		global g_reportReachedTarget
		global g_displayOneTimeMessage
		global g_previousTimeNow
		global g_timeAcumToRunAction
		global g_isRedWaypoint
		global g_stateRoombaTarget
		global STATE_IDLE
		global STATE_ROTATE_TO_ANGLE
		global STATE_GO_TO_TARGET
		global STATE_FREE_CONTROL
		if len(message) > 200:
				message = message[:200] + ".."
		if "requestDirection" not in message and "setTarget" not in message:				
			print ('(client message: ' + message + ')')
		#if cleanButtonPressed(): driveDirect( 0, 0 )  # If CLEAN button is pressed stop robot.  
		postime = message.find("_time_")
		vend = message.find("_end")
		time_sleep = 2
		if postime > 0 and vend > 0: 
			time_sleep = float(message[postime+6:vend])
			print('time_sleep is', time_sleep, '!!!!!!!!!')
		if "setTarget" in message:
			if g_reportReachedTarget is True:
				g_reportReachedTarget = False
				g_targetAngle = 100001
				print ('')
				print ('++++REPORTING CHANGE TO NEXT GPS')
				server.send_message_to_all("targetangle_success")
			else:
				posangle = message.find("_gps_")
				vend = message.find("_end")
				if g_targetAngle > 100000 and posangle > 0 and vend > 0 and (g_stateRoombaTarget == STATE_IDLE or g_stateRoombaTarget == STATE_FREE_CONTROL):
					gpsData = message[posangle+7:vend]
					gpsItems = gpsData.split(',')
					if (len(gpsItems) == 3):
						changeStateRobot(STATE_ROTATE_TO_ANGLE, float(gpsItems[0]), decimal(gpsItems[0]), decimal(gpsItems[0]))
		if "moveForward" in message:
			print ('moveForward vehicle')
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.forward_drone( 5, 5 )			# Velocity, Duration
			time.sleep( time_sleep )						# Wait X seconds
			irobotcontroller.stop_drone()					# Full stop!			
			server.send_message_to_all("moveforward_success")
		if "moveBackward" in message:
			print ('moveBackward vehicle')
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.forward_drone( -5, 5 )			# Velocity, Duration
			time.sleep( time_sleep )						# Wait X seconds
			irobotcontroller.stop_drone()					# Full stop!			
			server.send_message_to_all("movebackward_success")
		if "turnRight" in message:
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.turn_drone( 10, 2 )		# Degrees, Duration
			time.sleep( time_sleep )					# Wait X seconds
			irobotcontroller.stop_drone()				# Full stop!			
			server.send_message_to_all("turnright_success")
		if "turnLeft" in message:
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.turn_drone( -10, 2 )		# Degrees, Duration
			time.sleep( time_sleep )					# Wait X seconds
			irobotcontroller.stop_drone()				# Full stop!			
			server.send_message_to_all("turnleft_success")
		if "requestDirection" in message:
			output_send = "requestdirection_success_%1.2f" % g_distanceFromCenter
			# print (output_send)
			server.send_message_to_all(output_send)
		if "genericAction" in message:
			irobotcontroller.beepSound()
			server.send_message_to_all("genericAction_success")
		if "stopAction" in message:
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.stop_drone()				# Full stop!
			server.send_message_to_all("stopAction_success")			
		if "closeCommunication" in message:
			print ('Close Communication ++START++')
			initilitzationSystem()
			irobotcontroller.closeRobotCommunication()
			print ('Close Communication ++END++')
		if "armAndTakeOff" in message:
			print ('armAndTakeOff ++START++')
			initilitzationSystem()
			beginAltitude = message.find("armAndTakeOff_")+14
			endAltitude = message.find("_end")
			finalAltitudeTakeOff = float(message[beginAltitude:endAltitude])
			irobotcontroller.arm_and_takeoff(finalAltitudeTakeOff)
			print ('armAndTakeOff ++END++')
		if "returnHome" in message:
			print ('returnHome ++START++')
			initilitzationSystem()
			irobotcontroller.retun_home_drone()
			print ('returnHome ++END++')
		if "disarmDrone" in message:
			print ('disarmDrone ++START++')
			initilitzationSystem()
			irobotcontroller.disarm_drone()
			time.sleep( 5 )
			irobotcontroller.close_drone()
			print ('disarmDrone ++END++')

##########################################################################
##########################################################################
#---------GPSNavigationBehavior
##########################################################################
##########################################################################

class GPSNavigationBehavior(object):
	"""Behavior to find and get close to a colored object"""
	def __init__(self, interval=1):
		# Thread
		self.interval = interval

		thread = threading.Thread(target=self.run, args=())
		thread.daemon = True                            # Daemonize thread
		thread.start()   

	def process_control(self):
		instruction = "start"
		if instruction == "start":
			self.running = True
		elif instruction == "stop":
			self.running = False
		elif instruction == "exit":
			print("Stopping")
			exit()

	def run(self):
		global irobotcontroller
		global g_distanceFromCenter
		global g_targetAngle
		global g_targetLatitude
		global g_targetLongitude
		global g_reportReachedTarget
		global g_displayOneTimeMessage
		global g_previousTimeNow
		global g_timeAcumToRunAction
		global g_isRedWaypoint
		global g_stateRoombaTarget
		global STATE_IDLE
		global STATE_ROTATE_TO_ANGLE
		global STATE_GO_TO_TARGET		
		global STATE_FREE_CONTROL
		global g_currentAngle
		global g_currentIterations
		global g_checkWhatColor

		print("Setup Complete")
		# Main loop
		self.process_control()
		# ----------------------------------------------
		if g_stateRoombaTarget == STATE_ROTATE_TO_ANGLE:
			if g_targetAngle < 100000:
				# DISPLAY ENTER STATE MESSAGE
				displayOneTimeMessage("++STATE_GO_TO_ANGLE::GPS(%d,%d)" % (g_targetLatitude,g_targetLongitude))
				# CALCULATE TIME FOR ACTION
				calculateDeltaTime()
				# ONE TIME RUN: GO TO GPS COORDINATES
				if g_timeAcumToRunAction > 500:
					irobotcontroller.goto_drone(g_targetLatitude, g_targetLongitude)
					changeStateRobot(STATE_GO_TO_TARGET, 100001)
		# ----------------------------------------------
		if g_stateRoombaTarget == STATE_GO_TO_TARGET:
			if self.running:
				# DISPLAY ENTER STATE MESSAGE
				displayOneTimeMessage("++STATE_GO_TO_TARGET::GPS(%d,%d)" % (g_targetLatitude,g_targetLongitude))
				# CALCULATE TIME FOR ACTION
				calculateDeltaTime()
				# CHECK REACHED GPS COORDINATES
				if g_timeAcumToRunAction > 1000:
					g_timeAcumToRunAction = 0
					if irobotcontroller.is_drone_moving() is False:
						g_reportReachedTarget = True
						changeStateRobot(STATE_IDLE, 100001)

initilitzationSystem()
irobotcontroller = IDroneController()
process = start_server_process('color_track_behavior.html')
server = WebsocketServer(8080, '0.0.0.0')
server.set_fn_new_client(new_client)
server.set_fn_message_received(message_received)
server.run_forever()

# KILL
# initiateRobotCommunication()
# time.sleep( 5 )
# closeRobotCommunication()