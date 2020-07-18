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
from robotcontroller.irobot_controller import IRobotController

##########################################################################
##########################################################################
#---------GLOBAL VARIABLES FOR ROBOT STATE
##########################################################################
##########################################################################

STATE_DISCONNECTED = -2
STATE_IDLE = -1
STATE_FREE_CONTROL = 1

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

def initilitzationSystem():
	global STATE_DISCONNECTED
	global g_distanceFromCenter
	global g_targetAngle
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

def changeStateRobot(newState, angleGoal):
	global g_targetAngle
	global g_previousTimeNow
	global g_currentIterations
	global g_timeAcumToRunAction
	global g_displayOneTimeMessage
	global g_stateRoombaTarget
	global g_error_range
	global g_iterator_surpass
	global STATE_DISCONNECTED
	global STATE_IDLE
	global STATE_FREE_CONTROL
	g_stateRoombaTarget = newState
	g_currentIterations = 0
	g_previousTimeNow = 0
	g_timeAcumToRunAction = 0
	g_displayOneTimeMessage = True
	if g_stateRoombaTarget == STATE_IDLE:
		g_targetAngle = 100001
		print ('**CHANGE STATE: STATE_IDLE')
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
		irobotcontroller.initiateRobotCommunication()
		changeStateRobot(STATE_IDLE, 100001)
		server.send_message_to_all("Client connected")
def message_received(client, server, message):
		global berryIMUsensor
		global irobotcontroller
		global g_distanceFromCenter
		global g_distanceFromCenter
		global g_targetAngle
		global g_reportReachedTarget
		global g_displayOneTimeMessage
		global g_previousTimeNow
		global g_timeAcumToRunAction
		global g_isRedWaypoint
		global g_stateRoombaTarget
		global STATE_IDLE
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
		if "moveForward" in message:
			print ('moveForward vehicle')
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.driveDirect( 10, 10 )   		# Go straight for two seconds at 10 cm/sec
			time.sleep( time_sleep )  				  		# Wait X seconds
			irobotcontroller.driveDirect( 0, 0 )       		# Full stop!
			if irobotcontroller.bumped(): 
				irobotcontroller.driveDirect( 0, 0 )	# If the bumper is hit, go backwards
			server.send_message_to_all("moveforward_success")
		if "moveBackward" in message:
			print ('moveBackward vehicle')
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.driveDirect( -10, -10 )		# Go backward for two seconds at 10 cm/sec
			time.sleep( time_sleep )  						# Wait X seconds
			irobotcontroller.driveDirect( 0, 0 )       		# Full stop!
			if irobotcontroller.bumped(): 
				irobotcontroller.driveDirect( 0, 0 )		# If the bumper is hit, stop
			server.send_message_to_all("movebackward_success")
		if "turnRight" in message:
			print ('turnRight vehicle')
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.driveDirectRot( 0, 10)   	# Slowly turns the robot clockwise.  Right wheel at -5cm/s.  Left wheel at 5cm/s.
			time.sleep( time_sleep ) 					# Wait X seconds
			irobotcontroller.driveDirect( 0, 0 )      	# Full stop!
			server.send_message_to_all("turnright_success")
		if "turnLeft" in message:
			print ('turnLeft vehicle')
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.driveDirectRot( 0, -10)	# Slowly turns the robot clockwise.  Right wheel at -5cm/s.  Left wheel at 5cm/s.
			time.sleep( time_sleep )  					# Wait X seconds
			irobotcontroller.driveDirect( 0, 0 )		# Full stop!
			server.send_message_to_all("turnleft_success")
		if "requestDirection" in message:
			output_send = "requestdirection_success_%1.2f" % g_distanceFromCenter
			# print (output_send)
			server.send_message_to_all(output_send)
		if "genericAction" in message:
			print ('genericAction')
			irobotcontroller.beepSound()
			server.send_message_to_all("genericAction_success")
		if "closeCommunication" in message:
			print ('Close Communication ++START++')
			initilitzationSystem()
			irobotcontroller.closeRobotCommunication()
			print ('Close Communication ++END++')


initilitzationSystem()
irobotcontroller = IRobotController()
server = WebsocketServer(8080, '0.0.0.0')
server.set_fn_new_client(new_client)
server.set_fn_message_received(message_received)
server.run_forever()

# KILL
# initiateRobotCommunication()
# time.sleep( 5 )
# closeRobotCommunication()