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

# Open CV
import threading
from pid_camstream.image_app_core import start_server_process, get_control_instruction, put_output_image
import cv2
import numpy as np
import pid_camstream.pi_camera_stream as pi_camera_stream
from pid_camstream.pid_controller import PIController

# IRobot Controller
from robotcontroller.irobot_controller import IRobotController

# Line Followers
from followline.follow_line import LineTrackingBehavior

# Magnetometer
from magnetometer.berryIMU import BerryIMUCompass

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
		g_targetAngle = angleGoal - 180 + g_error_range
		if (g_targetAngle < 0):
			g_targetAngle = g_targetAngle + 360
		if (g_targetAngle < 0):
			g_targetAngle = 0
		output = "**CHANGE STATE: STATE_ROTATE_TO_ANGLE=%d" % g_targetAngle 
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
				print ('++++REPORTING CHANGE TO NEXT ANGLE')
				server.send_message_to_all("targetangle_success")
			else:
				posangle = message.find("_angle_")
				vend = message.find("_end")
				if g_targetAngle > 100000 and posangle > 0 and vend > 0 and (g_stateRoombaTarget == STATE_IDLE or g_stateRoombaTarget == STATE_FREE_CONTROL):
					changeStateRobot(STATE_ROTATE_TO_ANGLE, float(message[posangle+7:vend]))
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
			print ('turnRight vehicle:: YAW = %5.2f', berryIMUsensor.getYawValue())
			changeStateRobot(STATE_FREE_CONTROL, 100001)
			irobotcontroller.driveDirectRot( 0, 10)   	# Slowly turns the robot clockwise.  Right wheel at -5cm/s.  Left wheel at 5cm/s.
			time.sleep( time_sleep ) 					# Wait X seconds
			irobotcontroller.driveDirect( 0, 0 )      	# Full stop!
			server.send_message_to_all("turnright_success")
		if "turnLeft" in message:
			print ('turnLeft vehicle:: YAW = %5.2f', berryIMUsensor.getYawValue())
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
			print ('genericAction:: YAW = %5.2f', berryIMUsensor.getYawValue())
			irobotcontroller.beepSound()
			server.send_message_to_all("genericAction_success")
		if "closeCommunication" in message:
			print ('Close Communication ++START++')
			initilitzationSystem()
			irobotcontroller.closeRobotCommunication()
			print ('Close Communication ++END++')

##########################################################################
##########################################################################
#---------ColorTrackingBehavior
##########################################################################
##########################################################################

class ColorTrackingBehavior(object):
	"""Behavior to find and get close to a colored object"""
	def __init__(self, interval=1):
		# Tuning values
        
		# GREEN
		#self.low_range = (45, 52, 72)
		#self.high_range = (65, 255, 255)
		self.green_low_range = (45, 52, 72)
		self.green_high_range = (85, 255, 255)
		
		# RED
		#self.low_range = (0, 120, 70)
		#self.high_range = (10, 255, 255)
		self.red_low_range = (170, 120, 70)
		self.red_high_range = (180, 255, 255)

		self.correct_radius = 120
		self.center = 160

		# Thread
		self.interval = interval

		# Line Follower
		self.image_line_processed = np.zeros((240,320,3), np.uint8)

		thread = threading.Thread(target=self.run, args=())
		thread.daemon = True                            # Daemonize thread
		thread.start()   

	def getRightQuadrant(self, angle):
		q1 = abs(angle - 0)
		q2 = abs(angle - 90)
		q3 = abs(angle + 90)
		q4 = abs(angle - 180)
		
		finalQ = 0
		valueMin = q1
		if q1 < valueMin:
			finalQ = 0
			valueMin = q1
		if q2 < valueMin:
			finalQ = 90
			valueMin = q2
		if q3 < valueMin:
			finalQ = 270
			valueMin = q3
		if q4 < valueMin:
			finalQ = 180
			valueMin = q4
			
		return finalQ

	def rotateToQuadrant(self, finalQuadrant):
		global irobotcontroller
		global g_currentAngle
		if g_currentAngle == 0 and finalQuadrant == 270:
			irobotcontroller.driveDirectRotTime( 0, -10, 3.5)
			g_currentAngle = 270
			return g_currentAngle
		if g_currentAngle == 270 and finalQuadrant == 0:
			irobotcontroller.driveDirectRotTime( 0, 10, 3.5)
			g_currentAngle = 0
			return g_currentAngle
		if g_currentAngle < finalQuadrant:
			irobotcontroller.driveDirectRotTime( 0, 10, 3.5)
			g_currentAngle = g_currentAngle + 90
			return g_currentAngle
		if g_currentAngle > finalQuadrant:
			irobotcontroller.driveDirectRotTime( 0, -10, 3.5)
			g_currentAngle = g_currentAngle - 90
			return g_currentAngle
		
		return -1

	def process_control(self):
		instruction = "start"
		if instruction == "start":
			self.running = True
		elif instruction == "stop":
			self.running = False
		elif instruction == "exit":
			print("Stopping")
			exit()

	def find_object(self, original_frame):
		global g_isRedWaypoint
		global g_checkWhatColor
		"""Find the largest enclosing circle for all contours in a masked image.
		Returns: the masked image, the object coordinates, the object radius"""
		frame_hsv = cv2.cvtColor(original_frame, cv2.COLOR_BGR2HSV)
		if g_isRedWaypoint:
			masked = cv2.inRange(frame_hsv, self.red_low_range, self.red_high_range)
			if g_checkWhatColor:
				g_checkWhatColor = False
				print("Checking color RED!!!")
		else:
			masked = cv2.inRange(frame_hsv, self.green_low_range, self.green_high_range)
			if g_checkWhatColor:
				g_checkWhatColor = False
				print("Checking color GREEN!!!")

		# Find the contours of the image (outline points)
		contour_image = np.copy(masked)
		contours, _ = cv2.findContours(contour_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
		# Find enclosing circles
		circles = [cv2.minEnclosingCircle(cnt) for cnt in contours]
		# Filter for the largest one
		largest = (0, 0), 0
		for (x, y), radius in circles:
			if radius > largest[1]:
				largest = (int(x), int(y)), int(radius)
		return masked, largest[0], largest[1]

	def make_display(self, frame, processed):
		"""Create display output, and put it on the queue"""
		# Make a dualscreen view - two images of the same scale concatenated
		display_frame = np.concatenate((frame, processed, self.image_line_processed), axis=1)
		encoded_bytes = pi_camera_stream.get_encoded_bytes_for_frame(display_frame)
		put_output_image(encoded_bytes)

	def process_frame(self, frame):
		# Find the largest enclosing circle
		masked, coordinates, radius = self.find_object(frame)
		# Now back to 3 channels for display
		processed = cv2.cvtColor(masked, cv2.COLOR_GRAY2BGR)
		# Draw our circle on the original frame, then display this
		cv2.circle(frame, coordinates, radius, [255, 0, 0])
		self.make_display(frame, processed)
		# Yield the object details
		return coordinates, radius

	def run_follow_line(self, frame):
		global linetracking
		self.image_line_processed = cv2.copyMakeBorder(frame,0,0,0,0,cv2.BORDER_REPLICATE)
		final_turn, final_angle, final_shift, self.image_line_processed = linetracking.follow_way_line(self.image_line_processed)
		# print("++TURN(%d)::ANGLE(%f)::ANGLE(%f)" % (final_turn, final_angle, final_shift))
		return final_turn, final_angle, final_shift
		
	def run_towards_waypoint(self, radius, distanceAbsolute):
		global irobotcontroller
		global g_timeAcumToRunAction
		global g_currentIterations
		global g_distanceFromCenter
		global g_reportReachedTarget
		global g_checkWhatColor
		global g_isRedWaypoint
		found_marker = True
		g_timeAcumToRunAction = 0
		finalPush = False
		if self.isInsideCameraFocus(distanceAbsolute, radius):
			g_currentIterations = g_currentIterations + 1
			if g_distanceFromCenter < 0:
				if distanceAbsolute > 20:
					#print("++ROTATE A::Distance from center: %d" % g_distanceFromCenter)
					irobotcontroller.driveDirectRotTime( 0, 5, 0.25)
				else:
					irobotcontroller.driveDirectTime( 5, 5, 1 )
			else:
				if distanceAbsolute > 20:
					#print("++ROTATE B::Distance from center: %d" % g_distanceFromCenter)
					irobotcontroller.driveDirectRotTime( 0, -5, 0.25)
				else:
					irobotcontroller.driveDirectTime( 5, 5, 1 )
		else:
			if (distanceAbsolute >= 80 or (radius < 10 and distanceAbsolute < 80)) and (g_currentIterations > 2):
				irobotcontroller.driveDirectTime( 10, 10, 3 )
				finalPush = True
				print("++++FINAL PUSH")
			else:
				found_marker = False
				print("++++MARKER NOT FOUND")
				
		if finalPush:
			changeStateRobot(STATE_IDLE, 100001)
			g_reportReachedTarget = True
			g_checkWhatColor = True
			if g_isRedWaypoint:
				g_isRedWaypoint = False
			else:
				g_isRedWaypoint = True

		return found_marker

	def rotateToYaw(self, angleTargetYaw):
		global berryIMUsensor
		global irobotcontroller
		global g_currentAngle
		global g_accuracy_angle
		global g_iterator_surpass
		global g_last_direction_rotation
		global g_sign_diference_last_rotation
		yawAngleBerry = berryIMUsensor.getYawValue()
		print ("yawAngleBerry %5.2f , angleTargetYaw %5.2f #" % (yawAngleBerry,angleTargetYaw))
		
		# CHECK IF ANGLE INSIDE DETECTION RANGE.
		currentYaw = yawAngleBerry
		if (angleTargetYaw > 360 - g_accuracy_angle):
			currentYaw = currentYaw + 360
		else:	
			if (angleTargetYaw < g_accuracy_angle):
				if (currentYaw > 360 - g_accuracy_angle):
					currentYaw = currentYaw - 360
		
		g_sign_diference_last_rotation = currentYaw - angleTargetYaw				
		foundAngle = (abs(angleTargetYaw - currentYaw) < g_accuracy_angle)

		# KEEP GOING FOR g_iterator_surpass CYCLES.
		runNormaLogic = True
		if foundAngle:
			if (angleTargetYaw - currentYaw < 0) and (g_sign_diference_last_rotation > 0):
				if g_iterator_surpass > 0:
					runNormaLogic = False
					g_iterator_surpass = g_iterator_surpass - 1
					# print("******rotateToYaw::g_iterator_surpass[%d]::runNormaLogic[++AAA+++]"% (g_iterator_surpass))
					if g_last_direction_rotation:
						irobotcontroller.driveDirectRotTime( 0, 10, 0.18)
					else:
						irobotcontroller.driveDirectRotTime( 0, -10, 0.18)
			else:
				if (angleTargetYaw - currentYaw > 0) and (g_sign_diference_last_rotation < 0):
					if g_iterator_surpass > 0:
						runNormaLogic = False
						g_iterator_surpass = g_iterator_surpass - 1
						# print("******rotateToYaw::g_iterator_surpass[%d]::runNormaLogic[+++BBB+++]"% (g_iterator_surpass))
						if g_last_direction_rotation:
							irobotcontroller.driveDirectRotTime( 0, 10, 0.18)
						else:
							irobotcontroller.driveDirectRotTime( 0, -10, 0.18)
				
		# NORMAL ALINEATION.
		if runNormaLogic:
			if ((yawAngleBerry <= 90 + g_accuracy_angle) and (angleTargetYaw >= 270)):
				# print("-----SPECIAL ROTATION")
				g_last_direction_rotation = False
				irobotcontroller.driveDirectRotTime( 0, -10, 0.18)
			else:
				if (yawAngleBerry > angleTargetYaw):
					g_last_direction_rotation = False
					irobotcontroller.driveDirectRotTime( 0, -10, 0.15)
				else:
					g_last_direction_rotation = True
					irobotcontroller.driveDirectRotTime( 0, 10, 0.15)

		return foundAngle
		
	def lookForWaypointExistance(self, radius, x, direction_pid, speed_pid):
		global g_distanceFromCenter
		# The size is the first error
		radius_error = self.correct_radius - radius
		speed_value = speed_pid.get_value(radius_error)
		# And the second error is the based on the center coordinate.
		direction_error = self.center - x
		direction_value = direction_pid.get_value(direction_error)
		g_distanceFromCenter = (100 * (self.center - x)) / self.center						
		return abs(g_distanceFromCenter)

	def isInsideCameraFocus(self, distanceAbsolute, radius):
		return radius > 5 and distanceAbsolute < 85

	def run(self):
		global berryIMUsensor
		global irobotcontroller
		global g_distanceFromCenter
		global g_targetAngle
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
		# start camera
		camera = pi_camera_stream.setup_camera()
		# speed pid - based on the radius we get.
		speed_pid = PIController(proportional_constant=0.8, 
			integral_constant=0.1, windup_limit=100)
		# direction pid - how far from the middle X is.
		direction_pid = PIController(proportional_constant=0.25, 
			integral_constant=0.1, windup_limit=400)
		# warm up and servo move time
		# time.sleep(0.1)
		# Servo's will be in place - stop them for now.
		# self.robot.servos.stop_all()			
		print("Setup Complete")
		# Main loop
		for frame in pi_camera_stream.start_stream(camera):
			# Process Image
			(x, y), radius = self.process_frame(frame)
			self.process_control()
			# ----------------------------------------------
			if g_stateRoombaTarget == STATE_ROTATE_TO_ANGLE:
				if g_targetAngle < 100000:
					# DISPLAY ENTER STATE MESSAGE
					displayOneTimeMessage("++STATE_ROTATE_TO_ANGLE_%1.2f" % g_targetAngle)
					# CALCULATE TIME FOR ACTION
					calculateDeltaTime()
					# RUN ROTATE TO ANGLE ACTION
					if g_timeAcumToRunAction > 500:
						#finalQuadrant = self.getRightQuadrant(g_targetAngle)
						#localQuadrant = self.rotateToQuadrant(finalQuadrant)
						shouldGoToTarget = False
						if self.rotateToYaw(g_targetAngle):
							distanceAbsolute = self.lookForWaypointExistance(radius, x, direction_pid, speed_pid)
							shouldGoToTarget = self.isInsideCameraFocus(distanceAbsolute, radius)
						if shouldGoToTarget:
							changeStateRobot(STATE_GO_TO_TARGET, 100001)
							printAngle = "REACHED ANGLE %5.2f AND BERRY SENSOR YAW %5.2f" % (g_targetAngle, berryIMUsensor.getYawValue())
							print(printAngle)
						else:
							g_timeAcumToRunAction = 0
			# ----------------------------------------------
			if g_stateRoombaTarget == STATE_GO_TO_TARGET:
				distanceAbsolute = self.lookForWaypointExistance(radius, x, direction_pid, speed_pid)
				if self.running:
					# DISPLAY ENTER STATE MESSAGE
					displayOneTimeMessage("++STATE_GO_TO_TARGET_radius_%1.2f" % radius)
					# CALCULATE TIME FOR ACTION
					calculateDeltaTime()
					# RUN MOVE TO TARGET ACTION
					if g_timeAcumToRunAction > 1100:
						foundColorMarker = self.run_towards_waypoint(radius, distanceAbsolute)
						#if foundColorMarker == False:
						if False:
							final_turn, final_angle, final_shift = self.run_follow_line(frame)
							if final_turn != -1 and final_angle != -1 and final_shift != -1:
								g_currentIterations = 0
								if abs(final_shift) > 5:
									if final_shift > 0:
										irobotcontroller.driveDirectRotTime( 0, 5, 0.2)
									else:
										irobotcontroller.driveDirectRotTime( 0, -5, 0.2)
								else:
									irobotcontroller.driveDirectTime( 5, 5, 1 )
				else:
					# self.robot.stop_motors()
					if not self.running:
						speed_pid.reset()
						direction_pid.reset()

initilitzationSystem()
behavior = ColorTrackingBehavior()
berryIMUsensor = BerryIMUCompass()
irobotcontroller = IRobotController()
linetracking = LineTrackingBehavior()
process = start_server_process('color_track_behavior.html')
server = WebsocketServer(8080, '0.0.0.0')
server.set_fn_new_client(new_client)
server.set_fn_message_received(message_received)
server.run_forever()

# KILL
# initiateRobotCommunication()
# time.sleep( 5 )
# closeRobotCommunication()