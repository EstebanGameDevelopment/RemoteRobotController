from Tkinter import *
import math
import time
import thread
import socket
import select
import struct
import random
import sys, glob # for listing serial ports
import os  # to command the mp3 and wav player omxplayer

try:
    import serial
except ImportError:
    print "Import error.  Please install pyserial."
    raise

global FAILURE
FAILURE = False

class IRobotController(object):
	"""Fucntionality to control the robot"""
	def __init__(self, interval=1):
		self.connection = None

	def toTwosComplement2Bytes(self, value ):
			""" returns two bytes (ints) in high, low order
			whose bits form the input value when interpreted in
			two's complement
			"""
			# if positive or zero, it's OK
			if value >= 0:
				eqBitVal = value
				# if it's negative, I think it is this
			else:
				eqBitVal = (1<<16) + value
		
			return ( (eqBitVal >> 8) & 0xFF, eqBitVal & 0xFF )

	# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
	def sendCommandASCII(self, command):
		cmd = ""
		for v in command.split():
			cmd += chr(int(v))
		self.sendCommandRaw(cmd)

	# sendCommandRaw takes a string interpreted as a byte array
	def sendCommandRaw(self, command):
		try:
			if self.connection is not None:
				self.connection.write(command)
			else:
				print "Not connected."
		except serial.SerialException:
			print "Lost connection"
			self.connection = None
		#print ' '.join([ str(ord(c)) for c in command ])

	# getDecodedBytes returns a n-byte value decoded using a format string.
	# Whether it blocks is based on how the connection was set up.
	def getDecodedBytes(self, n, fmt):
		try:
			return struct.unpack(fmt, self.connection.read(n))[0]
		except serial.SerialException:
			print "Lost connection"
			tkMessageBox.showinfo('Uh-oh', "Lost connection to the robot!")
			self.connection = None
			return None
		except struct.error:
			print "Got unexpected data from serial port."
			return None

	def bytesOfR(self, r ):
			""" for looking at the raw bytes of a sensor reply, r """
			print('raw r is', r)
			for i in range(len(r)):
				print('byte', i, 'is', ord(r[i]))
			print('finished with formatR')

	def toBinary(self, val, numBits ):
			""" prints numBits digits of val in binary """
			if numBits == 0:  return
			self.toBinary( val>>1 , numBits-1 )
	#        print((val & 0x01), end=' ')  # print least significant bit

	def bitOfByte(self, bit, byte ):
		""" returns a 0 or 1: the value of the 'bit' of 'byte' """
		if bit < 0 or bit > 7:
			print('Your bit of', bit, 'is out of range (0-7)')
			print('returning 0')
			return 0
		return ((byte >> bit) & 0x01)

	# get8Unsigned returns an 8-bit unsigned value.
	def get8Unsigned(self):
		return self.getDecodedBytes(1, "B")

	# get lowest bit from an unsigned byte
	def getLowestBit(self):
		wheelsAndBumpsByte = self.getDecodedBytes(1, "B")
		print wheelsAndBumpsByte
		return self.bitOfByte(0, wheelsAndBumpsByte)

	# get second lowest bit from an unsigned byte
	def getSecondLowestBit(self):
		wheelsAndBumpsByte = self.getDecodedBytes(1, "B")
		print wheelsAndBumpsByte
		return self.bitOfByte(1, wheelsAndBumpsByte)

	def bumped(self):
		self.sendCommandASCII('142 7') 
		time.sleep( 0.02 )
		bumpedByte = self.getDecodedBytes( 1, "B" )
		if bumpedByte == 0:
			return False
		elif bumpedByte > 3:
			print "CRAZY BUMPER SIGNAL!"
		else:
			return True

	def cleanButtonPressed(self):
		self.sendCommandASCII('142 18') 
		buttonByte = self.getDecodedBytes( 1, "B" )
		if buttonByte == 0:
			return False
		elif buttonByte == 1:
			print "Clean Button Pressed!"
			return True
		elif buttonByte == 4:
			return False
		else:
			print "Some other button pressed!"
			FAILURE = True
			return False

	def dockButtonPressed(self):
		self.sendCommandASCII('142 18') 
		buttonByte = self.getDecodedBytes( 1, "B" )
		if buttonByte <> 4:
			return False
		else:
			print "Dock button pressed!"
			return True

	def shudder( self, period, magnitude, numberOfShudders):
		i = 0
		timestep = 0.02
		while i < numberOfShudders:
			i = i + 1
			#shake left
			t = 0
			while t < period:
				self.driveDirectRot( 0, magnitude )
				t = t + timestep
				time.sleep( timestep )
			#Shake right
			t = 0
			while t < period:
				self.driveDirectRot( 0, -magnitude )
				t = t + timestep
				time.sleep( timestep )
			self.driveDirect( 0, 0 )  # stop the previous motion command

	def onConnect(self):
		if self.connection is not None:
			print "Oops- You're already connected!"
			return

		try:
			ports = self.getSerialPorts()
			print "Available ports:\n" + '   '.join(ports)
			#port = raw_input("Port? Enter COM port to open.\nAvailable options:\n" + '\n'.join(ports))
			port = str( ports[0] )  # I'm guessing that the Roomba port is first in the list.  So far this works!  :)
		except EnvironmentError:
			port = raw_input("Port?  Enter COM port to open.")

		if port is not None:
			print "Trying " + str( port ) + "... "
		try:   #:tty
			#self.connection = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
			#self.connection = serial.Serial( str(port), baudrate=115200, timeout=1 )
			self.connection = serial.Serial( str(ports[0]), baudrate=115200, timeout=1 )
			print "Connected!"
		except:
			print "Failed.  Could not connect to " + str( port )

	def getSerialPorts(self):
		"""Lists serial ports
		From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

		:raises EnvironmentError:
			On unsupported or unknown platforms
		:returns:
			A list of available serial ports
		"""
		if sys.platform.startswith('win'):
			ports = ['COM' + str(i + 1) for i in range(256)]

		elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
			# this is to exclude your current terminal "/dev/tty"
			ports = glob.glob('/dev/tty[A-Za-z]*')

		elif sys.platform.startswith('darwin'):
			ports = glob.glob('/dev/tty.*')

		else:
			raise EnvironmentError('Unsupported platform')

		result = []
		for port in ports:
			try:
				s = serial.Serial(port)
				s.close()
				result.append(port)
			except (OSError, serial.SerialException):
				pass
		return result    

	def driveDirectTime(self, left, right, duration ):
		#print"driveDirectTime()"
		t = 0   # initialize timer
		while t < duration:
			self.driveDirect( left, right )
			time.sleep( 0.05 )
			t = t + .05
		self.driveDirect( 0, 0 )  # stop

	def driveDirect( self, leftCmSec = 0, rightCmSec = 0 ):
		""" sends velocities of each wheel independently
			   left_cm_sec:  left  wheel velocity in cm/sec (capped at +- 50)
			   right_cm_sec: right wheel velocity in cm/sec (capped at +- 50)
		"""
		#print "driveDirect()"
		if leftCmSec < -50: leftCmSec = -50
		if leftCmSec > 50:  leftCmSec = 50
		if rightCmSec < -50: rightCmSec = -50
		if rightCmSec > 50: rightCmSec = 50
		# convert to mm/sec, ensure we have integers
		leftHighVal, leftLowVal = self.toTwosComplement2Bytes( int( leftCmSec * 10 ) )
		rightHighVal, rightLowVal = self.toTwosComplement2Bytes( int( rightCmSec * 10 ) )

		# send these bytes and set the stored velocities
		byteListRight = ( rightHighVal , rightLowVal )
		byteListLeft = ( leftHighVal , leftLowVal )
		self.sendCommandRaw(struct.pack( ">Bhh", 145, int(rightCmSec * 10), int(leftCmSec * 10) ))
		return

	def driveDirectRot( self, robotCmSec = 0, rotation = 0 ):
		""" implements the driveDirect with a given rotation
			Positive rotation turns the robot CCW
			Negative rotation turns the robot CW
		"""
		#print "driveDirectRot()"
		vl = robotCmSec - rotation/2
		vr = robotCmSec + rotation/2
		self.driveDirect ( vl, vr )
		
	def driveDirectRotTime( self, robotCmSec = 0, rotation = 0, duration = 1 ):
		#print "driveDirectRotTime()"
		t = 0   # initialize timer
		vl = robotCmSec - rotation/2
		vr = robotCmSec + rotation/2
		while t < duration:
			self.driveDirect( vl, vr )
			time.sleep( 0.05 )
			t = t + .05
		self.driveDirect( 0, 0 )  # stop	

	def initiateRobotCommunication(self):
		print "Initiating Communications to the Create 2 Robot..."
		self.onConnect()
		time.sleep( 0.3 )
		self.sendCommandASCII('128')   # Start Open Interface in Passive
		time.sleep( 0.3 )
		self.sendCommandASCII('140 3 1 64 16 141 3')  # Beep
		time.sleep( 0.3 )
		#self.sendCommandASCII('131')   # Safe mode
		self.sendCommandASCII( '132' )   # Full mode 
		time.sleep( 0.3 )
		self.sendCommandASCII('140 3 1 64 16 141 3')  # Beep
		time.sleep( 0.1 )
		self.sendCommandASCII('139 4 0 255')  # Turn on Clean and Dock buttons
		time.sleep( 0.03 )

	def beepSound(self):
		self.sendCommandASCII('140 3 1 64 16 141 3')  # Beep
		
	def closeRobotCommunication(self):
		print "Closing Communication to the Create 2 Robot..."
		self.driveDirect( 0, 0 )  # stop robot if moving
		time.sleep( 0.05 )
		self.sendCommandASCII('140 3 1 64 16 141 3')  # Beep
		time.sleep( 0.3 )
		#self.sendCommandASCII('139 0 0 0')  # Turn off Clean and Dock buttons
		time.sleep( 0.03 )
		self.sendCommandASCII('138 0')  # turn off vacuum, etractors, and side brush
		time.sleep( 0.03 )
		#self.sendCommandASCII( '7' )  # Resets the robot 	
		self.sendCommandASCII( '173' )  # Stops the Open Interface to Roomba
		time.sleep( 0.3 )
		self.connection.close()
		time.sleep( 0.1 )
		self.connection = None
		time.sleep( 0.1 )
		# raise SystemExit	#  Exit program
