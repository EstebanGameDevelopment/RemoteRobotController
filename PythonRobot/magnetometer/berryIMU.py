import threading
import time
import math
import IMU
import datetime
import os


class BerryIMUCompass(object):
	"""Fucntionality to control the robot"""
	def __init__(self, interval=1):
		self.RAD_TO_DEG = 57.29578
		self.M_PI = 3.14159265358979323846
		self.G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need to update this value accordingly
		self.AA =  0.40      # Complementary filter constant

		self.magXmin = -76
		self.magYmin = -766
		self.magZmin = -508
		self.magXmax = 1917
		self.magYmax = 1414
		self.magZmax = 1330

		self.gyroXangle = 0.0
		self.gyroYangle = 0.0
		self.gyroZangle = 0.0
		self.CFangleX = 0.0
		self.CFangleY = 0.0

		IMU.detectIMU()     #Detect if BerryIMUv1 or BerryIMUv2 is connected.
		IMU.initIMU()       #Initialise the accelerometer, gyroscope and compass

		self.atimenow = datetime.datetime.now()

		self.TOTAL_SAMPLES_TO_AVERAGE = 40.0
		self.average_heading = 0.0
		self.values_heading = []

		thread = threading.Thread(target=self.run, args=())
		thread.daemon = True                            # Daemonize thread
		thread.start()   

	# Get the average Yaw value
	def getYawValue(self):
		return self.average_heading

	# Keep reading I/O from sensor
	def run(self):
		while True:
			#Read the accelerometer,gyroscope and magnetometer values
			ACCx = IMU.readACCx()
			ACCy = IMU.readACCy()
			ACCz = IMU.readACCz()
			GYRx = IMU.readGYRx()
			GYRy = IMU.readGYRy()
			GYRz = IMU.readGYRz()
			MAGx = IMU.readMAGx()
			MAGy = IMU.readMAGy()
			MAGz = IMU.readMAGz()

			#Apply compass calibration
			MAGx -= (self.magXmin + self.magXmax) /2
			MAGy -= (self.magYmin + self.magYmax) /2
			MAGz -= (self.magZmin + self.magZmax) /2

			##Calculate loop Period(LP). How long between Gyro Reads
			b = datetime.datetime.now() - self.atimenow
			self.atimenow = datetime.datetime.now()
			LP = b.microseconds/(1000000*1.0)
			outputString = "Loop Time %5.2f " % ( LP )

			#Convert Gyro raw to degrees per second
			rate_gyr_x =  GYRx * self.G_GAIN
			rate_gyr_y =  GYRy * self.G_GAIN
			rate_gyr_z =  GYRz * self.G_GAIN

			#Calculate the angles from the gyro.
			self.gyroXangle+=rate_gyr_x*LP
			self.gyroYangle+=rate_gyr_y*LP
			self.gyroZangle+=rate_gyr_z*LP

			#Convert Accelerometer values to degrees
			AccXangle =  (math.atan2(ACCy,ACCz)*self.RAD_TO_DEG)
			AccYangle =  (math.atan2(ACCz,ACCx)+self.M_PI)*self.RAD_TO_DEG

			#convert the values to -180 and +180
			if AccYangle > 90:
				AccYangle -= 270.0
			else:
				AccYangle += 90.0

			#Complementary filter used to combine the accelerometer and gyro values.
			self.CFangleX=self.AA*(self.CFangleX+rate_gyr_x*LP) +(1 - self.AA) * AccXangle
			self.CFangleY=self.AA*(self.CFangleY+rate_gyr_y*LP) +(1 - self.AA) * AccYangle

			#Calculate heading
			heading = 180 * math.atan2(MAGy,MAGx)/self.M_PI

			#Only have our heading between 0 and 360
			if heading < 0:
				heading += 360

			####################################################################
			###################Tilt compensated heading#########################
			####################################################################
			#Normalize accelerometer raw values.
			accXnorm = ACCx/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)
			accYnorm = ACCy/math.sqrt(ACCx * ACCx + ACCy * ACCy + ACCz * ACCz)

			#Calculate pitch and roll
			pitch = math.asin(accXnorm)
			roll = -math.asin(accYnorm/math.cos(pitch))

			#Calculate the new tilt compensated values
			magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)

			#The compass and accelerometer are orientated differently on the LSM9DS0 and LSM9DS1 and the Z axis on the compass
			#is also reversed. This needs to be taken into consideration when performing the calculations
			if(IMU.LSM9DS0):
				magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS0
			else:
				magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)   #LSM9DS1

			#Calculate tilt compensated heading
			tiltCompensatedHeading = 180 * math.atan2(magYcomp,magXcomp)/self.M_PI
			if tiltCompensatedHeading < 0:
				tiltCompensatedHeading += 360
				
			#Calculate average heading
			self.values_heading.insert(0,heading)
			if (len(self.values_heading) > self.TOTAL_SAMPLES_TO_AVERAGE):
				self.values_heading.pop()

			self.average_heading = 0
			for itemHeading in self.values_heading:
				self.average_heading = self.average_heading + itemHeading
				
			self.average_heading = self.average_heading / self.TOTAL_SAMPLES_TO_AVERAGE
				
			############################ END ##################################
			if 0:                       #Change to '0' to stop showing the angles from the accelerometer
				outputString += "# ACCX Angle %5.2f ACCY Angle %5.2f #  " % (AccXangle, AccYangle)

			if 0:                       #Change to '0' to stop  showing the angles from the gyro
				outputString +="\t# GRYX Angle %5.2f  GYRY Angle %5.2f  GYRZ Angle %5.2f # " % (self.gyroXangle,self.gyroYangle,self.gyroZangle)

			if 0:                       #Change to '0' to stop  showing the angles from the complementary filter
				outputString +="\t# CFangleX Angle %5.2f   CFangleY Angle %5.2f #" % (CFangleX,CFangleY)

			if 0:                       #Change to '0' to stop  showing the heading
				#outputString +="\t# YAW AVERAGE %5.2f  #" % (self.average_heading)
				outputString +="\t# HEADING %5.2f  tiltCompensatedHeading %5.2f #" % (heading,tiltCompensatedHeading)

			# print(outputString)

			#slow program down a bit, makes the output more readable
			time.sleep(0.03)
