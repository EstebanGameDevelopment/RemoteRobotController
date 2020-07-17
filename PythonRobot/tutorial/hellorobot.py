import serial

# sendCommandASCII takes a string of whitespace-separated, ASCII-encoded base 10 values to send
def sendCommandASCII(command):
	global connection
	cmd = ""
	for v in command.split():
		cmd += chr(int(v))
	connection.write(cmd)

port = serial.Serial("/dev/ttyUSB0")
connection = serial.Serial( str(port), baudrate=115200, timeout=1 )
sendCommandASCII('140 3 1 64 16 141 3')