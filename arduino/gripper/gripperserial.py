#Read from Arduino


import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
#Read from Arduino
#while True:
#	print ser.readline()
#'1 Hello world!\r\n'
#'2 Hello world!\r\n'
#'3 Hello world!\r\n'


def grippercontrol(cmd):
	"""Sends commands to a servo via an arduino using pyserial. 
	Must import serial before using, and initialize serial port
	import serial
    ser = serial.Serial('/dev/ttyACM0', 9600)

	cmd: string, 'open', 'wide', or 'narrow'
	"""
	cmd.lower()
	if cmd == 'open':
		val = '0'
	elif cmd == 'narrow':
		val = '1'
	elif cmd == 'wide':
		val == '2'
	else:
		print 'Invalid Command, please give "open", "wide" or "narrow" as inputs'
		break
	ser.write(val)


