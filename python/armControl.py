import st
import time
import serial

class ArmWrapper():
	def __init__(self, arm = 0):
		if arm == 0:
			self.arm = st.StArm('/dev/ttyUSB0')
			#self.arm.roate_wrist(x) so that gripper isn't knocked off!
			self.arm.rotate_hand(5900)	#pointing directly down
			self.arm.cartesian()
			self.arm.move_to(100, 6000, -1000)
		else: self.arm = arm
		self.ser = serial.Serial('/dev/ttyACM0', 9600)

	def goXY(self, index, height):
		x = int(index*300.-1400)  #maps [0 to 10] to [-1400 to 1600]
		y = int(height)    	  #2700 to 7300
		z = -1000
		print x,y
		self.arm.move_to(x,y,z)

	def release (self):
		pass 		# open the servo

	def grab (self, size):
		if piece == 1:
			pass
		if piece == 2:
			pass


'''
#aw = ArmWrapper(arm = 0)
while 1:
	cmd = raw_input("Tell me where to go! x y (x [0-10], y [0, 4600])\n")
	if cmd == 'quit':
		break
	cmdlist = cmd.split()
	if len(cmdlist) == 2:
		index = int(cmdlist[0])
		height = int(cmdlist[1])
		print index, height
		aw.goXY(index, height)
	else:
		print "I don't understand"
	
	#self.goXY(index, height)
'''
