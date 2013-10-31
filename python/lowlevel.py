import st
import time
import serial
import roslib
roslib.load_manifest('tetris_arm')
import sys
import rospy

#to subscribe to and publish images
from std_msgs.msg import String, Bool, UInt16
from tetris_arm.msg import TetArmArray

class ArmWrapper():
	def __init__(self, arm = 0):
		if arm == 0:
			self.arm = st.StArm('/dev/ttyUSB1', init = True)
			self.arm.start()
		else: self.arm = arm

		self.ser = serial.Serial('/dev/ttyACM1', 9600)
		self.gripper('2')					#start gripper open)
		
		# set up arm starting position
		self.arm.cartesian()
		self.arm.move_to(100, 6000, 0)
		self.arm.rotate_hand(1750)
		self.arm.cartesian()
		#self.go_home()

		# set constant heights to operate at
		self.zup = 500
		self.zdown = 0

		# toggle "picking" and "placing"
		self.pickandplace = "pick"
		
		# set current piece size  (0 - closed, 1 - small, 2 - big) TODO
		self.pieceSize = 0

		# Set up talkers and listeners
		self.gripperPub = rospy.Publisher("gripperSize", UInt16)
		self.holdingPub = rospy.Publisher("holding", Bool)
		self.pieceSub = rospy.Subscriber("piece", TetArmArray, self.goXYTH)
		self.downSub = rospy.Subscriber("downCmd", Bool, self.down)
		print 'set up pubsubs'

		self.x = 100
		self.y = 6000


	def goXYTH(self, data):
		if type(data) != tuple:
			data = data.data
		print 'Going!'
		if self.pickandplace == "pick":
			self.x, self.y, th, size = data
			self.gripper('2')
			self.size = size
		elif self.pickandplace == "place":
			print 'place mode'
			self.x, self.y, th, self.size = data
			self.gripper(str(self.size))
			#index, y, th, size = data
			#x = int(index*300.-1400)  #maps [0 to 10] to [-1400 to 1600]
		z = 0
		self.arm.move_to(self.x,self.y,z)
		self.rotate_gripper(th)
		self.arm.cartesian()

	def down (self, size):
		z = 100
		if self.size == 1:
			z = 300
		if self.size == 0:
			z = 100
		self.arm.move_to(self.x, self.y, -z)
		if self.pickandplace == "pick":
			self.gripper(str(size))
			holding = 1
		else: 
			self.gripper('0')
			holding = 0
		time.sleep(0.5)
		self.arm.move_to(self.x, self.y, z)
		self.holdingPub.publish(Bool(holding))		# send msg up to midbrain saying that the piece has been picked up/placed
			
	def gripper(self, size):
		#self.gripperPub.publish(UInt16(size))
		self.ser.write(size)
		print "sent gripper size"
		time.sleep(0.5)				#give time to pick up piece!

	def rotate_gripper(self, orientation):
		if orientation == 1:		# vertical
			self.arm.rotate_wrist(700)
		if orientation == 0:		# horizontal
			self.arm.rotate_wrist(-2300)
		self.arm.cartesian()

	def go_home(self):
		self.goXYTH((100,6000,0,2))
		
		

def main(args):
	rospy.init_node('armController', anonymous=True)
	aw = ArmWrapper()
	aw.pickandplace = 'place'
	aw.goXYTH((1000, 6000, 1, 2))
	aw.down(1)
	aw.goXYTH((-1000, 6000, 1, 1))
	aw.down(2)
	aw.go_home()
	
	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
