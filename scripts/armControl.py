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
			self.arm = st.StArm('/dev/ttyUSB0', init = False)
		else: self.arm = arm

		self.ser = serial.Serial('/dev/ttyACM0', 9600)
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
		self.armSub = rospy.Subscriber("armArray", TetArmArray, self.goXYTH)
		self.downSub = rospy.Subscriber("downCmd", Bool, self.down)
		print 'set up pubsubs'

		self.x = 100
		self.y = 6000


	def goXYTH(self, data):
		if self.pickandplace == "pick":
			self.x, self.y, th, size = data
			self.gripper('2')
			self.size = size
		elif self.pickandplace == "place":
			print 'place mode'
			index, y, th, size = data
			x = int(index*300.-1400)  #maps [0 to 10] to [-1400 to 1600]
		z = 0
		self.arm.move_to(self.x,self.y,z)
		self.rotate_gripper(th)
		self.arm.cartesian()

	def down (self, size):
		self.arm.move_to(self.x,self.y,-300)
		if self.pickandplace == "pick":
			self.gripper(str(size))
			holding = 1
		else: 
			self.gripper('0')
			holding = 0
		self.holdingPub.publish(Bool(holding))		# send msg up to midbrain saying that the piece has been picked up/placed
			
	def gripper(self, size):
		#self.gripperPub.publish(UInt16(size))
		self.ser.write(size)
		print "sent gripper size"
		time.sleep(0.5)				#give time to pick up piece!

	def rotate_gripper(self, orientation):
		if orientation == 'v':
			self.arm.rotate_wrist(900)
		if orientation == 'h':
			self.arm.rotate_wrist(-2100)
		self.arm.cartesian()

	def go_home(self):
		self.goXYTH((100,6000,'h',2))
		
		

def main(args):
	rospy.init_node('armController', anonymous=True)
	aw = ArmWrapper()
	aw.pickandplace = 'pick'
	aw.down(1)
	aw.pickandplace = 'place'
	aw.goXYTH((-1000, 6000, 'h', 1))
	aw.down(2)
	aw.go_home()
	
"""	try:
		print "spin"
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down"
"""

if __name__ == '__main__':
    main(sys.argv)
