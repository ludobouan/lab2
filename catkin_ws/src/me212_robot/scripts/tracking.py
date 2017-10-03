#!/usr/bin/python
from Adafruit_MotorHAT import Adafruit_MotorHAT
import rospy
from std_msgs.msg import Float64MultiArray
from math import pi, radians
class Tracking:
	def __init__(self):
		self.node_name = rospy.get_name()
		self.state = 1
		self.trig = None
		self.motorhat = Adafruit_MotorHAT(addr= 0x60)
		self.leftMotor 	= self.motorhat.getMotor(1)
		self.rightMotor = self.motorhat.getMotor(2)
		self.right_pwm = 120
		self.left_pwm = 120
		self.leftMotor.setSpeed(self.left_pwm)
		self.rightMotor.setSpeed(self.right_pwm)
		self.subPosition=rospy.Subscriber("/serial_node/odometry",Float64MultiArray,self.cbPosition)
		self.track = ''

		rospy.on_shutdown(self.custom_shutdown)
		rospy.loginfo("[%s] Initialized!" %self.node_name)
	def cbPosition(self,msg):
		x     = msg.data[0]
		y     = msg.data[1]
		theta = msg.data[2]
		theta = theta % (2* pi)
		self.cbMove(x,y,theta)
		print x,y,theta

	def cbMove(self,x,y,theta):
		if x<0: self.track = 'stop'
		elif x<1: self.track = 'straight'
		else: self.track = 'turn'

		if self.track == 'straight':
			self.straight()
		elif self.track == 'turn':
			self.turn()
		elif self.track == 'stop':
			self.custom_shutdown()

	def straight(self):
		self.leftMotor.setSpeed(self.left_pwm)
		self.rightMotor.setSpeed(self.right_pwm)
		self.leftMotor.run(1)
		self.rightMotor.run(1)


	def turn(self):
		self.leftMotor.setSpeed(int(self.left_pwm / 2))
		self.rightMotor.setSpeed(self.right_pwm)
		self.leftMotor.run(1)
		self.rightMotor.run(1)

	def custom_shutdown(self):
		self.leftMotor.run(4)
		self.rightMotor.run(4)
		del self.motorhat

if __name__ == '__main__':
	rospy.init_node('tracking', anonymous = False)
	Track = Tracking()
	rospy.spin()
