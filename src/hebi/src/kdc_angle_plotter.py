import rospy
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import math
import time

class Listener:
	def __init__(self):
		self.listener = rospy.Subscriber("kdc/arm_data", Float32MultiArray, self.callback)

		self.theta1_L = 0.0
		self.theta2_R = 0.0
		self.theta1_R = 0.0
		self.theta2_L = 0.0
		self.x = 0.0
		self.y = 0.0
		self.joint1 = 0.0
		self.joint2 = 0.0

		self.count = 0

		self.legendFlag = 0

	def callback(self, data):
		self.count = self.count + 1

		self.theta1_L = data.data[0]
		self.theta2_L = data.data[1]

		self.theta1_R = data.data[2]
		self.theta2_R = data.data[3]

		self.x = data.data[4]
		self.y = data.data[5]

		self.joint1 = data.data[6]
		self.joint2 = data.data[7]

		plt.figure(1)
		plt.ion()
		plt.show()
		self.plotData()

	def plotData(self):		
		# L1 = 0.277
		# L2 = 0.288

		# plt.figure(1)
		# plt.axis([-1, 1, -1, 1])

		# plt.plot([0, L1*math.cos(self.theta1_L), L1*math.cos(self.theta1_L) + L2*math.cos(self.theta1_L+self.theta2_R)], [0, L1*math.sin(self.theta1_L), L1*math.sin(self.theta1_L) + L2*math.sin(self.theta1_L+self.theta2_R)], 'b')
		# plt.plot([0, L1*math.cos(self.theta1_R), L1*math.cos(self.theta1_R) + L2*math.cos(self.theta1_R+self.theta2_L)], [0, L1*math.sin(self.theta1_R), L1*math.sin(self.theta1_R) + L2*math.sin(self.theta1_R+self.theta2_L)], 'g')
		
		# plt.scatter(self.x, self.y, color='r')

		# x = L1*math.cos(self.theta1_L) + L2*math.cos(self.theta1_L+self.theta2_R)
		# y = L1*math.sin(self.theta1_L) + L2*math.sin(self.theta1_L+self.theta2_R)

		# plt.title('X = %s, Y = %s'%(x,y))
		# plt.draw()
		# plt.clf()
		
		plt.figure(1)
		plt.suptitle('Desired vs. Actual Joint Angles')
		plt.subplot(211)
		# plt.scatter(self.count, self.theta1_L, color='r', label='Desired theta_1')
		# plt.scatter(self.count, self.joint1, color='b', label='Actual theta_1')
		plt.scatter(self.count, self.theta1_L - self.joint1, color='r', label='theta_1 Error')
		plt.scatter(self.count, 0.0, color='b')
		plt.xlabel('Time')
		plt.ylabel('Angle (radians)')
		if not self.legendFlag:
			plt.legend(loc=4)

		plt.subplot(212)
		# plt.scatter(self.count, self.theta2_R, color='r', label='Desired theta_2')
		# plt.scatter(self.count, self.joint2, color='b', label='Actual theta_2')
		plt.scatter(self.count, self.theta2_R - self.joint2, color='r', label='theta_2 Error')
		plt.scatter(self.count, 0.0, color='b')
		plt.xlabel('Time')
		plt.ylabel('Angle (radians)')
		if not self.legendFlag:
			plt.legend(loc=4)

		plt.draw()

		# if not self.legendFlag:
		# 	plt.pause(10)

		self.legendFlag = 1

if __name__=='__main__':
	rospy.init_node('arm_plotter')
	arm = Listener()
	rospy.spin()