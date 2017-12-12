#!/usr/bin/env python

""" Creates a node which sends acceleration and positions commands to the car at a specified rate
"""
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from time import sleep

class Parser():
	def __init__(self):
		self.ROS_RATE = 5		#Hz
	
	def run(self):
		""" Setup the node and send the commands to the car"""
		rospy.init_node('parse_lidar', anonymous=True)
		rospy.Subscriber("pc2", PointCloud2, self.callback)
		#self.prev_time = rospy.get_time()
		rate = rospy.Rate(self.ROS_RATE) # 5 hz, 200ms
		sleep(2)
		while not rospy.is_shutdown():
			rate.sleep()
	
	def callback(self,data):
		points = pc2.read_points(data)
		x = []
		y = []
		z = []
		for pt in points:
			if  .1> pt[0] > 0 and .1>pt[1]>0:
				x.append(pt[0])
				y.append(pt[1])
				z.append(pt[2])
				rospy.loginfo("X,Y: %s" % str((pt[0],pt[1])))

if __name__ == '__main__':
    try:
		parser = Parser()
		parser.run()
    except rospy.ROSInterruptException:
        pass
