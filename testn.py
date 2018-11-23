#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,sin,cos,radians,pi,sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import time
import test
import astar_compressed_path as acp
import draw_map as dm
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7,GPIO.OUT)

GPIO.output(7,False)

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.callback)
	self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.pose = Odometry()
        self.rate = rospy.Rate(10)
	self.angle = 0
	self.a = np.zeros(359)
	self.b = np.zeros(359)
	self.Rx = 0
	self.Ry = 0
	self.Rpi = 0
	self.Old_Rx = 0
	self.Old_Ry = 0
	self.Old_Rpi = 0
	self.Rvx = 0
	self.Rvy = 0
	self.Rv = 0
	self.Rw = 0
	self.tracking_on = 0
	self.patrol_on = 1
	self.tic = 0
	self.toc = 0
        self.sensing = 0
	self.t = 0.3
        self.Original_map = test.mapreading()
        self.lidar_map = np.zeros((30,30),int)
        self.Rx_i = 0
        self.Ry_i = 0
        self.lidar_flag = 1
        self.m2g_flag = 0
	self.final_x = 0
	self.final_y = 0
        self.path = []
        mymap = dm.astar()
        w = mymap.shape[0]
        l = mymap.shape[1]

        self.path = []

        for i in range(0,w,10) :
            new_mymap = []
            k = 0
            for j in range(0,l):
                if mymap[i][j] == 0 :
                    new_mymap.append(j)
                    k = k+1
            if k == 0 :
                continue
            self.path.append((i,new_mymap[0]))
            self.path.append((i,new_mymap[-1]))
            
        self.path.remove(self.path[0])
        
    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data.pose.pose.position
	self.orient = data.pose.pose.orientation
        self.pose.x = round(self.pose.x + 0.12, 4) 
        self.pose.y = round(self.pose.y + 0.12, 4)
	orientation_list = [self.orient.x, self.orient.y, self.orient.z, self.orient.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	theta = yaw
        self.angle = theta

    def callback_lidar(self,msg):
	if self.lidar_flag == 1:
		self.lidar_map = np.zeros((30,30),int)
		n = 0
		num = 0
		isum = 0
		jsum = 0
		self.Rx_i = 0
		self.Ry_i = 0

		self.toc = time.time()
		t = self.toc - self.tic
		if t >= 0.4:
		    #print('in')
		    self.tic = time.time()

		    self.Rx = 0
		    self.Ry = 0
		    
		    for i in range(359):
			self.a[i] = cos(radians(i) + self.angle)*round(msg.ranges[i],3) + self.pose.x
			self.b[i] = sin(radians(i) + self.angle)*round(msg.ranges[i],3) + self.pose.y
		        if (self.a[i] > 0 and self.a[i] < 1.2) and (self.b[i] > 0 and self.b[i] < 1.2):
		            x_occ = int(self.a[i]*(30/1.2))
		            y_occ = int(self.b[i]*(30/1.2))
		            self.lidar_map[x_occ][y_occ] = 1

		    for i in range(2,27):
                        for j in range(2,27):
		            if (self.lidar_map[i-1][j-1] == 0 and self.lidar_map[i-1][j] == 0 and self.lidar_map[i-1][j+1] == 0):
		                if (self.lidar_map[i][j-1] == 0 and self.lidar_map[i][j+1] == 0 and self.lidar_map[i+1][j-1] == 0):   
		                    if (self.lidar_map[i+1][j] == 0 and self.lidar_map[i+1][j+1] == 0):
		                        self.lidar_map[i][j] = 0
                    """
		    for i in range(2,27):
		        for j in range(2,27):
		            if self.lidar_map[i-1][j-1] == 1:
		                num += 1
		            if self.lidar_map[i-1][j] == 1:
		                num += 1
		            if self.lidar_map[i-1][j+1] == 1:
		                num += 1
		            if self.lidar_map[i][j-1] == 1:
		                num += 1
		            if self.lidar_map[i][j+1] == 1:
		                num += 1
		            if self.lidar_map[i+1][j-1] == 1:
		                num += 1
		            if self.lidar_map[i+1][j] == 1:
		                num += 1
		            if self.lidar_map[i+1][j+1] == 1:
		                num += 1
		            if num < 2:
		                self.lidar_map[i][j] = 0
		            num = 0    
                    """
		    for i in range(29):
		        for j in range(29):
		            if self.Original_map[i][j] == 0 and self.lidar_map[i][j] == 1:
		                isum += i
		                jsum += j
		                n += 1
		    #print isum
		    #print('lidar')

		    print n

		    if n == 0:
			n = 1

		    isum = isum/(n+0.0)
		    jsum = jsum/(n+0.0)

		    self.Rx_i = isum
		    self.Ry_i = jsum

		    #print isum
		    
		    self.Rx = round((isum*1.2/30),2)
		    self.Ry = round((jsum*1.2/30),2)
		    
		    #print self.Rx
		    #print self.Ry
		    

		    if n >= 5:
			print "target position ----> ",
			print self.Rx,
			print self.Ry
			print "target index    ----> ",
			print self.Rx_i,
			print self.Ry_i
			self.final_x = self.Rx_i
			self.final_y = self.Ry_i
		        self.tracking_on = 1
		    else:
			self.final_x = 0
			self.final_y = 0
		        self.tracking_on = 0

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
        return distance

    def move2goal(self,goal):
		goal_pose_ = Odometry()
		goal_pose = goal_pose_.pose.pose.position
		rospy.sleep(0.1)
		goal_pose.x = goal[0] #input("Set your x goal:")
		goal_pose.y = goal[1] #input("Set your y goal:")
		distance_tolerance = 0.01 #input("Set your tolerance:")
		vel_msg = Twist()
		r = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
		print "r : ",
		print r
		while r >= distance_tolerance:

		    #Proportional Controller
		    #linear velocity in the x-axis:
		    r = sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
		    psi = atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)
		    orientation_list = [self.orient.x, self.orient.y, self.orient.z, self.orient.w]
		    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
		    theta = yaw

		    #our control
		    phi = atan2(sin(psi-theta),cos(psi-theta))
		    beta = -phi - theta
		    
		    if np.abs(phi)>0.25 :
		        v = 0.0375*r
		        w = 6.4*phi - 0.15*beta
		    else :
		        v = 0.2
		        w = 0.05*phi - 0.075*beta

		    if v >= 0.2 :
		        v = 0.2
		    elif v <= -0.2 :
		        v = -0.2

		    if w >= 2.5 :
		        w = 2.5
		    elif w <= -2.5 :
		        w = -2.5

		    if -0.05 <= phi and phi <= 0.05 :
		        v = 0.2
		        w = 0

		    vel_msg.linear.x = v
		    vel_msg.linear.y = 0
		    vel_msg.linear.z = 0

		    #angular velocity in the z-axis:
		    vel_msg.angular.x = 0
		    vel_msg.angular.y = 0
		    vel_msg.angular.z = w


		    #Publishing our vel_msg
		    #print self.pose.x
		    #print self.pose.y
		    #print r
		    self.velocity_publisher.publish(vel_msg)
		    self.rate.sleep()
		#Stopping our robot after the movement is over
		vel_msg.linear.x = 0
		vel_msg.angular.z =0
		self.velocity_publisher.publish(vel_msg)
		
		"""
		self.lidar_flag = 1
		
		tic = time.time()
		toc = time.time()
		tt=toc-tic
		while tt <= 1.5:
                    toc = time.time()
		    tt = toc - tic

		self.lidar_flag = 0
		""" 	
    
    def patrol(self):
        asdf = 0
        GPIO.output(7,False) 
	"""    
        print "patrol"
        print "current poseition",
        print self.pose.x,
        print self.pose.y
        start = (int(self.pose.x/0.04), int(self.pose.y/0.04))
        goal = self.path[0]
        draw = 0
        print "goal ----> ",
        print goal
        print "start ----> ",
        print start
	patrol_path = acp.compath(start, goal, draw)
	tx = goal[0]
	ty = goal[1]
	tx = tx*0.04
	ty = ty*0.04
	print "patrol_path ------>",
	print patrol_path
	if patrol_path[0] == (tx,ty) :
            self.path.remove(self.path[0])
        
        if sqrt(pow((patrol_path[0][0] - self.pose.x), 2) + pow((patrol_path[0][1] - self.pose.y), 2)) < 0.01 :
            patrol_path.remove(patrol_path[0])
        
	go2goal(patrol_path,self)
        """

    def tracking(self):
        print "tracking"
        GPIO.output(7,True)
        vel_msg = Twist()
	rospy.sleep(0.1)

	self.tic = time.time()
	#print('tracking')

	self.Old_Rx = self.Rx
	self.Old_Ry = self.Ry

	start = (int(self.pose.x/0.04), int(self.pose.y/0.04))
	#goal = (int(self.Rx_i), int(self.Ry_i))
	goal = (int(self.final_x), int(self.final_y))
	draw = 0
	print "goal : ",
	print goal
	print "self.Rx_i ===> ",
	print self.Rx_i
	print "(int)self.Rx_i ====>",
	print int(self.Rx_i)
	print "self.Rx ===> ",
	print self.Rx
	tracking_path = acp.compath(start, goal, draw)
	print "tracking_path ----->",
	print tracking_path
	go2goal(tracking_path,self)
	
	ticc = time.time()
	tocc = time.time()
	ttt=tocc-ticc

	while ttt <= 1.5:
	        tocc = time.time()
		ttt = tocc - ticc

	self.lidar_flag = 1
	tic = time.time()
	toc = time.time()
	tt=toc-tic
	while tt <= 2.5:
	        toc = time.time()
		tt = toc - tic
	self.lidar_flag = 0


def go2goal(goal,x):
    for i in range(len(goal)):
        x.move2goal(goal[i])
    print('finish')
    
if __name__ == '__main__':
    x = turtlebot()
    rospy.sleep(0.5)

    while 1:
        try:
            #Testing our function
            if x.tracking_on == 1:
                x.tracking()
            else:
                x.tracking()
        except rospy.ROSInterruptException: pass
