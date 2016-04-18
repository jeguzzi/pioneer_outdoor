#!/usr/bin/env python

import pyproj

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, NavSatFix
from math import asin, sin, cos
import std_srvs.srv

MANUAL=0
AUTO=1

YELLOW_BUTTON=3
RED_BUTTON=1
GREEN_BUTTON=0
BLUE_BUTTON=2
BACK_BUTTON=6
START_BUTTON=5 #TODO: verify

class Controller(object):

    # utm 32N: EPSG:32632
    # utm 31N: EPSG:32631

    def utm_pose(lat, lon):
        #WGS84
        p1 = pyproj.Proj(init='epsg:4326')
        p2 = pyproj.Proj(init=self.utm_proj)
        x, y = pyproj.transform(p1, p2, lon, lat)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "utm"
        msg.pose.position.x = x
        msg.pose.position.y = y
        return msg

    def has_received_gps_wp(self, msg):
        if self.state == AUTO:
            lat = msg.latitude
            lon = msg.longitude
            pose = self.utm_pose(lat, lon)
            self.move_base_wp.publish(pose)
            rospy.loginfo("Send new waypoint to move_base %s", pose)

    def has_received_odometry(self, msg):
        self.pose = PoseStamped()
        self.pose.pose = msg.pose.pose
        self.pose.header = msg.header
    
    def set_home(self, pose):
        if pose:
            p = pose.pose.position
            o = pose.pose.orientation
            a = asin(o.z)*2
            self.set_param("~home",[p.x. p.y, a, pose.header.frame_id])

    def home(self):
        try:
            x, y, a, f = rospy.get_param('~home')
            msg = PoseStamped()
            msg.header.frame_id = f
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.orientation.z = sin(a*0.5)
            msg.pose.orientation.w = cos(a*0.5)
        except:
            return None
                

    def __init__(self):

        rospy.init_node('controller_node', anonymous=False)
        # rospy.on_shutdown(self.stop)
                   
        self.pose = None
        rospy.Subscriber("~odometry", Odometry, self.has_received_odometry)
                
        #read list of poses from parameters:

        utm_proj = rospy.get_param("~utm","epsg:32632")

        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #Publisher to move_base target point. We don't need to use the action interface because we are not tracking 
        self.move_base_wp = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1)
              
        #Subscribe to gps topic containing a target datum
        rospy.Subscriber('~waypoint', NavSatFix, self.has_received_gps_wp)

        #Subscribe to the joystick
        rospy.Subscriber('joy', Joy, self.has_received_joy)

        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))        
        rospy.loginfo("Connected to move base server")

        self.state=AUTO
        self.clear_costmaps = rospy.ServiceProxy('move_base/clear_costmaps', std_srvs.srv.Empty)
        rospy.spin()
        
    @property
    def state(self):
        return self._state

    @state.setter
    def state(self,value):
        if value == MANUAL and self._state == AUTO:
            self.invalidate_wp()
        self._state=value
        
    def invalidate_wp(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.move_base.cancel_goal()
        rospy.sleep(1)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo('DONE: implement stop')

    def go_home(self):
        self.state = AUTO
        wp = self.home()
        if wp:
            rospy.loginfo("Going HOME: %s", wp)
            self.send_wp(self.home_wp)
        else:
            rospy.warn("HOME undefined")
        
    def has_received_joy(self,msg):
        rospy.loginfo('DONE: Verify, has received msg.buttons %s', msg.buttons)
        if msg.buttons[RED_BUTTON] is 1:
            self.state = MANUAL
        
        if msg.buttons[GREEN_BUTTON] is 1:
            self.state = AUTO

        if msg.buttons[BLUE_BUTTON] is 1:
            pass

        if msg.buttons[YELLOW_BUTTON] is 1:
            self.go_home() 

        if msg.buttons[BACK_BUTTON] is 1:
            self.clear_costmaps() 
        
        if msg.buttons[START_BUTTON] is 1:
            self.set_home(self.pose)
            

if __name__ == '__main__':
    try:
        Controller()
    except rospy.ROSInterruptException:
        rospy.loginfo("Controller finished.")
