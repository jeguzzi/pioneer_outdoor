#!/usr/bin/env python                                                                                                                                                                                   
import rospy
from sensor_msgs.msg import NavSatFix
import lcm

from pioneer_outdoor.pose_gps_t import pose_t
from pioneer_outdoor.plan_waypoint_t import plan_waypoint_t



class LcmBridge(object):
    
    def __init__(self, robot_id, lcm_addr="udpm://239.255.76.67:7667?ttl=1"):
        self.robot_id=robot_id
        self.waypoint_pub = rospy.Publisher('waypoint', NavSattFix, queue_size=1)
        rospy.Subscriber('fix', NaVSatFix, self.has_received_fix)

    def __enter__(self):
        self.lc = lcm.LCM(lcm_addr)
        self.subscription = self.lc.subscribe("RNPPOS", self.has_received_lcm)

    def __exit__(self, exc_type, exc_value, traceback):
        self.lc.unsubscribe(self.subscription)

    def publish_lcm_fix(self, lat, lon, alt):
        msg = pose_t()
        channel = "POSE"
        msg.robotid = robotid
        msg.position =  [lon, lat, alt]
        msg.orientation = [0,0,0,0]
        self.lc.publish(channel, msg.encode())

    def has_received_lcm(self, channel, data):
        msg = plan_waypoint_t.decode(data)        
        lat, lon =  msg.latitude, msg.longitude
        self.publish_waypoint(lat, lon)

    def has_received_fix(self, msg):
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude
        self.publish_lcm_fix(lat, lon, alt)

    def publish_waypoint(lat, lon, waypoint_pub):
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.latitude = lat
        mag.longitude = lon
        self.waypoint.pub(msg)

    def run(self):
        while not rospy.is_shutdown():
            lc.handle()


if __name__ == '__main__':
    rospy.init_node('lcm_bridge', anonymous=True)
    robot_id = rospy.get_param('id')
    with LcmBridge(robot_id) as bridge:
        try:
            bridge.run()
        except rospy.ROSInterruptException:
            pass

