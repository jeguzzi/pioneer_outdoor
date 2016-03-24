#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Imu


def has_received_imu(pub):
    def cb(msg):
        msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 1e-3]
        msg.orientation_covariance = [0.03, 0, 0, 0, 0.03, 0, 0, 0, 0.03]
        pub.publish(msg)
    return cb


def main():
    rospy.init_node('imu_republisher', anonymous=True)
    pub = rospy.Publisher('out_imu', Imu, queue_size=1)
    rospy.Subscriber('in_imu', Imu, has_received_imu(pub))
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
