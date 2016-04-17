#!/usr/bin/env python


import rospy
from sensor_msgs.msg import Imu


def has_received_imu(pub, yaw_cov, ang_speed_cov):
    def cb(msg):
        msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, ang_speed_cov]
        msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, yaw_cov]
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu"
        pub.publish(msg)
    return cb


def main():
    rospy.init_node('imu_republisher', anonymous=True)
    pub = rospy.Publisher('out_imu', Imu, queue_size=1)

    yaw_cov = rospy.get_param("angle_cov", 0.03)
    ang_speed_cov = rospy.get_param("angular_speed_cov", 5e-4)

    rospy.Subscriber('in_imu', Imu,
                     has_received_imu(pub, yaw_cov, ang_speed_cov))
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
