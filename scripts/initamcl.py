#!/usr/bin/env python


import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import PoseWithCovariance
import tf
from std_srvs.srv import Empty


done = False


def has_received_odom(pub):
    def cb(msg):
        global done
        tf_tl = tf.TransformListener()
        c = msg.pose.covariance
        print msg
        if not(c[0] < 16 and c[7] < 16 and c[35] < 0.5):
            return

        try:
            tf_tl.waitForTransform("slam_map", "map", rospy.Time(0),
                                   rospy.Duration(10.0))
        except tf.Exception as e:
            rospy.logwarn(e)
            return

        try:
            p0 = PoseStamped(msg.header, msg.pose.pose)
            p0.header.stamp = rospy.Time(0)
            p1 = tf_tl.transformPose('slam_map', p0)
            p1.pose.position.z = 0
            p2 = PoseWithCovarianceStamped(
                p1.header, PoseWithCovariance(p1.pose, msg.pose.covariance))
            pub.publish(p2)
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logwarn(e)
            return
        done = True
    return cb


def main():
    rospy.init_node('initamcl', anonymous=True)
    rospy.wait_for_service('/global_localization')
    gl = rospy.ServiceProxy('/global_localization', Empty)
    gl()
    rospy.loginfo("AMCL is active, wait for gps message")
    pub = rospy.Publisher('/amcl/initialpose', PoseWithCovarianceStamped,
                          queue_size=1)
    rospy.Subscriber('/odometry/filtered', Odometry, has_received_odom(pub))
    while not rospy.is_shutdown() and not done:
        rospy.sleep(1)
    rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
