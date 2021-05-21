#!/home/j/.pyenv/versions/ros_py36/bin/python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
import tf

def main():
    rospy.init_node("draw_cam")
    rate =rospy.Rate(100)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        
        print("test")
        ox, oy, oz, ow = tf.transformations.quaternion_from_euler(0,0, 0)
        br.sendTransform((0,0,0),
                    (ox, oy, oz, ow),
                    rospy.Time.now(),
                    "velodyne",
                    "base_footprint")
        rate.sleep()
        ox, oy, oz, ow = tf.transformations.quaternion_from_euler(0,0, 0)
        br.sendTransform((0,0,0),
                    (ox, oy, oz, ow),
                    rospy.Time.now(),
                    "stereo_camera",
                    "base_footprint")
        rate.sleep()

        ox, oy, oz, ow = tf.transformations.quaternion_from_euler(0,0, 0)
        br.sendTransform((0,0,0),
                    (ox, oy, oz, ow),
                    rospy.Time.now(),
                    "base_footprint",
                    "map")
        rate.sleep()


        
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
