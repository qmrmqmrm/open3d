#!/home/j/.pyenv/versions/ros_py36/bin/python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import open3d

def main():
    rospy.init_node("cv2_to_imgmsg")
    rate = rospy.Rate(100)
    count = 0
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()
    pub_right = rospy.Publisher('/image_right', Image, queue_size=1)
    while not rospy.is_shutdown():
        _, frame = cap.read()
        # cv2.imshow("frame",frame)
        img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
        pub_right.publish(img_msg)
        # cv2.waitKey(1)
        print(f"{+count}")

        rate.sleep()
        
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass