#!/home/j/.pyenv/versions/ros_py36/bin/python3

import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import open3d

def callback(img_msg):
    bridge = CvBridge()
    frame_ = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    print(img_msg.header)
    cv2.imshow("fram",frame_)
    key = cv2.waitKey(1)
    
def main():
    rospy.init_node("imgmsg_to_cv2")
    pub_right = rospy.Subscriber('/re_img', Image,callback)
    rospy.spin()
        
if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass