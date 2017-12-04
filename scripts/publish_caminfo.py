#!/usr/bin/env python  
import roslib
import rospy

from sensor_msgs.msg import CameraInfo

import math

def talker():
    cam = CameraInfo()
    cam.width = 1
    cam.height = 1
    cam.header.frame_id = "/camera"
    cam.distortion_model = "plumb_bob"
    rospy.init_node('publish_caminfo', anonymous=True)
    pub = rospy.Publisher('camera_info', CameraInfo)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        f = math.sin(rospy.get_time() / (2* math.pi) * 600)
        f *=f
        cam.K[0] = f
        cam.K[4] = f
        cam.header.stamp=rospy.Time.now()

        pub.publish(cam)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass