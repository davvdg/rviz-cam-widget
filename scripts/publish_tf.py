#!/usr/bin/env python  
import roslib
import rospy

import tf

import math

def talker():
    br = tf.TransformBroadcaster()
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        t = (rospy.get_time() / (2* math.pi)) * 600
        print t

        br.sendTransform((0,0,0),
            tf.transformations.quaternion_from_euler(0, 0, t),
            rospy.Time.now(),
            "map",
            "roundabout")
        br.sendTransform((0,1,0),
            tf.transformations.quaternion_from_euler(0, math.pi/2, 0),
            rospy.Time.now(),
            "roundabout",
            "/camera")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass