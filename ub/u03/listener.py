#!/usr/bin/env python
import rospy
import random
from std_msgs.msg import UInt32

pub = 0

def callback(data):
    global pub
    r = random.randint(1, 100)
    d = data.data + r
    rospy.loginfo("I heard %d and add %d (result: %d)", data.data, r, d)
    pub.publish(d)

def listener():
    global pub
    pub = rospy.Publisher('pub_lis', UInt32, queue_size=10)
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("pub_tal", UInt32, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
