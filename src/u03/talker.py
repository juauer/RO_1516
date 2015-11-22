#!/usr/bin/env python
import rospy
import random
import time
from std_msgs.msg import UInt32

pub = 0
i   = 0

def callback(data):
    global pub
    global i

    if i > 8:
        rospy.loginfo("I heard %d. Now i'm retiring.", data.data)
	return

    i += 1
    r  = random.randint(1, 100)
    d  = data.data + r
    rospy.loginfo("I heard %d and add %d (result: %d)", data.data, r, d)
    pub.publish(d)

def talker():
    global pub
    pub = rospy.Publisher('pub_tal', UInt32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber("pub_lis", UInt32, callback)
    time.sleep(1) # the callback thread needs some time to show up
    pub.publish(random.randint(1, 100))
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
