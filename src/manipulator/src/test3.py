#!/usr/bin/env python
# -- coding: utf-8 --


import rospy
from geometry_msgs.msg import Point


def main():
    rospy.init_node('targettest')
    pos_pub = rospy.Publisher('target_position',Point,queue_size=1)

    a = []
    while not rospy.is_shutdown():
        a = list(map(float, input().split(' ')))
        msg = Point()
        msg.x = a[0]
        msg.y = a[1]
        msg.z = a[2]
        pos_pub.publish(msg)
    



if __name__ == '__main__':
    main()
