#!/usr/bin/env python3

import sys
import rospy
import message_filters
import math
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

class getObjectRange:
    def __init__(self):
        self.range_sub = rospy.Subscriber("/scan", LaserScan, queue_size=1, callback=self.callback)

        self.pub = rospy.Publisher("getObjectRange/Point", Point, queue_size=1)

    def callback(self, scan_msg):
        print("RANGEFINDER CALLBACK")
        out = Point()


        closest = float('inf')
        angle = 0
        for i in range(len(scan_msg.ranges)):
            if scan_msg.ranges[i] < closest and scan_msg.ranges[i] != 0:
                closest = scan_msg.ranges[i]
                ndx = i
                angle = scan_msg.angle_min + i*scan_msg.angle_increment

        if angle is not None:
            while angle > math.pi:
                angle -= 2*math.pi
            while angle < -math.pi:
                angle += 2*math.pi
                
            out.x = closest
            out.y = angle
            print("========================")
            print(out)
            print("========================")
            self.pub.publish(out)

        # if ndx is not None:
        #     avg = closest
        #     for i in range(4):
        #         avg = (avg + scan_msg.ranges[ndx + i] + scan_msg.ranges[ndx - i])
        #     avg = avg/7
        #     closest = avg
        return
    # ======================================================

def main(args):
    rospy.init_node('getObjectRange', anonymous=True)
    ic = getObjectRange()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Coordinate Director")

if __name__=='__main__':
    main(sys.argv)
