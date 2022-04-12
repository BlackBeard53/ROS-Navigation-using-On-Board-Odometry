#!/usr/bin/env python3

import sys
import math
import rospy
import numpy as np
from geometry_msgs.msg import Point, Vector3, Twist
from nav_msgs.msg import Odometry

class goToGoal:
    def __init__(self):
        print("---------------------------------")
        print('Starting Node: Navigate_to_Goal')
        print("---------------------------------")
        self.Init_pos = Point()
        self.globalPos = Point()
        self.Init = True
        self.odom_update = False
        self.goals = [(1.65, 0), (1.6, 1.6), (-0.10 , 1.4)]
        self.current = 0
        self.goal = self.goals[self.current]
        self.threshold = 0.05
        self.odom_sub = rospy.Subscriber("/odom", Odometry, queue_size=1, callback=self.callback)
        self.range_sub = rospy.Subscriber("getObjectRange/Point", Point, queue_size=1, callback=self.range_callback)
        self.publish = True
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.start = rospy.get_time()

    def callback(self, msg):
        self.updateOdometry(msg)
        self.odom_update = True
        return
    
    def range_callback(self, msg):
        thresholdMod = 0.12
        if self.odom_update:
            orientation = self.globalAng
            relX = self.goal[0] - self.globalPos.x
            relY = self.goal[1] - self.globalPos.y
            relDist = math.sqrt(relX**2 + relY**2)
            print("Distance to Goal Point: {}".format(relDist))
            angleToGoal = np.arctan2(relY,relX)
            angleToGoal -= orientation
            while angleToGoal > math.pi:
                angleToGoal -= 2*math.pi
            while angleToGoal < -math.pi:
                angleToGoal += 2*math.pi

            print('Global Position: {}'.format(orientation))
            print("Angle to Goal Point: {}".format(angleToGoal))
            # angleToGoal += math.pi/2

            out = Twist()
            lv = Vector3()
            av = Vector3()

            #goal state switching
            if(relDist < self.threshold):
                print("-------------------------------------------------") 
                print("Goal Reached: {}".format(self.goal))       
                # stop at the goal for 10 seconds and change the goal to next point if the goal is reached
                lv.x = 0
                av.z = 0
                out.angular = av
                out.linear = lv
                time = rospy.get_time() - self.start
                mins = time//60
                sec = time%60
                print("Time taken: {}:{}".format(int(mins), sec))
                print("-------------------------------------------------") 
                if self.publish:
                    self.vel_pub.publish(out)
                if self.current < len(self.goals) - 1:
                    self.threshold += thresholdMod
                    self.current += 1
                    self.goal = self.goals[self.current]
                rospy.sleep(10)

            print('Presence of Obstacle: {}'.format(abs(angleToGoal - msg.y) < 0.2))
            print('Obstacle Distance: {}'.format(msg.x))
            print('___________________________________________________')

            if(abs(angleToGoal - msg.y) < math.pi/3 and msg.x <= 0.3): # condition for wallfolllow or gtg
                print('********************************')
                print('Following along the Wall Edge')
                print('********************************')
                print('Orientation to object: {}'.format((msg.y-math.pi/2)))
                objectAng = msg.y - math.pi/2

                while objectAng > math.pi:
                    objectAng -= 2*math.pi
                while objectAng < -math.pi:
                    objectAng += 2*math.pi

                if (msg.x < 0.1):
                    lv.x = -0.22
                    av.z = 0
                # Check if bot is facing the object or not
                elif(abs(objectAng)<0.2): 
                    lv.x = 0.6
                    av.z = 0
                else:
                    # check proper sign
                    av.z = (objectAng)*(0.5) 
                    lv.x =0
                
                out.angular = av
                out.linear = lv
                print('Linear: {}'.format(out.linear.x))
                print('Angular: {}'.format(out.angular.z)) 
                if self.publish:
                    self.vel_pub.publish(out) 
            else:
                print('=============================')
                print('Going to Goal: {}'.format(self.goal))
                print('=============================')

                # check if bot is properly oriented towards the goal or not
                if(abs(angleToGoal)>0.15):  
                    # angular velocity to orient
                    lv.x = 0
                    av.z = (angleToGoal)*(0.5) 
                else:
                    # linear velocity to move towards the goal
                    lv.x = 0.6
                    av.z = (angleToGoal)*(0.2) 
                
                out.angular = av
                out.linear = lv
                print('Linear Velocity: {}'.format(out.linear.x))
                print('Angular Velocity: {}'.format(out.angular.z)) 
                if self.publish:
                    self.vel_pub.publish(out)
            self.odom_update = False
        return

    def updateOdometry(self, Odom):
        position = Odom.pose.pose.position
        #Orientation uses the quaternion aparametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            #print("Odom Reset")
            #print("===============================")
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            # These 4 were initially just Init_ang, not self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        

        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
    
def angleWrap(ang):

    while ang > math.pi:
        ang -= 2*math.pi
    while ang < -math.pi:
        ang += 2*math.pi

def main(args):
    rospy.init_node('Navigate_to_Goal', anonymous=True)
    ic = goToGoal()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS")

if __name__=="__main__":
    main(sys.argv)
