#!/usr/bin/env python
import rospy
import numpy as np 
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import simple_navigation_goals


def callback(data):
    datax = []
    datay = []
    angle_min = data.angle_min
    angle_increment= data.angle_increment
    move = simple_navigation_goals.SimpleNavigationGoals()
    twist = Twist()
#    rospy.loginfo(data.ranges)
    rospy.loginfo(len(data.ranges))
    for i in range(len(data.ranges)):
        if np.isinf(data.ranges[i]):
            #deleted useles value  
            None

        else:  
            angle = angle_min + angle_increment * i
            x = data.ranges[i] * np.cos(angle)
            y = data.ranges[i] * np.sin(angle)
            if x > 0.05 and x < 1 and y > -0.25 and y < 0.25:
    	        datax.append(x)
    	        datay.append(y)   
    x_moy = 0
    y_moy = 0 
    for i in range(len(datax)):    
        #rospy.loginfo("x={}, y ={}".format(datax[i],datay[i])) 
        x_moy = x_moy + datax[i]
        y_moy = y_moy + datay[i]
    if len(datax)!= 0:
        x_moy = x_moy / len(datax)
        y_moy = y_moy / len(datay)
        rospy.loginfo("x={}, y ={}".format(x_moy,y_moy)) 
        #on veut ramener le baricentre vers la zone d'interet : x = 0.75 et y = 0
        erreur_x = x_moy - 0.15   # si erreur_x est positif on veut avancer et si erreur_x et negatif on veut reculer 
        erreur_y = y_moy - 0 # si erreur_y est et positif il faut tourner vers la gauche et si erreur_y et negatif il faut tourner vers la droite  
        k_l = 1
        k_r = 10
        #cmd_vel(erreur_x * k_l,erreur_y * k_r ) # to_do envoyer ces vitesses en s'inspirant du teleop_key

        twist.linear.x = erreur_x * k_l
        twist.linear.y = 0.0
        twist.linear.z = 0.0   
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = erreur_y * k_r
        pub.publish(twist)

    

    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    listener()
