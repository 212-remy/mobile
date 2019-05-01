#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Peter Yu Sept 2016

import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion
from std_msgs.msg import Float32

import helper
from me212bot.msg import WheelCmdVel

serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)

## main function (Need to modify)
def main():
    rospy.init_node('me212bot', anonymous=True)
    
    ## 1. Initialize a subscriber
    rospy.Subscriber('/cmdvel', WheelCmdVel, cmdvel_callback)

    odometry_thread = threading.Thread(target = read_odometry_loop)
    odometry_thread.start()
    
    rospy.spin()


## msg handling function (Need to modify)
def cmdvel_callback(msg):  
    ## 2. Send msg.desiredWV_R and msg.desiredWV_L to Arduino.
    strCmd =  str(msg.desiredWV_R) + ',' + str(msg.desiredWV_L) + '\n'
    serialComm.write(strCmd)
    


# read_odometry_loop() is for reading odometry from Arduino and publish to rostopic. (No need to modify)
def read_odometry_loop():
    odom_pub = rospy.Publisher('/odom', Pose, queue_size = 1)
    distance_pub = rospy.Publisher('/distance', Float32, queue_size = 1)
    prevtime = rospy.Time.now()
    while not rospy.is_shutdown():
        # get a line of string that represent current odometry from serial
        serialData = serialComm.readline()
        
        # split the string e.g. "0.1,0.2,0.1" with commas
        splitData = serialData.split(',')
        
        # parse the 3 split strings into 3 floats
        try:
            x     = float(splitData[0])
            y     = float(splitData[1])
            pathDistance = float(splitData[2])
            theta = float(splitData[3])
            
            hz    = 1.0 / (rospy.Time.now().to_sec() - prevtime.to_sec())
            prevtime = rospy.Time.now()
            
            print 'x=', x, ' y=', y, ' theta =', theta, ' hz =', hz
            
            # publish odometry as Pose msg
            odom = Pose()
            odom.position.x = x
            odom.position.y = y
            qtuple = tfm.quaternion_from_euler(0, 0, theta)
            #print 'after quaternion conversion'
            odom.orientation = Quaternion(qtuple[0], qtuple[1], qtuple[2], qtuple[3])
            odom_pub.publish(odom)
            
            dist = Float32(pathDistance)
            distance_pub.publish(dist)
            
        except Exception as e:
            # print out msg if there is an error parsing a serial msg
            print 'Cannot parse', splitData
            print e
            

if __name__=='__main__':
    main()


