#!/usr/bin/python

# 2.12 Lab 3 AprilTag Navigation: use AprilTag to get current robot (X,Y,Theta) in world frame, and to navigate to target (X,Y,Theta)
# Peter Yu Sept 2016

import rospy
import tf
import numpy as np
import threading
import serial
import tf.transformations as tfm
import time

from me212bot.msg import WheelCmdVel
from apriltags.msg import AprilTagDetections
from helper import transformPose, pubFrame, cross2d, lookupTransform, pose2poselist, invPoselist, diffrad
#from std_msgs.msg import int32
from geometry_msgs.msg import Point

rospy.init_node('apriltag_navi', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()
    
def main():
    apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    #step_sub = rospy.Subscriber('/mobile_step', int32, step_callback, queue_size = 1)
    object_sub = rospy.Subscriber('/object_position', Point, object_callback, queue_size = 10)
    rospy.sleep(1)
    
    constant_vel = False
    if constant_vel:
        thread = threading.Thread(target = constant_vel_loop)
    else:
        thread = threading.Thread(target = navi_loop)
    thread.start()
    
    rospy.spin()

## sending constant velocity (Need to modify for Task 1)
def constant_vel_loop():
    print 'using constant loop'
    velcmd_pub = rospy.Publisher('/cmdvel', WheelCmdVel, queue_size = 1)
    rate = rospy.Rate(100) # 100hz
    
    while not rospy.is_shutdown() :
        wcv = WheelCmdVel()
        wcv.desiredWV_R = 0
        wcv.desiredWV_L = 0
        
        velcmd_pub.publish(wcv) 
        print wcv.desiredWV_R, wcv.desiredWV_L
        rate.sleep() 

## apriltag msg handling function
def apriltag_callback(data):
    # use apriltag pose detection to find where is the robot
    for detection in data.detections:
        if detection.id == 1:   # tag id is the correct one
            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
            pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')

def object_callback(data):
    obj_x = data.x
    obj_y = data.y
    obj_z = data.z


def step_callback(data):
    step = data.data
    print step


## navigation control loop (No need to modify)
def navi_loop():
    velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)
    target_pose2d = [0.25, 0, np.pi]
    rate = rospy.Rate(100) # 100hz
    
    wcv = WheelCmdVel()
    
    arrived = False
    arrived_position = False
    step = 1 #for testing without delta robot
    
    while not rospy.is_shutdown() :
        # Get robot pose
        robot_pose3d = lookupTransform(lr, '/map', '/robot_base')

        #go to pizza station
        if step == 1:
            if robot_pose3d is None:
                print 'Case 1.0 Tag not in view, Stop'
                wcv.desiredWV_R = 0.0  # right, left
                wcv.desiredWV_L = 0.0
                velcmd_pub.publish(wcv)  
                rate.sleep()
                continue
            
            robot_position2d  = robot_pose3d[0:2]
            target_position2d = target_pose2d[0:2]
            
            robot_yaw    = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
            robot_pose2d = robot_position2d + [robot_yaw]
            
            pos_delta         = np.array(target_position2d) - np.array(robot_position2d)
            robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
            heading_err_cross = cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
        
            if arrived or (np.linalg.norm( pos_delta ) < 0.08 and np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.05) :
                print 'Case 1.1  Stop'
                wcv.desiredWV_R = 0.0  
                wcv.desiredWV_L = 0.0
                arrived = True
                #done with step 1
                step = 2
            elif np.linalg.norm( pos_delta ) < 0.08:
                arrived_position = True
                if diffrad(robot_yaw, target_pose2d[2]) > 0:
                    print 'Case 1.2.1  Turn right slowly'      
                    wcv.desiredWV_R = -0.05 
                    wcv.desiredWV_L = 0.05
                else:
                    print 'Case 1.2.2  Turn left slowly'
                    wcv.desiredWV_R = 0.05  
                    wcv.desiredWV_L = -0.05
            elif arrived_position or np.fabs( heading_err_cross ) < 0.2:
                print 'Case 1.3  Straight forward'  
                wcv.desiredWV_R = 0.1
                wcv.desiredWV_L = 0.1
            else:
                if heading_err_cross < 0:
                    print 'Case 1.4.1  Turn right'
                    wcv.desiredWV_R = -0.1
                    wcv.desiredWV_L = 0.1
                else:
                    print 'Case 1.4.2  Turn left'
                    wcv.desiredWV_R = 0.1
                    wcv.desiredWV_L = -0.1
        
        #wait at pizza station until next step for at most 2 minutes
        start_time = time.time()
        if step == 2 and (time.time()-start_time > 120):
            wcv.desiredWV_R = 0.0  
            wcv.desiredWV_L = 0.0
        else:
            step = 3
        
        #go to waiter
        if step == 3:
            pass
            #Case 3.1: waiter not in view
            #dead reckoning
            
            #Case 3.2: waiter in view (cv x between 0 & 590, y between 0 & 440)
            
        
        #wait in front of waiter until next step for at most 2 minutes
        start_time = time.time()
        if step == 4 and (time.time()-start_time > 120):
            wcv.desiredWV_R = 0.0  
            wcv.desiredWV_L = 0.0
        else:
            step = 5
            
        #navigate to end
        if step == 5:
            pass



        velcmd_pub.publish(wcv)  
        
        rate.sleep()

if __name__=='__main__':
    main()
    
