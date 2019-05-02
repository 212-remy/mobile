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
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Pose

pi = np.pi
b = (0.45/2) #robot width
rospy.init_node('apriltag_navi', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()
    
def main():
    apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    #step_sub = rospy.Subscriber('/mobile_step', int32, step_callback, queue_size = 1)
    odom_sub = rospy.Subscriber('/odom', Pose, odom_callback, queue_size = 1)
    distance_sub = rospy.Subscriber('/distance', Float32, distance_callback, queue_size = 1)
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
        wcv.desiredWV_R = .1
        wcv.desiredWV_L = .1
        
        velcmd_pub.publish(wcv) 
        print wcv.desiredWV_R, wcv.desiredWV_L
        rate.sleep()
        
def shutdown():
    wcv.desiredWV_R = 0.0  
    wcv.desiredWV_L = 0.0
    velcmd_pub.publish(wcv)

## computing desired wheel velocity based on curvature and robot velocity - ANANYA        
def get_desiredWV(robot_velocity, path_curvature):
    desiredWV_L = robot_velocity - path_curvature*b*robot_velocity #b is a known constant NEED TO CHANGE, equations are from Lab 2
    desiredWV_R = 2*robot_velocity - desiredWV_L #positive path curvature: turn left, negative path curvature: turn right
    return desiredWV_R, desiredWV_L

## apriltag msg handling function
def apriltag_callback(data):
    # use apriltag pose detection to find where is the robot
    for detection in data.detections:
        if detection.id == 0:   # tag id is the correct one
            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
            pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')

def odom_callback(data):
    global robot_x 
    global robot_y
    global robot_theta
    
    robot_x = data.position.x
    robot_y = data.position.y
    quaternion = (data.orientation.x,
                  data.orientation.y,
                  data.orientation.z,
                  data.orientation.w)
    roll, pitch, yaw = tfm.euler_from_quaternion(quaternion)
    robot_theta = yaw
    #print 'Received robot (x, y, theta):', robot_x, robot_y, robot_theta

def distance_callback(data):
    global pathDistance
    pathDistance = data.data

def object_callback(data):
    global obj_x 
    global obj_y
    global obj_z
    
    obj_x = data.x
    obj_y = data.y
    obj_z = data.z
    #print 'Received object (x, y, z):', obj_x, obj_y, obj_z


def step_callback(data):
    global step
    step = data.data
    #print 'Received step:', step


def navi_loop():
    rospy.on_shutdown(shutdown)
    velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)
    target_pose2d = [0.25, 0, np.pi]
    rate = rospy.Rate(100) # 100hz
    
    wcv = WheelCmdVel()
    
    arrived = False
    arrived_position = False
    step = 3 #for testing without delta robot
    step_3_case = 4
    step_2_start = None
    step_4_start = None
    
    ref_theta_1 = robot_theta
    
    while not rospy.is_shutdown() :
        #~ try:
            #~ #print robot_x, robot_y, robot_theta, 'robot'
            #~ #print obj_x, obj_y, obj_z, 'object'
            #~ print pathDistance, 'distance'
        #~ except Exception as e:
            #~ pass
        
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
                print 'Done with step 1'
                step = 2
                dist_to_table = pathDistance #save pathDistance from start to table - AN
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
        
        #wait at pizza station until next step for at most 10 seconds
        if step == 2:
            if not step_2_start:
                step_2_start = time.time()
                
            if (time.time() < step_2_start + 10):
                wcv.desiredWV_R = 0.0  
                wcv.desiredWV_L = 0.0
            else:
                print 'Done with step 2'
                step = 3
        
        #go to waiter using dead reckoning
        if step == 3:
            #table_to_waiter = pathDistance - dist_to_table #initialize path distance to 0 at the table
            
            #back up
            if step_3_case == 1:  
                print "Case 3.1:", table_to_waiter
                K = 0
                wcv.desiredWV_R, wcv.desiredWV_L = get_desiredWV(-0.1, K)
                ref_theta_1 = robot_theta
                if (table_to_waiter >= 0.5):
                    step_3_case = 2
                
            #turn left
            elif step_3_case == 2:
                print "Case 3.2:", abs(robot_theta - ref_theta_1)
                wcv.desiredWV_R = .1
                wcv.desiredWV_L = -.1
                ref_dist = pathDistance
                if abs(robot_theta - ref_theta_1) >= pi/4:
                    step_3_case = 3
                
            #arc left (mainly forward)
            elif step_3_case == 3:
                print "Case 3.3:", (pathDistance - ref_dist)
                wcv.desiredWV_R = .11
                wcv.desiredWV_L = .1
                if (pathDistance - ref_dist) >= 1.3:
                    step_3_case = 4
                
            #turn to face waiter
            elif step_3_case == 4:
                print "Case 3.4 (Robot, Ref):", robot_theta, ref_theta_1
                #K = 1/b
                #wcv.desiredWV_R, wcv.desiredWV_L = get_desiredWV(0.05, K) #turn 45 degrees
                wcv.desiredWV_R = .1
                wcv.desiredWV_L = -.1
                if (robot_theta - ref_theta_1) >= pi:
                    step_3_case = 5
                
            #stop in front of waiter (cv x between 0 & 590, y between 0 & 440) 
            elif step_3_case == 5:
                print "Case 3.5: Done with step 3"
                wcv.desiredWV_R = 0.0
                wcv.desiredWV_L = 0.0
                step = 4
            
            else:
                print "STEP 3 UNKNOWN CASE:", step_3_case 
            
        #wait in front of waiter until next step for at most 10 seconds
        start_time = time.time()
        if step == 4:
            if not step_4_start:
                step_4_start = time.time()
                
            if (time.time() < step_4_start + 10):
                wcv.desiredWV_R = 0.0  
                wcv.desiredWV_L = 0.0
            else:
                print 'Done with step 4'
                step = 5
            
        #navigate to end
        if step == 5:
            print 'Done with step 5'
            
            #dead reckoning
            #if (april tag not in view):
            #   slight curve left
            #else:
            #   turn right, go forward
            #final stop
            wcv.desiredWV_R = 0.0  
            wcv.desiredWV_L = 0.0
            velcmd_pub.publish(wcv)
            quit()
            
        #print 'Publishing Velocity:', wcv.desiredWV_R, wcv.desiredWV_L
        velcmd_pub.publish(wcv)  
        
        rate.sleep()

if __name__=='__main__':
    main()
    
