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
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Point, Pose

pi = np.pi
b = (0.45/2) #robot width
rospy.init_node('apriltag_navi', anonymous=True)
lr = tf.TransformListener()
br = tf.TransformBroadcaster()
tag_id = None
tag_scale = 3/7
step = 0
delta_ready = False
    
def main():
    apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, apriltag_callback, queue_size = 1)
    odom_sub = rospy.Subscriber('/odom', Pose, odom_callback, queue_size = 1)
    distance_sub = rospy.Subscriber('/distance', Float32, distance_callback, queue_size = 1)
    object_sub = rospy.Subscriber('/object_position', Point, object_callback, queue_size = 10)
    step_sub = rospy.Subscriber('/delta_moved_pizza', Bool, delta_ready_callback)
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
    global tag_id
    # use apriltag pose detection to find where is the robot
    for detection in data.detections:
        if detection.id == 2:   # pizza station tag id
            tag_id = 2
            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
            pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')
            
        elif detection.id == 3:   # final tag id
            tag_id = 3
            poselist_tag_cam = pose2poselist(detection.pose)
            poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
            poselist_base_tag = invPoselist(poselist_tag_base)
            poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
            pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')
            
        # elif detection.id == 0:   #  tag id
        #     tag_id = 0
        #     #print "Tag 0 detected"
        #     poselist_tag_cam = pose2poselist(detection.pose)
        #     poselist_tag_base = transformPose(lr, poselist_tag_cam, 'camera', 'robot_base')
        #     poselist_base_tag = invPoselist(poselist_tag_base)
        #     poselist_base_map = transformPose(lr, poselist_base_tag, 'apriltag', 'map')
        #     pubFrame(br, pose = poselist_base_map, frame_id = '/robot_base', parent_frame_id = '/map')

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
    global waiter_x 
    global waiter_y
    global waiter_z
    
    waiter_x = data.x
    waiter_y = data.y
    waiter_z = data.z
    #print 'Received object (x, y, z):', waiter_x, waiter_y, waiter_z


def delta_ready_callback(data):
    global delta_ready
    delta_ready = data.data


def navi_loop():
    rospy.loginfo("I am alive. Please feed me.")
    step5_tag3_detected = False
    rospy.on_shutdown(shutdown)
    velcmd_pub = rospy.Publisher("/cmdvel", WheelCmdVel, queue_size = 1)
    mobile_ready_pub = rospy.Publisher('/mobile_ready', Bool, queue_size = 1) #-AN publish to delta robot mobile ready = True/False
    rate = rospy.Rate(100) # 100hz
    
    wcv = WheelCmdVel()
    

    arrived = False
    arrived_position = False
    step = 1 #for testing without delta robot
    step_2_start = None
    step_3_case = 1
    is_waiter_here = [False, False, False] # <100, 100<x<200, 200<x<300
    step_4_start = None
    waiter_moving = False
    waiter_start_time = None
    waiter_vel = None
    step_5_case = 1
    tag_3_detected = False
    ref_dist_5 = pathDistance
    
    ref_theta_1 = robot_theta
    
    dist_to_table = pathDistance #for testing step 3
    
    while not rospy.is_shutdown() :
        rospy.loginfo("in main loop")
        #~ try:
            #~ #print robot_x, robot_y, robot_theta, 'robot'
            #~ #print waiter_x, waiter_y, waiter_z, 'object'
            #~ print pathDistance, 'distance'
        #~ except Exception as e:
            #print e
            #~ pass
        
        # Get robot pose
        robot_pose3d = lookupTransform(lr, '/map', '/robot_base')

        #go to pizza station
        if step == 1:
            target_pose2d = [0.15, 0, np.pi]
            print pathDistance
            if pathDistance < 1.2:
                wcv.desiredWV_R = 1.0  # right, left
                wcv.desiredWV_L = 1.0 
                print pathDistance 
                              
            else:
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
                
                pos_delta         = (np.array(target_position2d) - np.array(robot_position2d)) #scale distance based on tag size
                robot_heading_vec = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])
                heading_err_cross = cross2d( robot_heading_vec, pos_delta / np.linalg.norm(pos_delta) )
                print np.linalg.norm( pos_delta )
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
        
        #wait at pizza station until next step for at most 2 seconds
        if step == 2:
            if not step_2_start:
                step_2_start = time.time()
                
            if (time.time() < step_2_start + 2): #not delta_ready
                wcv.desiredWV_R = 0.0  
                wcv.desiredWV_L = 0.0
            else:
                print 'Done with step 2'
                step = 3
                step_3_case = 1
        
        #go to waiter using dead reckoning
        if step == 3:
            table_to_waiter = pathDistance - dist_to_table #initialize path distance to 0 at the table
            
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
                if abs(robot_theta - ref_theta_1) >= (pi/4 + (10*pi)/180):
                    step_3_case = 3
                
            #arc left (mainly forward)
            elif step_3_case == 3:
                print "Case 3.3:", (pathDistance - ref_dist)
                wcv.desiredWV_R = .115
                wcv.desiredWV_L = .1
                if (pathDistance - ref_dist) >= 1.5:
                    #ideal angle relative to tag 0 is -3.1
                    #ideal x relative to waiter is 230
                    step_3_case = 4

                
            #turn to face waiter (cv x between 0 & 590, y between 0 & 440, z in meters)
            elif step_3_case == 4:
                #print "Case 3.4 (Robot, Ref):", robot_theta, ref_theta_1
                #K = 1/b
                #wcv.desiredWV_R, wcv.desiredWV_L = get_desiredWV(0.05, K) #turn 45 degrees
                #old_x = waiter_x
                #print old_x
                #check position of the waiter
                try:
                    print waiter_x, waiter_y, waiter_z
                    if waiter_x < 100:
                        is_waiter_here[0] = True
                    elif 100 <= waiter_x < 170 and is_waiter_here[0]:
                        is_waiter_here[1] = True
                    elif 170 <= waiter_x < 230 and is_waiter_here[1]:
                        is_waiter_here[2] = True
                    elif 230 <= waiter_x and is_waiter_here[2]: #need to add case for if there's an old value stored
                            step_3_case = 5
                    
                    wcv.desiredWV_R = .1
                    wcv.desiredWV_L = -.1

                except:
                        wcv.desiredWV_R = .1
                        wcv.desiredWV_L = -.1
                        print "waiter not in view"
                
            #stop in front of waiter 
            elif step_3_case == 5:
                print "Case 3.5: Done with step 3"
                step = 4
                
            else:
                print "STEP 3 UNKNOWN CASE:", step_3_case 
            


        #wait in front of waiter until next step for at most 10 seconds
        start_time = time.time()
        if step == 4:
            #if not step_4_start:
            #    step_4_start = time.time()
            #    print step_4_start

            if 400 <= waiter_x <= 500 and not waiter_moving:
                print "Waiter Moving:", waiter_x
                waiter_moving = True
                waiter_start_time = time.time()

            elif waiter_x > 500 and not waiter_vel and waiter_moving:
                waiter_vel = 100/(time.time()-waiter_start_time)
                print "Waiter Vel Defined", waiter_vel

            elif type(waiter_vel) == float:
                print "Waiting for Waiter to clear:", waiter_x
                if (time.time() > waiter_start_time + 200/waiter_vel + 5):
                    print 'Done with step 4'
                    ref_dist_5 = pathDistance
                    step = 5
            else:
                print "Waiter Pos:", waiter_x

            wcv.desiredWV_R = 0.0  
            wcv.desiredWV_L = 0.0

        #navigate to end
        if step == 5:
            target_pose2d = [0, 0, np.pi]
            if not robot_pose3d: #turn left
                wcv.desiredWV_R = (0.1+.05) if step5_tag3_detected else 0.1
                wcv.desiredWV_L = (-0.1+.05) if step5_tag3_detected else 0.1 
                print 'Case 5.1 Tag not in view'
            
            else:
                robot_position2d  = robot_pose3d[0:2]
                target_position2d = target_pose2d[0:2]
                pos_delta = (np.array(target_position2d) - np.array(robot_position2d))
                print robot_pose3d, pos_delta
                
    
                if np.linalg.norm(pos_delta) > 0.1:
                    wcv.desiredWV_R = 0.2
                    wcv.desiredWV_L = 0.2
                    step5_tag3_detected = True
                    print 'Case 5.2 Moving to Tag'
    
                elif np.linalg.norm(pos_delta) < 0.1 and np.linalg.norm(pos_delta) != 0.0:
                    robot_yaw = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
                    if np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.05:
                        wcv.desiredWV_R = 0
                        wcv.desiredWV_L = 0
                        step = 6
                        print 'Case 5.3.0 Done with Step 5'
    
                    elif diffrad(robot_yaw, target_pose2d[2]) > 0:
                        print 'Case 5.3.1  Turn right slowly'      
                        wcv.desiredWV_R = -0.05 
                        wcv.desiredWV_L = 0.05
                    else:
                        print 'Case 5.3.2  Turn left slowly'
                        wcv.desiredWV_R = 0.05  
                        wcv.desiredWV_L = -0.05
                        print np.linalg.norm(pos_delta)


        if step == 6:
            target_pose2d = [0.45, 0, np.pi]
            
            if not (robot_pose3d and tag_id):
                wcv.desiredWV_R = 0.0
                wcv.desiredWV_L = 0.0
                print 'Case 6.1 Tag not in view'
                
            else:
                
                robot_position2d  = robot_pose3d[0:2]
                target_position2d = target_pose2d[0:2]
                pos_delta = (np.array(target_position2d) - np.array(robot_position2d))
                robot_yaw = tfm.euler_from_quaternion(robot_pose3d[3:7]) [2]
    
                if np.linalg.norm( pos_delta ) < .05:
                    print 'Done with step 6'
                    #final stop
                    wcv.desiredWV_R = 0.0
                    wcv.desiredWV_L = 0.0
                    velcmd_pub.publish(wcv)
                    quit()
                
                elif np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.1:
                    print 'Case 6.3  Straight backward'  
                    wcv.desiredWV_R = -0.2
                    wcv.desiredWV_L = -0.2
                    
                elif np.linalg.norm( pos_delta ) > .05 and np.fabs(diffrad(robot_yaw, target_pose2d[2]))<0.1:
                    if heading_err_cross < 0:
                        print 'Case 6.4.1  Turn right'
                        wcv.desiredWV_R = -0.1
                        wcv.desiredWV_L = 0.1
                    else:
                        print 'Case 6.4.2  Turn left'
                        wcv.desiredWV_R = 0.1
                        wcv.desiredWV_L = -0.1

        mobile_ready_pub.publish(Bool(arrived)) #publish if mobile has arrived
        #print 'Publishing Velocity:', wcv.desiredWV_R, wcv.desiredWV_L
        velcmd_pub.publish(wcv)
        rate.sleep()

if __name__=='__main__':
    main()
    
