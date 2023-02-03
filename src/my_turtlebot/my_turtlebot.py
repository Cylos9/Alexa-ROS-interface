#!/usr/bin/env python3
import threading
import rospy
import actionlib
import geometry_msgs.msg
import move_base_msgs.msg
import tf,tf_conversions
from nav_msgs.msg import Odometry
import sys

class MyTurtlebot:
    def __init__(self):
        rospy.init_node('alexa_interface', disable_signals=True)
        
        #create a simpleActionClient that request to move_bave action server on turtlebot3
        self._move_base_client = actionlib.SimpleActionClient("move_base",move_base_msgs.msg.MoveBaseAction)
        self._move_base_client.wait_for_server()
        
        #initial position
        self._position = [0,0]
        #create Subscriber to get current position
        rospy.Subscriber("odom",Odometry,self._odom_callback)
        
        #create a list of pre-defined location 
        self._locations = self._createLocationList()

    def moveTo(self,location_cmd):
        
        try:
            goal = self._locations[location_cmd]
        except KeyError:
            error_msg= "the location of the {} is not registed yet!".format(location_cmd)
            print(error_msg)
            
            return error_msg
        
        self._move_base_client.send_goal(goal)
        
        success_msg = "I'm heading over to the {}!".format(location_cmd)
        
        return success_msg

    def stop(self):
        self._move_base_client.cancel_goal()
        
        return 1
    
    def getCurrentPosition(self):
        
        return self._position
    
    def _createGoalMessage(self,position,orientation):
        goal_msg = move_base_msgs.msg.MoveBaseGoal()
        
        #header
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = "map"
        
        #pose
        goal_msg.target_pose.pose.position.x = position[0]
        goal_msg.target_pose.pose.position.y = position[1]
        
        goal_msg.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2]))
        
        return goal_msg

    def _createLocationList(self):
        #get locations from Parameter server
        locations_dicts = rospy.get_param("/house_locations")
        
        #get and remove frame_id from locations_dicts
        frame_id = locations_dicts.pop("frame_id")
        
        if frame_id == "map":
            locations_keys = list(locations_dicts.keys())
            
            locations_for_cmd = {}
            
            #create a location list with move_base_msgs/MoveBaseGoal
            for location_key in locations_keys:
                location = locations_dicts[location_key]
                locations_for_cmd[location["name"]] = self._createGoalMessage(location["position"],location["orientation"])
        
        else:
            rospy.logfatal("Your locations list was loaded with frame_id ='{}' ! Please change to 'odom' frame!".format(frame_id))
            rospy.logfatal("Node is forcelly terminated!")
            # force program to be terminated
            rospy.signal_shutdown("Wrong frame_id !")
            
            return 0
                   
        return locations_for_cmd

    def _odom_callback(self,data):
        
        self._position[0] = data.pose.pose.position.x
        self._position[1] = data.pose.pose.position.y
        
        return 1
