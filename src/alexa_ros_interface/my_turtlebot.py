#!/usr/bin/env python3
import rospy
import actionlib
import geometry_msgs.msg
import move_base_msgs.msg
import tf
import tf_conversions
from nav_msgs.msg import Odometry

# MyTurtlebot class definition
# Class to control the turtlebot movement and navigation
class MyTurtlebot:
    def __init__(self, disable_signals=False):
        
        # Initialization of class members    
        self._position = []
        self._location_list = {}
        self._area_list = {}
        self._goal_list = {}
        self._status_list = {
            "0": "PENDING",
            "1": "ACTIVE",
            "2": "PREEMPTED",
            "3": "SUCCEEDED",
            "4": "ABORTED",
            "5": "REJECTED",
            "6": "PREEMPTING",
            "7": "RECALLING",
            "8": "RECALLED",
            "9": "LOST"
        }
        self._last_goal = ""
        
        # Initialize ROS node
        # Disable the signals to stop the node from receiving external signals (for multithreading)
        rospy.init_node('alexa_interface', disable_signals=disable_signals)

        # Create a simple action client for move_base action server
        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", move_base_msgs.msg.MoveBaseAction)
        
        rospy.loginfo("Wating for Action Server start ...")
        self._move_base_client.wait_for_server() # Wait for the action server to start
        rospy.loginfo("Connected to Action Server !!!")

        # Create a tf listener to handle tf transformations
        self._tf_listener = tf.TransformListener()
        rospy.sleep(1) # Wait for the tf buffer to be filled, to avoid tf.ExtrapolationException

        # Create a Subscriber to receive the current position of the robot
        rospy.Subscriber("odom", Odometry, self._odom_callback)
    
    """
    PUBLIC FUNCTION
    """
    
    ## @brief Initialize the robot by loading parameters and create a goal list
    ##
    ## @param 
    ##
    ## @return False if failing to load parameter
    def init_robot(self):
        # Load map information from the parameter server
        success = self._load_map_info()
        if not success:
            rospy.logwarn("Fail to load map info !")
            return False
        else:
            # Create a list of goals for the robot to move to
            self._create_goal_list()
            return True
         
    ## @brief Send a goal to move_base action server
    ##
    ## @param location The location that we want robot move to
    ##
    ## @return True if success. False if the location is not registered
    def move_to(self, location):  
        # Try to get the goal for the specified location
        try:
            goal = self._goal_list[location]
            self._last_goal = location
        except KeyError:
            # If the location is not found, log a warning message
            msg = "The location of the {} is not registered yet!".format(
                location)
            rospy.logwarn(msg)
            return False
        
        # Send the goal to the move_base client
        self._move_base_client.send_goal(goal)
        msg = "The robot is heading over to the {}!".format(location)
        rospy.loginfo(msg)
        return True
    
    ## @brief Stop the robot by cancel the goal was sent to action server
    ##
    ## @param 
    ##
    ## @return True

    def stop(self):  
        # Cancel the current goal
        self._move_base_client.cancel_goal()
        rospy.loginfo("Your robot is requested to stop!")
        return True

    ## @brief get the current location of the robot in the pre-defined area list
    ##
    ## @param 
    ##
    ## @return current location (string)
    def get_robot_location(self):  
        # Get the current x and y position of the robot
        pos_x, pos_y = self._position
        current_location = None
        # Find the location that robot is locating at
        for area_key in self._area_list:
            area = self._area_list[area_key]
            x_min, x_max = area["area"]["x"]
            y_min, y_max = area["area"]["y"]

            if x_min < pos_x < x_max and y_min < pos_y < y_max:
                current_location = area["name"]
                break
            else:
                pass

        return current_location
    
    ## @brief Get status of the goal that was sent by move_to() method
    ##
    ## @param 
    ##
    ## @return goal stasus that followed output of move_base_client.get_state(), 
    ##
    ## @return last goal location. Return None if goal is not correctly set 
    ##        
    def get_goal_status(self):
        # Get the current status of the goal
        status_id = str(self._move_base_client.get_state())
        status = self._status_list[status_id]

        if status == "ACTIVE" or "PREEMPTED" or "SUCCEEDED" or "ABORTED" or "PREEMPTING":
            # Return the status and the last goal if the status is one of these values
            return status, self._last_goal
        else:
            return status, None
       
    ## @brief Get the location name list
    ##
    ## @param 
    ##
    ## @return location name list
    def get_location_list(self):  
        locations_name = []
        for location_key in self._location_list:
            location = self._location_list[location_key]
            locations_name.append(location["name"])
        # Return a list of all the location names
        return locations_name

    ## @brief Get the area name list
    ##
    ## @param 
    ##
    ## @return area name list
    def get_area_list(self):  
        areas_name = []
        for area_key in self._area_list:
            area = self._area_list[area_key]
            areas_name.append(area["name"])
        # Return a list of all the area names
        return areas_name
    
    
    
    """
    PRIVATE FUNCTION (for internal use)
    """
    
    ## @brief Create a goal message to send to action server
    ##
    ## @param position The coordiate x,y of the robot in /map frame
    ##
    ## @param orientation orientation of the bot in (roll,pitch,yaw)
    ##
    ## @return goal message
    def _create_goal_msg(self, position, orientation):
        goal_msg = move_base_msgs.msg.MoveBaseGoal()
        # header
        goal_msg.target_pose.header.stamp = rospy.Time.now()
        goal_msg.target_pose.header.frame_id = "map"
        # pose
        goal_msg.target_pose.pose.position.x = position[0]
        goal_msg.target_pose.pose.position.y = position[1]

        goal_msg.target_pose.pose.orientation = geometry_msgs.msg.Quaternion(
            *tf_conversions.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2]))

        return goal_msg

    ## @brief Load config .yaml file from paremeter server to get pre-defined location and area list
    ##
    ## @param 
    ##
    ## @return False if config file is not be loaded or frame_id was wrongly set
    ##
    ## @return True if success
    def _load_map_info(self):
        
        try:
            frame_id = rospy.get_param("/house_map/frame_id")
            # get location_list and area_list from Parameter server
            self._location_list = rospy.get_param("/house_map/locations")
            self._area_list = rospy.get_param("/house_map/areas")
        except KeyError:
            rospy.logwarn(
                "Can not find location or area list from Parameter Server !")
            return False

        if frame_id != "map":
            # If frame_id was not set to '/map', print the error msg
            rospy.logfatal(
                "Your map infomation was loaded with frame_id ='{}' ! Please change to 'odom' frame!".format(frame_id))
            rospy.logfatal("Node is forcelly terminated!")
            # Force program to be terminated
            rospy.signal_shutdown("Wrong frame_id!")
            return False

        return True
    
    ## @brief Create a list of goals
    ##
    ## @param 
    ##
    ## @return True
    def _create_goal_list(self):
        for location_key in self._location_list:
            location = self._location_list[location_key]
            
            #update _goal_list
            # message type of a goal { string : move_base_msgs.msg.MoveBaseGoal}
            self._goal_list[location["name"]] = self._create_goal_msg(
                location["position"], location["orientation"])
        return True

    ## @brief # Callback function for updating position from /odom topic
    ##
    ## @param odom_msg messgage in /odom topic
    ##
    ## @return True if success, False if the following exception was raised (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException)
    def _odom_callback(self, odom_msg): 
        
        pose = geometry_msgs.msg.PoseStamped()

        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose
        try:
            pose_in_map = self._tf_listener.transformPose("/map", pose)
            # Update position
            self._position = [pose_in_map.pose.position.x,
                              pose_in_map.pose.position.y]
            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        
        return False
