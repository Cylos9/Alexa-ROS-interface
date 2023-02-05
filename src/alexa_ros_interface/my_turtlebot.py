#!/usr/bin/env python3
import threading
import rospy
import actionlib
import geometry_msgs.msg
import move_base_msgs.msg
import tf
import tf_conversions
from nav_msgs.msg import Odometry


class MyTurtlebot:
    def __init__(self, disable_signals=False):

        # class member
        self._position = []
        self._locationList = {}
        self._areaList = {}
        self._goalList = {}
        self._statusList = {
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
        self._lastGoal = ""
        # Init ROS node
        rospy.init_node('alexa_interface', disable_signals=disable_signals)

        # create a simpleActionClient that request to move_bave action server
        self._move_base_client = actionlib.SimpleActionClient(
            "move_base", move_base_msgs.msg.MoveBaseAction)

        rospy.loginfo("Wating for Action Server start ...")
        self._move_base_client.wait_for_server()
        rospy.loginfo("Connected to Action Server !!!")

        # tf transform handler
        self._tf_listener = tf.TransformListener()
        # wait until tf buffer be filled up, avoiding tf.ExtrapolationException
        rospy.sleep(1)

        # create Subscriber to get current position
        rospy.Subscriber("odom", Odometry, self._odomCallback)

################# PUBLIC FUNCTION #########################

    def initRobot(self): #DONE
        success = self._loadMapInfo()
        if not success:
            rospy.logwarn("Fail to load map !")
        else:
            self._createGoalList()

        return True

    def getGoalStatus(self): #DONE

        status_id = str(self._move_base_client.get_state())
        status = self._statusList[status_id]
        
        if status == "ACTIVE" or "PREEMPTED" or "SUCCEEDED" or "ABORTED" or "PREEMPTING":
            return status,self._lastGoal
        else:
            return status,None

    def moveTo(self, location):  # DONE
        try:
            goal = self._goalList[location]
            self._lastGoal = location
        except KeyError:
            msg = "The location of the {} is not registed yet!".format(
                location)
            rospy.logwarn(msg)
            return False

        self._move_base_client.send_goal(goal)

        msg = "The robot is heading over to the {}!".format(location)
        rospy.loginfo(msg)

        return True

    def stop(self):  # DONE
        self._move_base_client.cancel_goal()
        rospy.loginfo("Your robot is requested to stop!")
        return True

    def getRobotLocation(self):  # DONE
        pos_x, pos_y = self._position
        current_location = None

        for area_key in self._areaList:
            area = self._areaList[area_key]
            x_min, x_max = area["area"]["x"]
            y_min, y_max = area["area"]["y"]

            if x_min < pos_x < x_max and y_min < pos_y < y_max:
                current_location = area["name"]
                break
            else:
                pass

        return current_location

    def getLocationList(self):  # DONE
        locations_name = []
        for location_key in self._locationList:
            location = self._locationList[location_key]
            locations_name.append(location["name"])

        return locations_name

    def getAreaList(self):  # DONE
        areas_name = []
        for area_key in self._areaList:
            area = self._areaList[area_key]
            areas_name.append(area["name"])

        return areas_name

################# PROTECTED FUNCTION (internal use) #########################
    def _createGoalMessage(self, position, orientation):  # DONE
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

    def _loadMapInfo(self):  # DONE
        # get locationList and areaList from Parameter server
        try:
            frame_id = rospy.get_param("/house_map/frame_id")
            self._locationList = rospy.get_param("/house_map/locations")
            self._areaList = rospy.get_param("/house_map/areas")
        except KeyError:
            rospy.logwarn(
                "Can not find location or area list from Parameter Server !")
            return False

        if frame_id != "map":
            rospy.logfatal(
                "Your map infomation was loaded with frame_id ='{}' ! Please change to 'odom' frame!".format(frame_id))
            rospy.logfatal("Node is forcelly terminated!")
            # force program to be terminated
            rospy.signal_shutdown("Wrong frame_id!")
            return False

        return True

    def _createGoalList(self):  # DONE

        # create goal list
        for location_key in self._locationList:
            location = self._locationList[location_key]

            # message type of a goal { string : move_base_msgs.msg.MoveBaseGoal}
            self._goalList[location["name"]] = self._createGoalMessage(
                location["position"], location["orientation"])
        return True

    def _odomCallback(self, odom_msg):  # DONE

        pose = geometry_msgs.msg.PoseStamped()

        pose.header = odom_msg.header
        pose.pose = odom_msg.pose.pose
        try:
            pose_in_map = self._tf_listener.transformPose("/map", pose)
            # update position
            self._position = [pose_in_map.pose.position.x,
                              pose_in_map.pose.position.y]
            return True

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        return False


if __name__ == "__main__":

    myBot = MyTurtlebot()

    rospy.spin()