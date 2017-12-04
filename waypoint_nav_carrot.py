# The program "go_to_specific_point_on_map.py" by Mark Silliman (Copyright (c) 2015) created as part of the "Learn TurtleBot and ROS"
# online tuturial was referenced in the creation of this program. The source code for this program can be found at 
# https://github.com/markwsilliman/turtlebot/. 


# This program defines 5 waypoint positions in world coordinates for the turtlebot under test to attempt to navigate to. 
# The program begins by determining the turtlebot's current position in its world coordinate system and then uses these 
# position values to determine the locations in the world coordinate system of the 5 waypoints defined by the user. The user
# will thus modify this script prior to running to include the 5 waypoint locations, given as relative x and y distances 
# from the turtlebot's starting position. Once the world coordinate locations of the 5 waypoints have been calculated, the 
# waypoint coordinates are sequentially sent as goals to the turtlebot's move_base node. The turtlebot, in response to receiving
# a goal, will attempt to then navigate to the specified waypoint location, while avoiding obstacles along the way through 
# its move_base navigation stack planner (setup upon running gmapping_demo.launch). If the turtlebot is successful in navigating 
# to the given waypoint in a given amount of time (here defined as 60s), the program will print a "success" message to the user and 
# will proceed to give the next waypoint goal to the turtlebot so that it can attempt to navigate to that next waypoint location. 
# If the turtlebot is unsuccessful in navigating to the given waypoint, an "unsuccessful" message will be printed to the user 
# and the program will proceed to give the next waypoint goal to the turtlebot anyway so that it can attempt to navigate to that 
# waypoint location next. Once all of the waypoints have been sent to the turtlebot, the program will terminate. 

# The launch files turtlebot_bringup minimal.launch and turtlebot_navigation gmapping_demo.launch should be run on Turtlebot 
# before running this script on the host computer. 


import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
import tf


class WaypointNavigation():
    def __init__(self):

        self.goal_sent = False

    # What to do when Ctrl-C is input or failure occurs
    rospy.on_shutdown(self.shutdown)
    
    # Start move_base action server
    self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Wait for the action server to come up")

    # Allow up to 5 seconds for the action server to come up
    self.move_base.wait_for_server(rospy.Duration(5))


    def go_to_waypoint(self, pos, quat):

        end_goal_search = 0
        carrot_div = 1

        while (not end_goal_search):

            # Send a goal
            self.goal_sent = True
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = Pose(Point(pos['x']/carrot_div, pos['y']/carrot_div, 0.000),
                                     Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

            # Send move goal to the Turtlebot so it attempts to navigate to the waypoint
            self.move_base.send_goal(goal)

            # Allow TurtleBot up to 60 seconds to navigate to the waypoint
            success = self.move_base.wait_for_result(rospy.Duration(60)) 

            state = self.move_base.get_state()
            result = False

            if success and state == GoalStatus.SUCCEEDED:
                # The turtlebot successfully navigated to the designated waypoint
                if carrot_div == 1:
                    result = True
                    end_goal_search = 1
                else:
                    carrot_div = 1
            elif carrot_div >= 8:
                end_goal_search = 1
                result = False
                self.move_base.cancel_goal()
            else:
                self.move_base.cancel_goal()
                carrot_div = carrot_div*2

        self.goal_sent = False
        return result

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stopping the program")
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_nav', anonymous=False)
        april = WaypointNavigation()

        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not (listener.frameExists("/base_link") and listener.frameExists("/map")):
            continue
        rospy.sleep(1)
        if (listener.frameExists("/base_link") and listener.frameExists("/map")): 
            try:
                # Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
                (map_position,map_quaternion) = listener.lookupTransform("/base_link", "/map", rospy.Time(0))
                print position 
                print quaternion
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "Transform Exception Reached!"



        # World Coordinates for Waypoint #1
        position = {'x': -(map_position[0])+3, 'y' : map_position[1]+0.5}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to Waypoint #1 at world coordinate location (%s, %s)", position['x'], position['y'])
        success = april.go_to_waypoint(position, quaternion)

        if success:
            rospy.loginfo("April reached Waypoint #1!")
        else:
            rospy.loginfo("April failed to reach Waypiont #1")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)


        # World Coordinates for Waypoint #2
        position = {'x': 1.22, 'y' : 2.56}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to Waypoint #2 at world coordinate location (%s, %s)", position['x'], position['y'])
        success = april.go_to_waypoint(position, quaternion)

        if success:
            rospy.loginfo("April reached Waypoint #2!")
        else:
            rospy.loginfo("April failed to reach Waypiont #2")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)


        # World Coordinates for Waypoint #3
        position = {'x': 1.22, 'y' : 2.56}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to Waypoint #3 at world coordinate location (%s, %s)", position['x'], position['y'])
        success = april.go_to_waypoint(position, quaternion)

        if success:
            rospy.loginfo("April reached Waypoint #3!")
        else:
            rospy.loginfo("April failed to reach Waypiont #3")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting the program.")

