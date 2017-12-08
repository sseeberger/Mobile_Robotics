


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
        rospy.loginfo("Waiting for TF transforms")
        while not (listener.frameExists("/tag_0") and listener.frameExists("/map")):
            continue
        rospy.sleep(1)
        if (listener.frameExists("/tag_0") and listener.frameExists("/map")): 
            try:
                # Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
                (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/tag_0", rospy.Time(0))
                rospy.loginfo("The location of tag_0 in world coordinates is")
                print tag_position 
                print tag_quaternion
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "Transform Exception Reached!"
        
        
        
        
        # World Coordinates for Waypoint #1
        position = {'x': tag_position[0], 'y' : tag_position[1]}
        quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

        rospy.loginfo("Go to Tag 0 at approximate world coordinate location (%s, %s)", position['x'], position['y'])
        success = april.go_to_waypoint(position, quaternion)

        if success:
            rospy.loginfo("April reached Tag 0!")
        else:
            rospy.loginfo("April failed to reach Tag 0")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)
       

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting the program.")

