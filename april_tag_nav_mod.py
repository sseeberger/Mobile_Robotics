


import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
import tf
from nav_msgs.msg import *
import numpy as np
from kobuki_msgs.msg import *
from tf.transformations import *


firstAprilTag = ''
secondAprilTag = ''
thirdAprilTag = ''
fourthAprilTag = ''
fifthAprilTag = ''


costmap = []
pt = []
width = 0
height = 0
resolution = 0
region_size = 0

prev_goal_x_1 = 0
prev_goal_y_1 = 0
prev_goal_x_2 = 0
prev_goal_y_2 = 0
prev_goal_x_3 = 0
prev_goal_y_3 = 0
good_goal = 0


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
	
	self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)


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
            success = self.move_base.wait_for_result(rospy.Duration(20)) 

            state = self.move_base.get_state()
            result = False

            if success and state == GoalStatus.SUCCEEDED:
                # The turtlebot successfully navigated to the designated waypoint
                if carrot_div == 1:
                    result = True
                    end_goal_search = 1
                else:
                    carrot_div = 1
            elif carrot_div >= 1:
                end_goal_search = 1
                result = False
                self.move_base.cancel_goal()
            else:
                self.move_base.cancel_goal()
                carrot_div = carrot_div*2

        self.goal_sent = False
        return result
      
    def full_rot(self):
    
      r = rospy.Rate(5)
      turn_cmd = Twist()
      turn_cmd.linear.x = 0
      turn_cmd.angular.z = np.pi/6
      
      k = 0
      while (k < 160):
	self.cmd_vel.publish(turn_cmd)
	rospy.sleep(0.1)
	k = k + 1
      
      rospy.sleep(0.5)
      

    def shutdown(self):
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stopping the program")
        rospy.sleep(1)
        

def callback(data):
	    global costmap
	    global pt
	    global width
	    global height
	    global resolution
	    global region_size
	    costmap = list(data.data)
	    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", np.where(costmap == 100)[0])
	    meta = data.info
	    oo = meta.origin
	    pt = oo.position
	    rospy.loginfo("origin position: (%s, %s)", pt.x, pt.y)
	    width = meta.width
	    rospy.loginfo("map width: (%s)", width)
	    height = meta.height
	    rospy.loginfo("map height: (%s)", height)
	    resolution = meta.resolution
	    rospy.loginfo("map resolution: (%s)", resolution)
	    region_size = int(np.ceil((1/resolution)/4))
	    rospy.loginfo("Region Size: (%s)", region_size)
		
		
def bumpCallback(data):
		dir = data.bumper
		state = data.state
		if (state):
			rospy.loginfo("bump")
	   

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_nav', anonymous=False)
        april = WaypointNavigation()
        
        grid = OccupancyGrid()
        
        global_cost_sub = rospy.Subscriber("/map", OccupancyGrid, callback)
	rospy.Subscriber("/kobuki_node/events/bumper", BumperEvent, bumpCallback)
        
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not (listener.frameExists("/base_link") and listener.frameExists("/map")):
            continue
        rospy.sleep(1)
        if (listener.frameExists("/base_link") and listener.frameExists("/map")): 
            try:
                # Store the turtlebot's starting position in the global coordinate system in the april_map_position and april_map_quaternion variables
                (april_map_position,april_map_quaternion) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                print april_map_position 
                print april_map_quaternion
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "Transform Exception Reached!"
       

        
        while(1):
	  maparea = []
	  loop_count = 0
	  
	  #global_cost_sub.unregister()
	  #global_cost_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback)
	  rospy.sleep(1)

	  while (not good_goal):
	    
	    n = np.random.randint(0,len(costmap)-1)
	    for k in range(-2,3,1):
	      start = n+(k*width)-5
	      end = n+(k*width)+5 
	      maparea = maparea + costmap[start:end]
		
	    traversable = (maparea.count(0) == len(maparea))
		
	    if (traversable):
	      
	      rospy.loginfo("Map Area: (%s)", maparea)
	      rospy.loginfo("Contains traversable point: (%s)", traversable)
	      explore_goal = n 
	      explore_goal_x = (explore_goal%width)*resolution + pt.x
	      explore_goal_y = ((explore_goal - explore_goal%width)/width)*resolution + pt.y 
		  
	      x_diff = np.absolute(explore_goal_x - april_map_position[0])
	      y_diff = np.absolute(explore_goal_y - april_map_position[1])
	      
	      if (loop_count < 50):
		if ((x_diff <= 1.5) and (x_diff >= 0.5) and (y_diff <= 1.5) and (y_diff >= 0.5)):
		  if ((np.absolute(explore_goal_x-prev_goal_x_2) > 0.5) and (np.absolute(explore_goal_y-prev_goal_y_2) > 0.5)):
		    if ((np.absolute(explore_goal_x-prev_goal_x_3) > 0.5) and (np.absolute(explore_goal_y-prev_goal_y_3) > 0.5)):
		      good_goal = 1
		    else:
		      good_goal = 0
		  else:
		    good_goal = 0
	        else:
		  good_goal = 0
	      else:
		good_goal = 1

	    maparea = []
	    loop_count = loop_count + 1
	  
	  
	  
	  prev_goal_x_3 = prev_goal_x_2
	  prev_goal_y_3 = prev_goal_y_2
	  prev_goal_x_2 = prev_goal_x_1
	  prev_goal_y_2 = prev_goal_y_1
	  prev_goal_x_1 = explore_goal_x
	  prev_goal_y_1 = explore_goal_y
	  good_goal = 0
	  
	  
	  # Go to the exploration waypoint
	  position = {'x': explore_goal_x, 'y' : explore_goal_y}
	  quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}

	  rospy.loginfo("Go to Tag 0 at approximate world coordinate location (%s, %s)", position['x'], position['y'])
	  success = april.go_to_waypoint(position, quaternion)

	  if success:
	      rospy.loginfo("April reached Tag 0!")
	  else:
	      rospy.loginfo("April failed to reach Tag 0")

	  # Sleep to give the last log messages time to be sent
	  rospy.sleep(1)
	  april.full_rot()
	  while not (listener.frameExists("/base_link") and listener.frameExists("/map")):
	      continue
	  rospy.sleep(1)
	  if (listener.frameExists("/base_link") and listener.frameExists("/map")): 
	      try:
		  # Store the turtlebot's starting position in the global coordinate system in the april_map_position and april_map_quaternion variables
		  (april_map_position,april_map_quaternion) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
		  print april_map_position 
		  print april_map_quaternion
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		  print "Transform Exception Reached!"
	  
        
        
        

       

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting the program.")

