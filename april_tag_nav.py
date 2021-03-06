# The program "go_to_specific_point_on_map.py" by Mark Silliman (Copyright (c) 2015) created as part of the "Learn TurtleBot and ROS"
# online tuturial was referenced in the creation of this program. The source code for this program can be found at 
# https://github.com/markwsilliman/turtlebot/. 
#
# 
# This program implements a random point generation algorithm to direct a Turtlebot (April) to traverse the free space around it, 
# map its surroundings, and find 5 fiducial markers (April tags) located within its environment. The Turtlebot is given a particular 
# order of April tag "tag ids" and attempts to sequentially navigate to the location of each tag in the correct order. Once the Turtlebot 
# locates an April tag in its environment, it checks to see whether it is currently attempting to navigate to the April tag with that tag 
# id. If it finds that it is, the Turtlebot will immediately attempt to nagvigate to that April tag. If it finds that it is not, it will 
# save the location of that April tag so that it can be used for navigation in the future when that April tag is reached within the ordered 
# list. Once the Turtlebot successfully navigates to an April tag, it will perform a "waving" motion by turning slightly to the left and 
# then slightly to the right to indicate to the observers that it successfully found and navigated to that April tag. 
# For the random point exploration algorithm, this script creates a ROS node that subscribes to the "/map" topic generated by gmapping. 
# Thus, the script continuously reads the current costmap being generated by gmapping into a Python list. The script then generates a random
# integer to use as an index for this list, and examines the value at this index and a small netighborhood around it (a 5x5 region of the 
# costmap). If every examined value in the neighborhood is a 0 (free space with no cost to traverse to), the script will perform a conversion 
# on this point to determine how far away it is from the Turtlebot in world coordinates. If the point is close enough to the Turtlebot (between 
# 0.5 and 1.5 meters according to the current thresholds) and far enough away from the previous two locations traversed to by the Turtlebot
# (over 0.5 meters away according to the current threshold), the Turtlebot will attempt to navigate to this point next to continue to explore 
# its surroundings. Otherwise, the script will generate another random point and continue this process until a point that satisfies the 
# conditions is found. This random point navigation will continue until the Turtlebot finds an April tag in its surroundings, at which point it 
# will respond to the April tag(s) as described previously. Once the Turtlebot has successfully navigated to each April tag in the list in the 
# order provided, the script will terminate, the Turtlebot will stop exploring its environment, and a message indicating the Turtlebot's 
# success in navigating to all of the April tags will be displayed in the terminal. 
# 
# 
# The launch files turtlebot_bringup minimal.launch and gmapping_demo_altered.launch should be run on Turtlebot 
# before running this script on the host computer. Additionally, the launch file april_tags_detector.launch should be run on the 
# host computer before running this script for successful operation to occur. 
# 
#



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
from std_msgs.msg import *

# Variable initializations for variables set by subscribing to the "/map" topic
costmap = []
pt = []
width = 0
height = 0
resolution = 0
region_size = 0

# Variable initializations for storing previous 3 points navigated to in the map 
prev_goal_x_1 = 0
prev_goal_y_1 = 0
prev_goal_x_2 = 0
prev_goal_y_2 = 0
prev_goal_x_3 = 0
prev_goal_y_3 = 0
good_goal = 0

# Variable initializations for flags for inidividual point navigation and overall navigation success
finished_navigating = 0
all_tags_found = 0

# Variable initializations for running transform code to detect April tags in the Turtlebot's environment 
listener = None
tag_position = None
tag_quaternion = None
AprilTag1 = None
AprilTag2 = None
AprilTag3 = None
AprilTag4 = None
AprilTag5 = None



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

        # Send a goal
        self.goal_sent = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(pos['x'], pos['y'], 0.000),
                                 Quaternion(quat['r1'], quat['r2'], quat['r3'], quat['r4']))

        # Send move goal to the Turtlebot so it attempts to navigate to the waypoint
        self.move_base.send_goal(goal)

    def found_tag(self):
      # Have turtlebot perform "waving" motion upon finding an April tag
      r = rospy.Rate(5)
      turn_cmd = Twist()
      turn_cmd.linear.x = 0
      turn_cmd.angular.z = np.pi/2
      
      # Rotate slightly left 
      k = 0
      while (k < 10):
	self.cmd_vel.publish(turn_cmd)
	rospy.sleep(0.1)
	k = k + 1
	
      # Rotate slightly right 
      turn_cmd.angular.z = -np.pi/2
      k = 0
      while (k < 10):
	self.cmd_vel.publish(turn_cmd)
	rospy.sleep(0.1)
	k = k + 1
      
    
    
    def full_rot(self):
    
      # Perform a full rotation
      # Called after each random point is traversed to in order to help the robot try to find the April tags in 
      # its environment 
      r = rospy.Rate(5)
      turn_cmd = Twist()
      turn_cmd.linear.x = 0
      turn_cmd.angular.z = np.pi/6
      
      # Turn slowly and check for April tag transforms between each turning increment 
      k = 0
      while (k < 160):
	self.cmd_vel.publish(turn_cmd)
	rospy.sleep(0.1)
	k = k + 1
	
	if (listener.frameExists("/" + AprilTag1.tag_id) and listener.frameExists("/map")): 
	  try:
	    # Store the first April tag position in the global coordinate system in the tag_position and tag_quaternion variables
	    (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag1.tag_id, rospy.Time(0))
	    rospy.loginfo("The location of %s in world coordinates is", AprilTag1.tag_id)
	    print tag_position 
	    print tag_quaternion
	    AprilTag1.have_pos = 1
	    AprilTag1.x_pos = tag_position[0]
	    AprilTag1.y_pos = tag_position[1]
	  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    print "Transform Exception Reached! Tag 1"
	      
	      
	if (listener.frameExists("/" + AprilTag2.tag_id) and listener.frameExists("/map")): 
	  try:
	    # Store the second April tag position in the global coordinate system in the tag_position and tag_quaternion variables
	    (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag2.tag_id, rospy.Time(0))
	    rospy.loginfo("The location of %s in world coordinates is", AprilTag2.tag_id)
	    print tag_position 
	    print tag_quaternion
	    AprilTag2.have_pos = 1
	    AprilTag2.x_pos = tag_position[0]
	    AprilTag2.y_pos = tag_position[1]
	  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    print "Transform Exception Reached! Tag 2"
		  
	    
	if (listener.frameExists("/" + AprilTag3.tag_id) and listener.frameExists("/map")): 
	  try:
	    # Store the third April tag position in the global coordinate system in the tag_position and tag_quaternion variables
	    (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag3.tag_id, rospy.Time(0))
	    rospy.loginfo("The location of %s in world coordinates is", AprilTag3.tag_id)
	    print tag_position 
	    print tag_quaternion
	    AprilTag3.have_pos = 1
	    AprilTag3.x_pos = tag_position[0]
	    AprilTag3.y_pos = tag_position[1]
	  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    print "Transform Exception Reached! Tag 3"
		  
	    
	if (listener.frameExists("/" + AprilTag4.tag_id) and listener.frameExists("/map")): 
	  try:
	    # Store the fourth April tag position in the global coordinate system in the tag_position and tag_quaternion variables
	    (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag4.tag_id, rospy.Time(0))
	    rospy.loginfo("The location of %s in world coordinates is", AprilTag4.tag_id)
	    print tag_position 
	    print tag_quaternion
	    AprilTag4.have_pos = 1
	    AprilTag4.x_pos = tag_position[0]
	    AprilTag4.y_pos = tag_position[1]
	  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    print "Transform Exception Reached! Tag 4"

	    
	if (listener.frameExists("/" + AprilTag5.tag_id) and listener.frameExists("/map")): 
	  try:
	    # Store the fifth April tag position in the global coordinate system in the tag_position and tag_quaternion variables
	    (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag5.tag_id, rospy.Time(0))
	    rospy.loginfo("The location of %s in world coordinates is", AprilTag5.tag_id)
	    print tag_position 
	    print tag_quaternion
	    AprilTag5.have_pos = 1
	    AprilTag5.x_pos = tag_position[0]
	    AprilTag5.y_pos = tag_position[1]
	  except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	    print "Transform Exception Reached! Tag 5"
      
      rospy.sleep(0.5)
      

    def shutdown(self):
    	# What to do on shutdown
        if self.goal_sent:
            self.move_base.cancel_goal()
        rospy.loginfo("Stopping the program")
        rospy.sleep(1)
        


class AprilTag():
	# Class for defining a particular April tag
  
    def __init__(self,tag_id,curr_nav):
      self.tag_id = tag_id
      self.curr_nav = curr_nav
      self.found = 0
      self.have_pos = 0
      self.x_pos = 0 
      self.y_pos = 0
  
        

def callback(data):
		# Callback run when new data is posted to the "/map" topic

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
	    #rospy.loginfo("origin position: (%s, %s)", pt.x, pt.y)
	    width = meta.width
	    #rospy.loginfo("map width: (%s)", width)
	    height = meta.height
	    #rospy.loginfo("map height: (%s)", height)
	    resolution = meta.resolution
	    #rospy.loginfo("map resolution: (%s)", resolution)
	    region_size = int(np.ceil((1/resolution)/4))
	    #rospy.loginfo("Region Size: (%s)", region_size)
		
		
def bumpCallback(data):
		# Callback for subscribing to "/kobuki_node/events/bumper" topic to access bumper information
		# (Currently unused)

		dir = data.bumper
		state = data.state
		if (state):
			rospy.loginfo("bump")
	   

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_nav', anonymous=False)     		# Initialize the node for the script
        ft_pub = rospy.Publisher('foundIt', Int8, queue_size=10) 	# Publisher for lights and sound (currently unused)
        
        april = WaypointNavigation()			# Define april as being the object for waypoint navigation 
        
        # List of April Tag objects
        # Modify the value of the "tag_id" variable being defined for each class 
        # to change the order in which the Turtlebot navigates to particular April tags
        AprilTag1 = AprilTag('tag_0',1)
        AprilTag2 = AprilTag('tag_1',0)
        AprilTag3 = AprilTag('tag_2',0)
        AprilTag4 = AprilTag('tag_3',0)
        AprilTag5 = AprilTag('tag_4',0)
        
        # Datatype for occupancy grid 
        grid = OccupancyGrid()
        
        # Subscribe to the "/map" topic and the "/kobuki_node/events/bumper" topic
        global_cost_sub = rospy.Subscriber("/map", OccupancyGrid, callback)
	rospy.Subscriber("/kobuki_node/events/bumper", BumperEvent, bumpCallback)
        
        # Transform code to read the current position of the Turtlebot in world coordinates 
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
       

        # While all of the April tags have not been found and successfully navigated to 
        while(not all_tags_found):
	  maparea = []
	  loop_count = 0
	  
	  rospy.sleep(1)

	  # While a good random point to traverse to has not been found
	  while (not good_goal):
	    
	    # Generate random index and read in region around that index in the costmap
	    n = np.random.randint(0,len(costmap)-1)
	    for k in range(-2,3,1):
	      start = n+(k*width)-5
	      end = n+(k*width)+5 
	      maparea = maparea + costmap[start:end]
		
		# Point is traversable if all of the points in the area have a value equal to 0
	    traversable = (maparea.count(0) == len(maparea))
		
	    if (traversable):
	      
	      #rospy.loginfo("Map Area: (%s)", maparea)
	      #rospy.loginfo("Contains traversable point: (%s)", traversable)
	      explore_goal = n 
	      explore_goal_x = (explore_goal%width)*resolution + pt.x
	      explore_goal_y = ((explore_goal - explore_goal%width)/width)*resolution + pt.y 
		  
		  # Find distance from current Turtlebot position to the random point generated in the x and y directions
	      x_diff = np.absolute(explore_goal_x - april_map_position[0])
	      y_diff = np.absolute(explore_goal_y - april_map_position[1])
	      
	      if (loop_count < 50):
	      	# Test if point is close enough to the Turtlebot and far away enough from the last 2 locations 
	      	# traversed to by the Turtlebot 
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

	  rospy.loginfo("Go to random point at approximate world coordinate location (%s, %s)", position['x'], position['y'])
	  april.go_to_waypoint(position, quaternion)
	  
	  while(not finished_navigating):
	    
	    # Test to see if robot has tried to reach random target yet
	    finished_navigating = april.move_base.wait_for_result(rospy.Duration(0)) 
	    
	    if (listener.frameExists("/" + AprilTag1.tag_id) and listener.frameExists("/map")): 
	      try:
		  # Store the first April tag position in the global coordinate system in the tag_position and tag_quaternion variables
		  (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag1.tag_id, rospy.Time(0))
		  rospy.loginfo("The location of %s in world coordinates is", AprilTag1.tag_id)
		  print tag_position 
		  print tag_quaternion
		  AprilTag1.have_pos = 1
		  AprilTag1.x_pos = tag_position[0]
		  AprilTag1.y_pos = tag_position[1]
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		  print "Transform Exception Reached! Tag 1"
	      
	      
	    if (listener.frameExists("/" + AprilTag2.tag_id) and listener.frameExists("/map")): 
	      try:
		  # Store the second April tag position in the global coordinate system in the tag_position and tag_quaternion variables
		  (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag2.tag_id, rospy.Time(0))
		  rospy.loginfo("The location of %s in world coordinates is", AprilTag2.tag_id)
		  print tag_position 
		  print tag_quaternion
		  AprilTag2.have_pos = 1
		  AprilTag2.x_pos = tag_position[0]
		  AprilTag2.y_pos = tag_position[1]
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		  print "Transform Exception Reached! Tag 2"
		  
	    
	    if (listener.frameExists("/" + AprilTag3.tag_id) and listener.frameExists("/map")): 
	      try:
		  # Store the third April tag position in the global coordinate system in the tag_position and tag_quaternion variables
		  (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag3.tag_id, rospy.Time(0))
		  rospy.loginfo("The location of %s in world coordinates is", AprilTag3.tag_id)
		  print tag_position 
		  print tag_quaternion
		  AprilTag3.have_pos = 1
		  AprilTag3.x_pos = tag_position[0]
		  AprilTag3.y_pos = tag_position[1]
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		  print "Transform Exception Reached! Tag 3"
		  
	    
	    if (listener.frameExists("/" + AprilTag4.tag_id) and listener.frameExists("/map")): 
	      try:
		  # Store the fourth April tag position in the global coordinate system in the tag_position and tag_quaternion variables
		  (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag4.tag_id, rospy.Time(0))
		  rospy.loginfo("The location of %s in world coordinates is", AprilTag4.tag_id)
		  print tag_position 
		  print tag_quaternion
		  AprilTag4.have_pos = 1
		  AprilTag4.x_pos = tag_position[0]
		  AprilTag4.y_pos = tag_position[1]
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		  print "Transform Exception Reached! Tag 4"

	    
	    if (listener.frameExists("/" + AprilTag5.tag_id) and listener.frameExists("/map")): 
	      try:
		  # Store the fifth April tag position in the global coordinate system in the tag_position and tag_quaternion variables
		  (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag5.tag_id, rospy.Time(0))
		  rospy.loginfo("The location of %s in world coordinates is", AprilTag5.tag_id)
		  print tag_position 
		  print tag_quaternion
		  AprilTag5.have_pos = 1
		  AprilTag5.x_pos = tag_position[0]
		  AprilTag5.y_pos = tag_position[1]
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		  print "Transform Exception Reached! Tag 5"
	    
	    
	    # Send the Turtlebot to AprilTag1 if it is next in the list and we have found the location of the tag in the environment
	    if (AprilTag1.curr_nav == 1):
	      if (AprilTag1.found != 1): 
		if (AprilTag1.have_pos == 1):
		  april.move_base.cancel_goal()
		  april.goal_sent = False
		  position = {'x': AprilTag1.x_pos, 'y' : AprilTag1.y_pos}
		  quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		  rospy.loginfo("Going to %s ", AprilTag1.tag_id)
		  april.go_to_waypoint(position, quaternion)
		  success = april.move_base.wait_for_result(rospy.Duration(30))
		  state = april.move_base.get_state()
		  finished_navigating = 1
		  if success and state == GoalStatus.SUCCEEDED:
		    rospy.loginfo("Found %s !", AprilTag1.tag_id)
		    ft_pub.publish(1)
		    rospy.sleep(5)
		    april.found_tag()
		    AprilTag1.found = 1
		    AprilTag1.curr_nav = 0
		    AprilTag2.curr_nav = 1
		  else: 
		    AprilTag1.have_pos = 0
		    

		# Send the Turtlebot to AprilTag2 if it is next in the list and we have found the location of the tag in the environment  
	    if (AprilTag2.curr_nav == 1):
	      if (AprilTag2.found != 1): 
		if (AprilTag2.have_pos == 1):
		  april.move_base.cancel_goal()
		  april.goal_sent = False
		  position = {'x': AprilTag2.x_pos, 'y' : AprilTag2.y_pos}
		  quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		  rospy.loginfo("Going to %s ", AprilTag2.tag_id)
		  april.go_to_waypoint(position, quaternion)
		  success = april.move_base.wait_for_result(rospy.Duration(30))
		  state = april.move_base.get_state()
		  finished_navigating = 1
		  if success and state == GoalStatus.SUCCEEDED:
		    rospy.loginfo("Found %s !", AprilTag2.tag_id)
		    ft_pub.publish(1)
		    rospy.sleep(5)
		    april.found_tag()
		    AprilTag2.found = 1
		    AprilTag2.curr_nav = 0
		    AprilTag3.curr_nav = 1
		  else: 
		    AprilTag2.have_pos = 0
		
		# Send the Turtlebot to AprilTag3 if it is next in the list and we have found the location of the tag in the environment  
	    if (AprilTag3.curr_nav == 1):
	      if (AprilTag3.found != 1): 
		if (AprilTag3.have_pos == 1):
		  april.move_base.cancel_goal()
		  april.goal_sent = False
		  position = {'x': AprilTag3.x_pos, 'y' : AprilTag3.y_pos}
		  quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		  rospy.loginfo("Going to %s ", AprilTag3.tag_id)
		  april.go_to_waypoint(position, quaternion)
		  success = april.move_base.wait_for_result(rospy.Duration(30))
		  state = april.move_base.get_state()
		  finished_navigating = 1
		  if success and state == GoalStatus.SUCCEEDED:
		    rospy.loginfo("Found %s !", AprilTag3.tag_id)
		    ft_pub.publish(1)
		    rospy.sleep(5)
		    april.found_tag()
		    AprilTag3.found = 1
		    AprilTag3.curr_nav = 0
		    AprilTag4.curr_nav = 1
		  else: 
		    AprilTag3.have_pos = 0
		

	    # Send the Turtlebot to AprilTag4 if it is next in the list and we have found the location of the tag in the environment
	    if (AprilTag4.curr_nav == 1):
	      if (AprilTag4.found != 1): 
		if (AprilTag4.have_pos == 1):
		  april.move_base.cancel_goal()
		  april.goal_sent = False
		  position = {'x': AprilTag4.x_pos, 'y' : AprilTag4.y_pos}
		  quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		  rospy.loginfo("Going to %s ", AprilTag4.tag_id)
		  april.go_to_waypoint(position, quaternion)
		  success = april.move_base.wait_for_result(rospy.Duration(30))
		  state = april.move_base.get_state()
		  finished_navigating = 1
		  if success and state == GoalStatus.SUCCEEDED:
		    rospy.loginfo("Found %s !", AprilTag4.tag_id)
		    ft_pub.publish(1)
		    rospy.sleep(5)
		    april.found_tag()
		    AprilTag4.found = 1
		    AprilTag4.curr_nav = 0
		    AprilTag5.curr_nav = 1
		  else: 
		    AprilTag4.have_pos = 0
		  
		# Send the Turtlebot to AprilTag5 if it is next in the list and we have found the location of the tag in the environment  
	    if (AprilTag5.curr_nav == 1):
	      if (AprilTag5.found != 1): 
		if (AprilTag5.have_pos == 1):
		  april.move_base.cancel_goal()
		  april.goal_sent = False
		  position = {'x': AprilTag5.x_pos, 'y' : AprilTag5.y_pos}
		  quaternion = {'r1' : 0.000, 'r2' : 0.000, 'r3' : 0.000, 'r4' : 1.000}
		  rospy.loginfo("Going to %s ", AprilTag5.tag_id)
		  april.go_to_waypoint(position, quaternion)
		  success = april.move_base.wait_for_result(rospy.Duration(30))
		  state = april.move_base.get_state()
		  finished_navigating = 1
		  if success and state == GoalStatus.SUCCEEDED:
		    rospy.loginfo("Found %s !", AprilTag5.tag_id)
		    ft_pub.publish(1)
		    rospy.sleep(5)
		    april.found_tag()
		    AprilTag5.found = 1
		    AprilTag5.curr_nav = 0
		    all_tags_found = 1
		  else: 
		    AprilTag5.have_pos = 0
              
	       	      	      
          state = april.move_base.get_state()
          
          if state == GoalStatus.SUCCEEDED:
              # The Turtlebot successfully navigated to the designated waypoint
              rospy.loginfo("April reached a point!")
          else:
          	# The Turtlebot did not successfully navigate to the designated waypoint
	      rospy.loginfo("April did not reach the point")  
          
	  april.move_base.cancel_goal()
	  april.goal_sent = False

	  # Sleep to give the last log messages time to be sent
	  rospy.sleep(2)
	  
	  ft_pub.publish(0)
	  finished_navigating = 0
	  
	  # Perform a full rotation to continue mapping and searching for April tags
	  april.full_rot()
	  
	  while not (listener.frameExists("/base_link") and listener.frameExists("/map")):
	      continue
	  rospy.sleep(1)
	  if (listener.frameExists("/base_link") and listener.frameExists("/map")): 
	      try:
		  # Store the turtlebot's next position in the global coordinate system in the april_map_position and april_map_quaternion variables
		  (april_map_position,april_map_quaternion) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
		  print april_map_position 
		  print april_map_quaternion
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		  print "Transform Exception Reached! Base to map"
	  	  
    
    # The Turtlebot successfully navigated to all 5 provided April tags in the environment!     
	rospy.loginfo("April reached all of the April Tags!")
        

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting the program.")

