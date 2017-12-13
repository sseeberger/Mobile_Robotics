


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

finished_navigating = 0
all_tags_found = 0


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
        


class AprilTag():
  
    def __init__(self,tag_id,curr_nav):
      self.tag_id = tag_id
      self.curr_nav = curr_nav
      self.found = 0
      self.have_pos = 0
      self.x_pos = 0 
      self.y_pos = 0
  
        

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
		dir = data.bumper
		state = data.state
		if (state):
			rospy.loginfo("bump")
	   

if __name__ == '__main__':
    try:
        rospy.init_node('waypoint_nav', anonymous=False)
        ft_pub = rospy.Publisher('foundIt', Int8, queue_size=1) #publisher for lights and sound
        
        april = WaypointNavigation()
        
        AprilTag1 = AprilTag('tag_0',1)
        AprilTag2 = AprilTag('tag_1',0)
        AprilTag3 = AprilTag('tag_2',0)
        AprilTag4 = AprilTag('tag_3',0)
        AprilTag5 = AprilTag('tag_4',0)
        
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
       

        
        while(not all_tags_found):
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
	      
	      #rospy.loginfo("Map Area: (%s)", maparea)
	      #rospy.loginfo("Contains traversable point: (%s)", traversable)
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

	  rospy.loginfo("Go to random point at approximate world coordinate location (%s, %s)", position['x'], position['y'])
	  april.go_to_waypoint(position, quaternion)
	  
	  while(not finished_navigating):
	    
	    # Test to see if robot has tried to reach random target yet
	    finished_navigating = april.move_base.wait_for_result(rospy.Duration(0)) 
	    
	    if (listener.frameExists("/" + AprilTag1.tag_id) and listener.frameExists("/map")): 
	      try:
		  # Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
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
		  # Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
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
		  # Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
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
		  # Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
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
		  # Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
		  (tag_position,tag_quaternion) = listener.lookupTransform("/map", "/" + AprilTag5.tag_id, rospy.Time(0))
		  rospy.loginfo("The location of %s in world coordinates is", AprilTag5.tag_id)
		  print tag_position 
		  print tag_quaternion
		  AprilTag5.have_pos = 1
		  AprilTag5.x_pos = tag_position[0]
		  AprilTag5.y_pos = tag_position[1]
	      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		  print "Transform Exception Reached! Tag 5"
	    
	    
	    
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
		    AprilTag1.found = 1
		    AprilTag1.curr_nav = 0
		    AprilTag2.curr_nav = 1
		  else: 
		    AprilTag1.have_pos = 0
		    
		  
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
		    AprilTag2.found = 1
		    AprilTag2.curr_nav = 0
		    AprilTag3.curr_nav = 1
		  else: 
		    AprilTag2.have_pos = 0
		  
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
		    AprilTag3.found = 1
		    AprilTag3.curr_nav = 0
		    AprilTag4.curr_nav = 1
		  else: 
		    AprilTag3.have_pos = 0
		

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
		    AprilTag4.found = 1
		    AprilTag4.curr_nav = 0
		    AprilTag5.curr_nav = 1
		  else: 
		    AprilTag4.have_pos = 0
		  
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
		    AprilTag5.found = 1
		    AprilTag5.curr_nav = 0
		    all_tags_found = 1
		  else: 
		    AprilTag5.have_pos = 0
              
	       	      	      
          state = april.move_base.get_state()
          
          if state == GoalStatus.SUCCEEDED:
              # The turtlebot successfully navigated to the designated waypoint
              rospy.loginfo("April reached a point!")
          else:
	      rospy.loginfo("April did not reach the point")  
          
	  april.move_base.cancel_goal()
	  april.goal_sent = False

	  # Sleep to give the last log messages time to be sent
	  rospy.sleep(2)
	  
	  ft_pub.publish(0)
	  
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
		  print "Transform Exception Reached! Base to map"
	  	  
        
	rospy.loginfo("April reached all of the April Tags!")
        

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting the program.")

