


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

prev_goal_x = 0
prev_goal_y = 0
goal_offset = 1


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
        
        rospy.loginfo("Initial Rotations... ")
       

        
        while(1):
	  maparea = []
	  
	  #global_cost_sub.unregister()
	  #global_cost_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback)
	  rospy.sleep(1)

	  
	  for n in range(0,len(costmap)-1,region_size):
		for k in range(0,region_size,1):
		  start = n+(k*width)
		  end = n+(k*width)+region_size
		  maparea = maparea + costmap[start:end]
		
		unexplored = maparea.count(-1)
		traversable = (maparea.count(0) >= 10)
		highcost = maparea.count(100)
		if (traversable and (unexplored > (region_size*region_size)/5) and highcost == 0):
		  rospy.loginfo("Map Area: (%s)", maparea)
		  rospy.loginfo("Number of Unexplored points in Region: (%s)", unexplored)
		  rospy.loginfo("Contains traversable point: (%s)", traversable)
		  explore_goal = n  + ((maparea.index(0) - maparea.index(0)%region_size)/region_size)*width + maparea.index(0)%region_size
		  explore_goal_x_init = (explore_goal%width)*resolution + pt.x
		  explore_goal_y_init = ((explore_goal - explore_goal%width)/width)*resolution + pt.y
		  
		  explore_goal_x = np.random.uniform((explore_goal_x_init-goal_offset),(explore_goal_x_init+goal_offset))
		  explore_goal_y = np.random.uniform((explore_goal_y_init-goal_offset),(explore_goal_y_init+goal_offset))
		  explore_goal = int(((explore_goal_y - pt.y)/resolution)*width + ((explore_goal_x - pt.x)/resolution))

		  while((costmap[explore_goal] != 0) and (costmap[explore_goal+1] != 0) and (costmap[explore_goal-1] != 0) and (costmap[explore_goal+width] != 0)and (costmap[explore_goal-width] != 0)):
		    explore_goal_x = np.random.uniform((explore_goal_x_init-goal_offset),(explore_goal_x_init+goal_offset))
		    explore_goal_y = np.random.uniform((explore_goal_y_init-goal_offset),(explore_goal_y_init+goal_offset))
		    explore_goal = int(((explore_goal_y - pt.y)/resolution)*width + ((explore_goal_x - pt.x)/resolution))   
		  
		  rospy.loginfo("explore_goal_x: (%s)", explore_goal_x)
		  rospy.loginfo("explore_goal_y: (%s)", explore_goal_y)
		  #sanity = (maparea[maparea.index(0)] == costmap[explore_goal])
		  #rospy.loginfo("Sanity Check: (%s)", sanity)
		  if ((np.absolute(explore_goal_x-prev_goal_x) > 0.5) and (np.absolute(explore_goal_y-prev_goal_y) > 0.5)):
		    break

		maparea = []
	  
	  prev_goal_x = explore_goal_x
	  prev_goal_y = explore_goal_y
	  
	  
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
	  
        
        
        

        #listener = tf.TransformListener()
        #rate = rospy.Rate(10.0)
        #rospy.loginfo("Waiting for TF transforms")
        #while not (listener.frameExists("/tag_0") and listener.frameExists("/map")):
            #continue
        #rospy.sleep(1)
        #if (listener.frameExists("/tag_0") and listener.frameExists("/map")): 
            #try:
                ## Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
                #(tag_position,tag_quaternion) = listener.lookupTransform("/map", "/tag_0", rospy.Time(0))
                #rospy.loginfo("The location of tag_0 in world coordinates is")
                #print tag_position 
                #print tag_quaternion
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                #print "Transform Exception Reached!"

       

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting the program.")

