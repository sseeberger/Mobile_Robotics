import rospy
import tf

if __name__ == '__main__':
    try:
      
        rospy.init_node('pos_listener', anonymous=False)

        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not (listener.frameExists("/base_link") and listener.frameExists("/map")):
            continue
        rospy.sleep(1)
        if (listener.frameExists("/base_link") and listener.frameExists("/map")): 
            try:
                # Store the turtlebot's starting position in the global coordinate system in the map_position and map_quaternion variables
                (map_position,map_quaternion) = listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                print map_position 
                print map_quaternion
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "Transform Exception Reached!"


    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting the program.")

