#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def main():
    try:
        rospy.init_node('twist_publisher_node')

        # Create a publisher for the '/cmd_vel' topic
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.sleep(2)  # Delay to allow subscribers to connect

        # Create a Twist message with the desired angular velocity
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        twist_msg.angular.z = 0.1  # Adjust the angular velocity as needed

        rate = rospy.Rate(10)  # Publish at a rate of 10 Hz

        while not rospy.is_shutdown():
            pub.publish(twist_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        print('ROS node stopped.')

if __name__ == '__main__':
    main()
