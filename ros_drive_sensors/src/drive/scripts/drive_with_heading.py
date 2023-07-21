#!/usr/bin/env python3
import rospy
import os
import sys
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))

from geometry_msgs.msg import Twist
from sphero_sdk import SpheroRvrObserver
from sphero_sdk import DriveFlagsBitmask

rvr = SpheroRvrObserver()

def cmd_vel_callback(msg):
    # Extract the angular velocity from the Twist message
    angular_velocity = msg.angular.z

    # Convert the angular velocity to degrees (you may need to adjust this conversion based on your requirements)
    heading_degrees = int(angular_velocity * 180 / 3.14159)

    # Delay to allow RVR to drive
    time.sleep(1)

    # Drive the RVR robot with the specified heading
    rvr.drive_with_heading(
        speed=10,  # Valid speed values are 0-255
        heading=heading_degrees,  # Valid heading values are 0-359
        flags=DriveFlagsBitmask.none.value
    )

    # Delay to allow RVR to drive
    time.sleep(1)

    # Print the command
    print(f"Driving RVR with heading: {heading_degrees} degrees")

def main():
    try:
        rospy.init_node('rvr_node')

        # Initialize the RVR robot
        rvr.wake()
        rospy.sleep(2)  # Give RVR time to wake up
        rvr.reset_yaw()

        # Subscribe to the '/cmd_vel' topic
        rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        print('ROS node stopped.')

    finally:
        # Close the connection to the RVR robot
        rvr.close()

if __name__ == '__main__':
    main()
