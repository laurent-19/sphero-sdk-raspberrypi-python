#!/usr/bin/env python3
import os
import sys
import time
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RvrStreamingServices

rvr = SpheroRvrObserver()

# Odometry variables
x = 0.0
y = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0

# vx = 0.0
# vy = 0.0
# vth = 0.0

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_broadcaster = tf.TransformBroadcaster()

def imu_handler(imu_data):
    global roll, pitch, yaw
    #print("IMU data: ",imu_data)
    roll = math.radians(imu_data['IMU']['Roll'])
    pitch = math.radians(imu_data['IMU']['Pitch'])
    yaw = math.radians(imu_data['IMU']['Yaw'])  # Convert yaw to radians
    print("Yaw: ", yaw)

def locator_handler(locator_data):
    global x, y
    #print("Locator data: ",locator_data)
    x = locator_data['Locator']['X']
    y = locator_data['Locator']['Y']

def main():
    """ This program demonstrates how to enable multiple sensors to stream and publish odometry.
    """
    global roll, pitch, yaw
    global x, y

    try:
        rospy.init_node('sphero_odometry_publisher')

        rvr.wake()

        # Give RVR time to wake up
        time.sleep(2)

        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.imu,
            handler=imu_handler
        )
        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.locator,
            handler=locator_handler
        )

        rvr.sensor_control.start(interval=250)

        current_time = rospy.Time.now()
        last_time = rospy.Time.now()

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # # compute odometry in a typical way given the velocities of the robot
            # dt = (current_time - last_time).to_sec()
            # delta_x = (vx * cos(yaw) - vy * sin(yaw)) * dt
            # delta_y = (vx * sin(yaw) + vy * cos(yaw)) * dt
            # delta_th = vth * dt

            # x += delta_x
            # y += delta_y
            # yaw += delta_th

            # first, we'll publish the transform over tf
            odom_broadcaster.sendTransform(
                (y, -x, 0.),
                tf.transformations.quaternion_from_euler(0, 0, yaw),
                # tf.transformations.quaternion_from_euler(roll, pitch, yaw),
                current_time,
                "base_link",
                "odom"
            )

            # next, we'll publish the odometry message over ROS
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(y, -x, 0.), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw)))

            # set the velocity
            odom.child_frame_id = "base_link"
            # odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
            odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

            # publish the message
            odom_pub.publish(odom)

            last_time = current_time
            r.sleep()

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

    finally:
        rvr.sensor_control.clear()

        # Delay to allow RVR issue command before closing
        time.sleep(.5)
        
        rvr.close()


if __name__ == '__main__':
    main()
