#!/usr/bin/env python3
import rospy
import os
import sys
import time
import math
from math import sin, cos, pi

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import RvrStreamingServices
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

rvr = SpheroRvrObserver()

# Odometry variables
x = 0.0
y = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0
vx = 0.0
vy = 0.0
wz = 0.0

# Odometry publisher
odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
# Transform publisher
odom_broadcaster = tf.TransformBroadcaster()

# Define data handlers
def imu_handler(imu_data):
    global roll, pitch, yaw
    roll = math.radians(imu_data['IMU']['Roll'])
    pitch = math.radians(imu_data['IMU']['Pitch'])
    yaw = math.radians(imu_data['IMU']['Yaw'])
    print("Yaw: ", yaw)

def locator_handler(locator_data):
    global x, y
    x = locator_data['Locator']['X']
    y = locator_data['Locator']['Y']

def velocity_handler(velocity_data):
    global vx,vy
    vx = velocity_data['Velocity']['X']
    vy = velocity_data['Velocity']['Y']

def gyro_handler(gyro_data):
    global wz
    wz = gyro_data['Gyroscope']['Z'] * 3.14 / 180

# Define callback function for robot control
# RC Controller: Input: linear/angular velocities - Twist msg
#                Output: commands for right/left wheels velocities, diff drive
def cmd_vel_callback(twist):
    try:
        print("Twist cmd: ", twist)
        # This is published by the nav_stack node move_base - Twist(linear,angular)
        linear_x = twist.linear.x  # Extract linear velocity along the Y-axis from the Twist message
        angular_z = twist.angular.z  # Extract angular velocity around the Z-axis from the Twist message

        # Convert angular velocity from degrees/s to radians/s
        angular_z_rad = int(math.degrees(angular_z))

        print("Received twist message:")
        print("Linear velocity (m/s):", linear_x)
        print("Angular velocity (rad/s):", angular_z_rad)

        # Set the linear and angular velocities to drive the robot using the drive_rc_si_units function
        rvr.drive_rc_si_units(
            linear_velocity=linear_x,
            yaw_angular_velocity=angular_z_rad,
            flags=0
        )

    except Exception as e:
        print(f"Error occurred in cmd_vel_callback: {str(e)}")


def main():
    try:
        rospy.init_node('rvr_node')

        rvr.wake()
        time.sleep(2)
        rvr.reset_yaw()

        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.imu,
            handler=imu_handler
        )
        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.locator,
            handler=locator_handler
        )
        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.velocity,
            handler=velocity_handler
        )
        rvr.sensor_control.add_sensor_data_handler(
            service=RvrStreamingServices.gyroscope,
            handler=gyro_handler
        )
        rvr.sensor_control.start(interval=250)

        current_time = rospy.Time.now()

        #Subscribe to the nav_stack controller for command velocities
        rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

        r = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            # Publish odometry transforms using x,y position and yaw orientation
            # The robot moves on Y axis, but nav_stack expect X-axis movement
            # Rotate the XY plane ==>  x -> y; y -> -x
            odom_broadcaster.sendTransform(
                (y, -x, 0.),
                tf.transformations.quaternion_from_euler(0, 0, yaw),
                current_time,
                "base_link",
                "odom"
            )
            
            # Create odometry message using the: * x,y position - Locator
            #                                    * yaw - IMU
            #                                    * X,Y linear vel. - Accelerometer
            #                                    * Z angular vel. - Gyroscope
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.pose.pose = Pose(Point(y, -x, 0.), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw)))
            odom.child_frame_id = "base_link"
            odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, wz))

            # Publish omometry msg
            odom_pub.publish(odom)

            r.sleep()

    except rospy.ROSInterruptException:
        print('ROS node stopped.')

    finally:
        rvr.sensor_control.clear()
        time.sleep(.5)
        rvr.close()

if __name__ == '__main__':
    main()
