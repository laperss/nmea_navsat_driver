#!/usr/bin/env python
from __future__ import print_function
import rospy
import rosbag
import sys
import os
import tf
import matplotlib.pyplot as plt
from positioner import Positioner


if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    print("Please provide an input file")
    sys.exit(1)

pwd = os.getcwd()
filepath = pwd + '/' + filename

print(filepath)

t_start = 0
t_stop = 550

bag = rosbag.Bag(filepath)

topiclist = ['/fix',
             '/vel',
             '/heading',
             '/magnetic_heading',
             '/time_reference']

position_data = [[] for i in range(6)]
heading_data = [[] for i in range(4)]
magnetic_heading_data = [[] for i in range(4)]
velocity_data = [[] for i in range(3)]

t0 = None
positioner = None
for topic, msg, t in bag.read_messages(topics=topiclist):
    if t0 is None:
        t0 = t.to_sec()
    deltat = t.to_sec() - t0
    if topic == '/fix':
        if positioner == None:
            ORIGIN = (msg.latitude, msg.longitude)
            HEADING = 90.0
            positioner = Positioner(ORIGIN, HEADING)
        X, Y = positioner.get_local_position(msg.latitude, msg.longitude)
        position_data[0].append(deltat)
        position_data[1].append(msg.latitude)
        position_data[2].append(msg.longitude)
        position_data[3].append(msg.altitude)
        position_data[4].append(X)
        position_data[5].append(Y)
    elif topic == '/heading':
        heading_data[0].append(deltat)
        quat = (msg.quaternion.x, msg.quaternion.y,
                msg.quaternion.z, msg.quaternion.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        heading_data[1].append(euler[0])  # Roll
        heading_data[2].append(euler[1])  # Pitch
        heading_data[3].append(euler[2])  # Yaw
    elif topic == '/magnetic_heading':
        magnetic_heading_data[0].append(deltat)
        quat = (msg.quaternion.x, msg.quaternion.y,
                msg.quaternion.z, msg.quaternion.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        magnetic_heading_data[1].append(euler[0])  # Roll
        magnetic_heading_data[2].append(euler[1])  # Pitch
        magnetic_heading_data[3].append(euler[2])  # Yaw
    elif topic == '/vel':
        velocity_data[0].append(deltat)
        velocity_data[1].append(msg.twist.linear.x)
        velocity_data[2].append(msg.twist.linear.y)


if heading_data[0] != [] and heading_data[1] != []:
    plt.figure()
    # plt.plot(heading_data[0], [
    #         i*180/3.14 for i in heading_data[1]], label="Roll")
    # plt.plot(heading_data[0], [
    #         i*180/3.14 for i in heading_data[2]], label="Pitch")
    plt.plot(heading_data[0], [
             i*180/3.14 for i in heading_data[3]], label="Heading")
    plt.plot(magnetic_heading_data[0], [
             i*180/3.14 for i in magnetic_heading_data[3]], label="Magnetic heading")

    plt.title("GPS heading")
    plt.ylabel("Degrees")
    plt.xlabel("Time [s]")
    plt.legend()

if position_data[0] != []:
    plt.figure()
    plt.subplot(1, 2, 1)
    plt.plot(position_data[0], position_data[1], label="Latitude")
    plt.plot(position_data[0], position_data[2], label="Longitude")
    plt.plot(position_data[0], position_data[3], label="Altitude")
    plt.title("GPS position")
    plt.xlabel("Time [s]")
    plt.legend()
    plt.subplot(1, 2, 2)
    plt.plot(position_data[0], position_data[4], label="x")
    plt.plot(position_data[0], position_data[5], label="y")
    plt.xlabel("Time [s]")
    plt.ylabel("Distance [m]")
    plt.legend()

if velocity_data[0] != [] and velocity_data[1] != []:
    plt.figure()
    plt.plot(velocity_data[0], velocity_data[1])
    plt.title("GPS velocity")
    plt.ylabel("Velocity [mps]")
    plt.xlabel("Time [s]")
    plt.legend()

plt.show()
