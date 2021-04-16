#!/usr/bin/env python3
# coding=utf8

import rospy

from gazebo_msgs.msg import ModelState, ModelStates
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler

mavros_ = "/mavros{}"
N = 2
YAW_SHIFT = 0.

x_list = y_list = z_list = [None] * N
pose_x = pose_y = pose_z = 0
orientation_x = orientation_y = orientation_z = orientation_w = 0
linear_twist = None
PS = None
gz_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
gz_msg = ModelState()
teleported = False


def main_loop():
    freq = 60
    rate = rospy.Rate(freq)

    while not rospy.is_shutdown():
        pass
        rate.sleep()


def subscribe_state():
    rospy.Subscriber("/gazebo/model_states", ModelStates, _gz_pose_cb)


def _pose_cb(_pose, idx):
    x_list[idx] = _pose.pose.position.x
    y_list[idx] = _pose.pose.position.y
    z_list[idx] = _pose.pose.position.z


def _gz_pose_cb(_states):
    global pose_x, pose_y, pose_z, orientation_w, orientation_x, orientation_y, orientation_z, linear_twist, \
        teleported

    x = y = z = q_x = q_y = q_z = q_w = c = 0
    for pose in _states.pose[2:]:
        # if not teleported:
        x += pose.position.x
        y += pose.position.y
        z += pose.position.z

        q_x += pose.orientation.x
        q_y += pose.orientation.y
        q_z += pose.orientation.z
        q_w += pose.orientation.w

        # q_x = pose.orientation.x
        # q_y = pose.orientation.y
        # q_z = pose.orientation.z
        # q_w = pose.orientation.w

        c += 1
    # if not teleported:
    pose_x = x / c
    pose_y = y / c
    pose_z = z / c
    pose_point = Point()
    pose_point.x = pose_x
    pose_point.y = pose_y
    pose_point.z = pose_z

    orientation_x = q_x / c
    orientation_y = q_y / c
    orientation_z = q_z / c
    orientation_w = q_w / c

    # orientation_x = q_x
    # orientation_y = q_y
    # orientation_z = q_z
    # orientation_w = q_w

    linear_twist_x = linear_twist_y = linear_twist_z = 0
    angular_twist_x = angular_twist_y = angular_twist_z = 0
    for twist in _states.twist[2:]:
        linear_twist_x += twist.linear.x
        linear_twist_y += twist.linear.y
        linear_twist_z += twist.linear.z
        angular_twist_z += twist.angular.z
        angular_twist_z += twist.angular.z
        angular_twist_z += twist.angular.z

    # PS = _states.pose[2]

    gz_msg.model_name = "barycenter"
    orientation_list = [orientation_x, orientation_y, orientation_z, orientation_w]
    roll, pitch, yaw = euler_from_quaternion(orientation_list)
    # gz_msg.pose.position = _states.pose[2].position if teleported else pose_point
    gz_msg.pose.position = pose_point
    gz_msg.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw + YAW_SHIFT))
    # gz_msg.pose.orientation = Quaternion(*quaternion_from_euler(roll, pitch, yaw))
    gz_msg.twist.linear.x = linear_twist_x / c
    gz_msg.twist.linear.y = linear_twist_y / c
    gz_msg.twist.linear.z = linear_twist_z / c
    # print(gz_msg)
    gz_pub.publish(gz_msg)
    # if not teleported:
    #     print(gz_msg)
    # teleported = True


if __name__ == '__main__':
    rospy.init_node("follower_node")

    subscribe_state()

    main_loop()
    rospy.spin()
