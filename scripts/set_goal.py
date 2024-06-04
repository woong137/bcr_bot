#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import yaml
from geometry_msgs.msg import Quaternion
import tf.transformations
from std_msgs.msg import String
import math


def load_zone_coordinates(yaml_file, zone_name):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
        zone = data['zones'][zone_name]
        return zone['x'], zone['y'], zone['theta']


def create_quaternion_from_theta(theta_deg):
    theta_rad = math.radians(theta_deg)
    quaternion = tf.transformations.quaternion_from_euler(0, 0, theta_rad)
    return Quaternion(*quaternion)


def movebase_client(x, y, theta_deg):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation = create_quaternion_from_theta(theta_deg)

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_state() == actionlib.GoalStatus.SUCCEEDED


def target_zone_callback(msg):
    zone_name = msg.data
    rospy.loginfo(f"Received target zone: {zone_name}")

    try:
        x, y, theta_deg = load_zone_coordinates('coordinates.yaml', zone_name)
        result = movebase_client(x, y, theta_deg)
        if result:
            rospy.loginfo("목표 지점에 도달했습니다.")
        else:
            rospy.logerr("목표 지점으로의 이동에 실패했습니다.")
    except KeyError:
        rospy.logerr(f"Zone '{zone_name}' is not defined in the YAML file.")
    except rospy.ROSInterruptException:
        rospy.logerr("프로그램이 종료되었습니다.")


if __name__ == '__main__':
    rospy.init_node('set_goal')

    rospy.Subscriber('target_zone', String, target_zone_callback)

    rospy.spin()
