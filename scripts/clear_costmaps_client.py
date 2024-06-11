#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty


def clear_costmaps(robot_id):
    service_name = f'bcr_bot_{robot_id}/move_base/clear_costmaps'
    rospy.wait_for_service(service_name)
    try:
        clear_costmaps_service = rospy.ServiceProxy(service_name, Empty)
        clear_costmaps_service()
        rospy.loginfo(
            f"Successfully called clear_costmaps service for {service_name}.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed for {service_name}: {e}")


if __name__ == '__main__':
    rospy.init_node('clear_costmaps_client')

    # Set the number of robots
    number_of_robots = rospy.get_param('~number_of_robots', 5)
    rate = rospy.Rate(1)  # 1 Hz, adjust the rate as needed

    while not rospy.is_shutdown():
        for robot_id in range(number_of_robots):
            clear_costmaps(robot_id)
        rate.sleep()
