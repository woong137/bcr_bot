#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import threading


def publish_target_zone(bot_name, target_zone):
    topic_name = f"/{bot_name}/target_zone"
    pub = rospy.Publisher(topic_name, String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing to {topic_name}: {target_zone}")
        pub.publish(target_zone)
        rate.sleep()


if __name__ == '__main__':
    try:
        # 각 로봇의 타겟 구역 설정
        bot_zones = {
            "bcr_bot_0": "S1",
            "bcr_bot_1": "S2",
            "bcr_bot_2": "S3"
        }

        # ROS 초기화
        rospy.init_node('fleet_management', anonymous=False)

        # 각각의 로봇에 대해 스레드를 생성하여 토픽 발행
        threads = []
        for bot_name, zone in bot_zones.items():
            thread = threading.Thread(
                target=publish_target_zone, args=(bot_name, zone))
            thread.start()
            threads.append(thread)

        # 모든 스레드가 종료될 때까지 대기
        for thread in threads:
            thread.join()

    except rospy.ROSInterruptException:
        pass
