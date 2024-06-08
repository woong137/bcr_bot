#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import threading


class FleetManager:
    def __init__(self):
        self.bot_zones = {
            "bcr_bot_0": "S1",
            "bcr_bot_1": "S2",
            "bcr_bot_2": "S3"
        }
        rospy.init_node('fleet_management', anonymous=False)

    def publish_target_zone(self, bot_name, target_zone):
        topic_name = f"/{bot_name}/target_zone"
        pub = rospy.Publisher(topic_name, String, queue_size=10)
        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            rospy.loginfo(f"Publishing to {topic_name}: {target_zone}")
            pub.publish(target_zone)
            rate.sleep()

    def main(self):
        threads = []
        for bot_name, zone in self.bot_zones.items():
            thread = threading.Thread(
                target=self.publish_target_zone, args=(bot_name, zone))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()

if __name__ == '__main__':
    try:
        fleet_manager = FleetManager()
        fleet_manager.main()
    except rospy.ROSInterruptException:
        pass
