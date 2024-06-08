#!/usr/bin/env python3

import rospy
import tf
import rospkg
import yaml
from std_msgs.msg import String
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point
import threading

'''
# AMR
1. 각 셀의 부품 현황 확인
2. 부품이 부족하면 근처에 있는 AMR에게 부품을 요청
3. 부품을 요청받은 AMR은 부품을 공급받은 후 부품이 필요한 셀로 이동하여 부품 공급
4. 작업이 끝나면 대기 장소로 이동

# AGV

# 기타
1. 각 셀의 부품 현황
2. 사용 가능한 셀
3. 각 셀의 공정 진행도 출력
4. 각 로봇의 임무 출력

'''

class FleetManager:

    def __init__(self):
        self.bot_zones = {
            "bcr_bot_0": "D1",
            "bcr_bot_1": "D2",
            "bcr_bot_2": "D3",
            "bcr_bot_3": "D4",
            "bcr_bot_4": "D5",
        }

    def publish_target_zone(self, bot_name, target_zone):
        topic_name = f"/{bot_name}/target_zone"
        pub = rospy.Publisher(topic_name, String, queue_size=10)

        pub.publish(target_zone)

    def main(self):
        threads = []
        for bot_name, zone in self.bot_zones.items():
            thread = threading.Thread(
                target=self.publish_target_zone, args=(bot_name, zone))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()


class PartSpawner():

    def __init__(self) -> None:
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('bcr_bot')+"/models/"
        self.zones = self.load_zone_coordinates(
            self.rospack.get_path('bcr_bot')+"/param/zone_coordinates.yaml")
        self.distance_threshold = 0.5
        self.angle_threshold = 0.1
        self.robots = ["bcr_bot_0", "bcr_bot_1", "bcr_bot_2", "bcr_bot_3", "bcr_bot_4"]

        self.spawn_model = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel)
        self.model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

    def main(self):
        for robot_namespace in self.robots:
            target_zone = FleetManager().bot_zones[robot_namespace]
            target_zone_value = self.zones[target_zone]
            is_at_zone = self.check_robot_reached_zone(
                robot_namespace, target_zone_value)
            is_supply_zone = target_zone.startswith('S')
            is_demand_zone = target_zone.startswith('D')
            has_model = self.checkModel(target_zone_value['part'], robot_namespace)

            if has_model == False and is_at_zone == True and is_supply_zone == True:
                print(f"{robot_namespace}로봇이 {target_zone} 구역에서 부품 {target_zone_value['part']}를 수령했습니다.")
                spawn_point = Point(
                    x=target_zone_value['x'], y=target_zone_value['y'], z=0.5)
                self.spawnModel("car_wheel", robot_namespace,
                                spawn_point, [0, 0, 0])

            elif has_model == True and is_at_zone == True and is_demand_zone == True:
                print(f"{robot_namespace}로봇이 {target_zone} 구역에서 부품 {target_zone_value['part']}를 공급했습니다.")
                self.deleteModel("car_wheel", robot_namespace)

    def checkModel(self, part, robot_namespace):
        part_name = f"{part}({robot_namespace})"
        try:
            res = self.model_state(part_name, "world")
            return res.success
        except rospy.ServiceException as e:
            # rospy.logerr(f"Service call failed: {e}")
            return False

    def check_robot_reached_zone(self, robot_namespace, zone):
        trans, euler = self.getPose(robot_namespace)
        distance = ((zone['x'] - trans[0])**2 +
                    (zone['y'] - trans[1])**2)**0.5
        angle = abs(zone['theta'] - euler[2])
        if distance < self.distance_threshold and angle < self.angle_threshold:
            return True
        else:
            return False

    def spawnModel(self, part, robot_namespace, spawn_point, spawn_angle):
        with open(self.path + part + "/model.sdf", "r") as f:
            part_sdf = f.read()

        quat = tf.transformations.quaternion_from_euler(
            spawn_angle[0], spawn_angle[1], spawn_angle[2])
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(spawn_point, orient)

        part_name = part + "(" + robot_namespace + ")"
        self.spawn_model(part_name, part_sdf, '', pose, 'world')
        rospy.sleep(1)

    def deleteModel(self, part, robot_namespace):

        part_name = part + "(" + robot_namespace + ")"
        self.delete_model(part_name)
        rospy.sleep(1)

    def getPose(self, robot_namespace):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform(
                'map', f'{robot_namespace}/base_footprint', rospy.Time(), rospy.Duration(5.0))
            (trans, rot) = listener.lookupTransform(
                'map', f'{robot_namespace}/base_footprint', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            return trans, euler

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF error: %s", e)
            return None, None

    def load_zone_coordinates(self, yaml_file):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
            zones = {}
            for zone_name, coords in data['zones'].items():
                zones[zone_name] = {
                    'x': coords['x'],
                    'y': coords['y'],
                    'theta': coords['theta'],
                    'part': coords['part']
                }
        return zones


if __name__ == "__main__":
    print("Waiting for gazebo services...")
    rospy.init_node("fleet_management")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/get_model_state")

    fleet_manager = FleetManager()
    part_spawner = PartSpawner()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        fleet_manager.main()
        part_spawner.main()
        rate.sleep()
