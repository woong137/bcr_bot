#!/usr/bin/env python3

import rospy
import tf
import rospkg
import yaml
from std_msgs.msg import String
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point


class FleetManagerAMR:
    def __init__(self, target_zones):
        self.target_zones = target_zones
        self.current_zones = {bot: 0 for bot in self.target_zones.keys()}
        self.publishers = {bot_name: rospy.Publisher(
            f"/{bot_name}/target_zone", String, queue_size=10) for bot_name in self.target_zones.keys()}

    def publish_target_zone(self, bot_name, target_zone):
        self.publishers[bot_name].publish(target_zone)

    def update_target_zones(self, part_spawner):
        for robot_namespace, zones in self.target_zones.items():
            current_index = self.current_zones[robot_namespace]
            target_zone = zones[current_index]
            target_zone_value = part_spawner.zones[target_zone]
            has_model = part_spawner.check_model(
                target_zone_value['part'], robot_namespace)

            if current_index == 0 and has_model:
                self.current_zones[robot_namespace] = 1
            elif current_index == 1 and not has_model:
                self.current_zones[robot_namespace] = 0

    def main(self, part_spawner):
        self.update_target_zones(part_spawner)
        for bot_name, zones in self.target_zones.items():
            current_index = self.current_zones[bot_name]
            target_zone = zones[current_index]
            self.publish_target_zone(bot_name, target_zone)
            print(f"{bot_name} 로봇의 목표 구역: {target_zone}")


class PartSpawner:
    def __init__(self, robots):
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('bcr_bot') + "/models/"
        self.zones = self.load_zone_coordinates(
            self.rospack.get_path('bcr_bot') + "/param/zone_coordinates.yaml")
        self.distance_threshold = 0.11
        self.angle_threshold = 0.1
        self.robots = robots

        self.spawn_model = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel)
        self.model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

    def main(self, fleet_manager_amr):
        for robot_namespace in self.robots:
            target_zone = fleet_manager_amr.target_zones[robot_namespace][
                fleet_manager_amr.current_zones[robot_namespace]]
            target_zone_value = self.zones[target_zone]
            is_at_zone = self.check_robot_reached_zone(
                robot_namespace, target_zone_value)
            is_supply_zone = target_zone.startswith('S')
            is_demand_zone = target_zone.startswith('D')
            has_model = self.check_model(
                target_zone_value['part'], robot_namespace)

            if not has_model and is_at_zone and is_supply_zone:
                rospy.loginfo(
                    f"{robot_namespace} 로봇이 {target_zone} 구역에서 부품 {target_zone_value['part']}를 수령했습니다.")
                spawn_point = Point(
                    x=target_zone_value['x'], y=target_zone_value['y'], z=0.5)
                self.spawn_model_func(
                    target_zone_value['part'], robot_namespace, spawn_point, [0, 0, 0])

            elif has_model and is_at_zone and is_demand_zone:
                rospy.loginfo(
                    f"{robot_namespace} 로봇이 {target_zone} 구역에서 부품 {target_zone_value['part']}를 공급했습니다.")
                self.delete_model_func(
                    target_zone_value['part'], robot_namespace)

    def check_model(self, part, robot_namespace):
        part_name = f"{part}({robot_namespace})"
        try:
            res = self.model_state(part_name, "world")
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def check_robot_reached_zone(self, robot_namespace, zone):
        trans, euler = self.get_pose(robot_namespace)
        if trans is None or euler is None:
            return False
        distance = ((zone['x'] - trans[0]) ** 2 +
                    (zone['y'] - trans[1]) ** 2) ** 0.5
        angle = abs(zone['theta'] - euler[2])
        return distance < self.distance_threshold and angle < self.angle_threshold

    def spawn_model_func(self, part, robot_namespace, spawn_point, spawn_angle):
        with open(self.path + part + "/model.sdf", "r") as f:
            part_sdf = f.read()
        quat = tf.transformations.quaternion_from_euler(
            spawn_angle[0], spawn_angle[1], spawn_angle[2])
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(spawn_point, orient)
        part_name = f"{part}({robot_namespace})"
        try:
            self.spawn_model(part_name, part_sdf, '', pose, 'world')
            rospy.sleep(1)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn model: {e}")

    def delete_model_func(self, part, robot_namespace):
        part_name = f"{part}({robot_namespace})"
        try:
            self.delete_model(part_name)
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to delete model: {e}")

    def get_pose(self, robot_namespace):
        listener = tf.TransformListener()
        try:
            listener.waitForTransform(
                'map', f'{robot_namespace}/base_footprint', rospy.Time(), rospy.Duration(10.0))
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

    ##TODO: 각 로봇의 목표 구역을 새로운 맵에 맞게 설정
    target_zones = {
        "bcr_bot_0": ["S1", "D1", "S2", "D1"],
        "bcr_bot_1": ["S3", "D2", "S4", "D2"],
        "bcr_bot_2": ["S13", "D3", "S14", "D3"],
        "bcr_bot_3": ["S15", "D4", "S16", "D4"],
    }
    robots = list(target_zones.keys())

    fleet_manager_amr = FleetManagerAMR(target_zones)
    part_spawner = PartSpawner(robots)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():

        fleet_manager_amr.main(part_spawner)
        part_spawner.main(fleet_manager_amr)
        rate.sleep()
