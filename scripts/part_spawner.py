#!/usr/bin/env python3

import rospy
import tf
import rospkg
import yaml
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point


class PartSpawner():

    def __init__(self) -> None:
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('bcr_bot')+"/models/"
        self.part = self.path+"car_wheel/model.sdf"
        self.part_name = "car_wheel"
        self.spawn_point = Point(x=0, y=0, z=0.3)
        self.spawn_angle = [0, 0, 0]  # rad
        self.delete_point = Point(x=1, y=1, z=0)
        self.supply_zones = self.load_zone_coordinates(
            self.rospack.get_path('bcr_bot')+"/param/zone_coordinates.yaml", "S")
        self.demend_zones = self.load_zone_coordinates(
            self.rospack.get_path('bcr_bot')+"/param/zone_coordinates.yaml", "D")
        self.distance_threshold = 0.5
        self.angle_threshold = 0.1
        self.robots = ["bcr_bot_0", "bcr_bot_1", "bcr_bot_2"]

        self.spawn_model = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel)
        self.model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

    def main(self):
        for robot_namespace in self.robots:

            is_at_supply_zone, reached_supply_zone = self.check_robot_reached_zone(
                robot_namespace, self.supply_zones)
            if self.checkModel("car_wheel", robot_namespace) == False and is_at_supply_zone == True:
                print(f"Robot {robot_namespace} is at zone {reached_supply_zone}")
                spawn_point = Point(
                    x=self.supply_zones[reached_supply_zone]['x'], y=self.supply_zones[reached_supply_zone]['y'], z=0.5)
                self.spawnModel("car_wheel", robot_namespace,
                                spawn_point, [0, 0, 0])

            is_at_demand_zone, reached_demand_zone = self.check_robot_reached_zone(
                robot_namespace, self.demend_zones)
            if self.checkModel("car_wheel", robot_namespace) == True and is_at_demand_zone == True:
                print(f"Robot {robot_namespace} is at zone {reached_demand_zone}")
                self.deleteModel("car_wheel", robot_namespace)

    def checkModel(self, part, robot_namespace):
        part_name = part + "(" + robot_namespace + ")"
        res = self.model_state(part_name, "world")
        return res.success

    def getDistance(self):
        res = self.model_state(self.part_name, "world")
        return ((self.delete_point.x - res.pose.position.x)**2 +
                (self.delete_point.y - res.pose.position.y)**2 +
                (self.delete_point.z - res.pose.position.z)**2)\
            ** 0.5

    def check_robot_reached_zone(self, robot_namespace, zones):
        trans, euler = self.getPose(robot_namespace)
        for zone_name, zone in zones.items():
            distance = ((zone['x'] - trans[0])**2 +
                        (zone['y'] - trans[1])**2)**0.5
            angle = abs(zone['theta'] - euler[2])
            if distance < self.distance_threshold and angle < self.angle_threshold:
                return True, zone_name  # robot이 zone에 도달했는 지, 그 zone의 이름을 반환

        return False, None

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


    def load_zone_coordinates(yaml_file, zone_startswith):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)
            zones = {}
            for zone_name, coords in data['zones'].items():
                if zone_name.startswith(zone_startswith):
                    zones[zone_name] = {
                        'x': coords['x'],
                        'y': coords['y'],
                        'theta': coords['theta'],
                        'part': coords['part']
                    }
        return zones


if __name__ == "__main__":
    print("Waiting for gazebo services...")
    rospy.init_node("part_spawner")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/get_model_state")
    rate = rospy.Rate(15)
    part_spawner = PartSpawner()
    while not rospy.is_shutdown():
        part_spawner.main()
        rate.sleep()
