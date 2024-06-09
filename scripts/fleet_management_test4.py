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
3. 부품을 요청받은 AMR은 가까운 부품을 수령 구역으로 이동 후 부품 수령하기
4. 부품 수령한 후 부품이 필요한 셀로 이동하여 부품 공급
5. 작업이 끝나면 대기 장소로 이동

# AGV

# 기타
1. 각 셀의 부품 현황
2. 사용 가능한 셀
3. 각 셀의 공정 진행도 출력
4. 각 로봇의 임무 출력

'''


class FleetManagerMain():
    '''
    각 셀의 데이터
    {C1: {part: car_wheel, current_count: 0, required_count: 4, is_working: False, work_progress: 0},  ...}

    각 로봇의 데이터
    {bcr_bot_0: {previous_zone: S1, target_zone: D1, is_working: False}, ...}

    넉넉하게 부품을 보유하기 위한 비율
    self.excess_part_ratio = 3

    셀이 부품이 필요한 지 확인
        current_count가 required_count * excess_part_ratio보다 작으면 부품을 요청

    셀에 필요한 부품을 수령할 수 있는 구역들 찾기
    그 구역들과 가장 가까운 임무 없는 로봇에게 부품을 요청

    current_count가 required_count보다 크고 AGV가 셀에 도착하면
        해당 셀의 is_working을 True로 변경
        work_progress를 서서히 증가시키기
        work_progress가 100이 되면 is_working을 False로 변경
        current_count를 current_count - required_count로 변경

    각 셀의 데이터와 각 로봇의 데이터 출력

    '''

    def __init__(self):
        self.excess_part_ratio = 3
        self.cells_needing_parts = {"C1": {"part": "car_wheel", "current_count": 0, "required_count": 4, "is_working": False, "work_progress": 0},
                                    "C2": {"part": "car_wheel", "current_count": 0, "required_count": 4, "is_working": False, "work_progress": 0},
                                    "C3": {"part": "car_wheel", "current_count": 0, "required_count": 4, "is_working": False, "work_progress": 0},
                                    "C4": {"part": "car_wheel", "current_count": 0, "required_count": 4, "is_working": False, "work_progress": 0},
                                    "C5": {"part": "car_wheel", "current_count": 0, "required_count": 4, "is_working": False, "work_progress": 0},
                                    }
        # TODO: previous_zone에 스폰 지점 넣기
        self.robots = {"bcr_bot_0": {"previous_zone": None, "target_zone": None, "is_working": False},
                       "bcr_bot_1": {"previous_zone": None, "target_zone": None, "is_working": False},
                       "bcr_bot_2": {"previous_zone": None, "target_zone": None, "is_working": False},
                       "bcr_bot_3": {"previous_zone": None, "target_zone": None, "is_working": False},
                       "bcr_bot_4": {"previous_zone": None, "target_zone": None, "is_working": False},
                       }
        self.zones = load_zone_coordinates(
            rospkg.RosPack().get_path('bcr_bot')+"/param/zone_coordinates.yaml")
        # TODO: 초기 로봇 위치를 스폰 지점으로 설정
        self.robot_pose = {"bcr_bot_0": {"x": 0, "y": 0, "theta": 0},
                           "bcr_bot_1": {"x": 0, "y": 0, "theta": 0},
                           "bcr_bot_2": {"x": 0, "y": 0, "theta": 0},
                           "bcr_bot_3": {"x": 0, "y": 0, "theta": 0},
                           "bcr_bot_4": {"x": 0, "y": 0, "theta": 0},
                           }

    def main(self):
        self.update_robot_pose()
        self.request_part()
        self.print_data()

    def request_part(self):
        for cell_name, cell_data in self.cells_needing_parts.items():
            if cell_data["current_count"] < cell_data["required_count"] * self.excess_part_ratio:
                target_part = self.cells_needing_parts[cell_name]["part"]
                target_zones = self.find_zones_with_part(
                    self.zones, target_part)
                for target_zone in target_zones:
                    # TODO: 루프 돌때마다 찾지 않도록 수정
                    nearest_free_robot = self.find_nearest_free_robot(
                        target_zone)
                    self.robots[nearest_free_robot]["is_working"] = True
                    self.robots[nearest_free_robot]["target_zone"] = target_zone

    def update_robot_pose(self):
        for robot_namespace, pose in self.robot_pose.items():
            trans, euler = getPose(robot_namespace)
            self.robot_pose[robot_namespace]['x'] = trans[0]
            self.robot_pose[robot_namespace]['y'] = trans[1]
            self.robot_pose[robot_namespace]['theta'] = euler[2]

    def find_nearest_free_robot(self, zone_name):
        nearest_distance = 100000
        nearest_free_robot = None
        for robot_namespace, robot_data in self.robots.items():
            if robot_data["is_working"] == False:
                dx = self.zones[zone_name]['x'] - \
                    self.robot_pose[robot_namespace]['x']
                dy = self.zones[zone_name]['y'] - \
                    self.robot_pose[robot_namespace]['y']
                distance = (dx**2 + dy**2)**0.5
                if distance < nearest_distance:
                    nearest_distance = distance
                    nearest_free_robot = robot_namespace
        return nearest_free_robot

    def find_zones_with_part(zones, desired_part):
        result = []
        for zone_name, attributes in zones.items():
            if attributes.get('part') == desired_part:
                result.append(zone_name)
        return result

    def print_data(self):
        print("Cells needing parts:")
        for cell_name, cell_data in self.cells_needing_parts.items():
            print(f"{cell_name}: {cell_data}")
        print("\nRobots:")
        for robot_namespace, robot_data in self.robots.items():
            print(f"{robot_namespace}: {robot_data}")


class FleetManagerAMR():

    def __init__(self):
        self.target_zones = {
            "bcr_bot_0": FleetManagerMain().robots["bcr_bot_0"]["target_zone"],
            "bcr_bot_1": FleetManagerMain().robots["bcr_bot_1"]["target_zone"],
            "bcr_bot_2": FleetManagerMain().robots["bcr_bot_2"]["target_zone"],
            "bcr_bot_3": FleetManagerMain().robots["bcr_bot_3"]["target_zone"],
            "bcr_bot_4": FleetManagerMain().robots["bcr_bot_4"]["target_zone"],
        }
        self.waiting_zones = ["bcr_bot_1": "W1", "bcr_bot_2": "W2", "bcr_bot_3": "W3", "bcr_bot_4": "W4"]

    def publish_target_zone(self, bot_name, target_zone):
        topic_name = f"/{bot_name}/target_zone"
        pub = rospy.Publisher(topic_name, String, queue_size=10)

        pub.publish(target_zone)

    def main(self):
        threads = []
        for bot_name, zone in self.target_zones.items():
            if zone is None:
                target_zone = self.waiting_zones[bot_name]
            else:
                target_zone = zone
            thread = threading.Thread(
                target=self.publish_target_zone, args=(bot_name, target_zone))
            thread.start()
            threads.append(thread)

        for thread in threads:
            thread.join()


class PartSpawner():

    def __init__(self) -> None:
        self.rospack = rospkg.RosPack()
        self.path = self.rospack.get_path('bcr_bot')+"/models/"
        self.zones = load_zone_coordinates(
            self.rospack.get_path('bcr_bot')+"/param/zone_coordinates.yaml")
        self.distance_threshold = 0.5
        self.angle_threshold = 0.1
        self.robots = ["bcr_bot_0", "bcr_bot_1",
                       "bcr_bot_2", "bcr_bot_3", "bcr_bot_4"]

        self.spawn_model = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel)
        self.model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

    def main(self):
        for robot_namespace in self.robots:
            target_zone = FleetManagerAMR().target_zones[robot_namespace]
            target_zone_value = self.zones[target_zone]
            is_at_zone = self.check_robot_reached_zone(
                robot_namespace, target_zone_value)
            is_supply_zone = target_zone.startswith('S')
            is_demand_zone = target_zone.startswith('D')
            has_model = self.checkModel(
                target_zone_value['part'], robot_namespace)

            if has_model == False and is_at_zone == True and is_supply_zone == True:
                print(
                    f"{robot_namespace}로봇이 {target_zone} 구역에서 부품 {target_zone_value['part']}를 수령했습니다.")
                spawn_point = Point(
                    x=target_zone_value['x'], y=target_zone_value['y'], z=0.5)
                self.spawnModel("car_wheel", robot_namespace,
                                spawn_point, [0, 0, 0])

            elif has_model == True and is_at_zone == True and is_demand_zone == True:
                print(
                    f"{robot_namespace}로봇이 {target_zone} 구역에서 부품 {target_zone_value['part']}를 공급했습니다.")
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
        trans, euler = getPose(robot_namespace)
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


def getPose(robot_namespace):
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


def load_zone_coordinates(yaml_file):
    # zones = {zone_name: {x: x, y: y, theta: theta, part: part}, ...}
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

    fleet_manager_main = FleetManagerMain()
    fleet_manager_amr = FleetManagerAMR()
    part_spawner = PartSpawner()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        fleet_manager_main.main()
        fleet_manager_amr.main()
        part_spawner.main()
        rate.sleep()
