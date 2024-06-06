#!/usr/bin/env python3

import rospy
import tf
import rospkg
import random
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

        self.spawn_model = rospy.ServiceProxy(
            "/gazebo/spawn_sdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel)
        self.model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

    def checkModel(self):
        res = self.model_state(self.part_name, "world")
        return res.success

    def getPosition(self):
        res = self.model_state(self.part_name, "world")
        return res.pose.position.z

    def getDistance(self):
        res = self.model_state(self.part_name, "world")
        return ((self.delete_point.x - res.pose.position.x)**2 +
                (self.delete_point.y - res.pose.position.y)**2 +
                (self.delete_point.z - res.pose.position.z)**2)\
            ** 0.5

    def spawnModel(self):
        part = self.part
        with open(part, "r") as f:
            part_sdf = f.read()

        quat = tf.transformations.quaternion_from_euler(
            self.spawn_angle[0], self.spawn_angle[1], self.spawn_angle[2])
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(self.spawn_point, orient)
        self.spawn_model(self.part_name, part_sdf, '', pose, 'world')
        rospy.sleep(1)

    def deleteModel(self):
        self.delete_model(self.part_name)
        rospy.sleep(1)

    def shutdown_hook(self):
        self.deleteModel()
        print("Shutting down")


if __name__ == "__main__":
    print("Waiting for gazebo services...")
    rospy.init_node("part_spawner")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    rospy.wait_for_service("/gazebo/get_model_state")
    rate = rospy.Rate(15)
    part_spawner = PartSpawner()
    rospy.on_shutdown(part_spawner.shutdown_hook)
    while not rospy.is_shutdown():
        if part_spawner.checkModel() == False:
            part_spawner.spawnModel()
        elif part_spawner.getDistance() < 0.5:
            part_spawner.deleteModel()
        rate.sleep()
