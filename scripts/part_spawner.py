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
        self.path = self.rospack.get_path('bcr_bot')+"/urdf/"
        self.part = self.path+"bcr_bot.xacro"

        self.spawn_model = rospy.ServiceProxy(
            "/gazebo/spawn_urdf_model", SpawnModel)
        self.delete_model = rospy.ServiceProxy(
            "/gazebo/delete_model", DeleteModel)
        self.model_state = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState)

    def checkModel(self):
        res = self.model_state("part", "world")
        return res.success

    def getPosition(self):
        res = self.model_state("part", "world")
        return res.pose.position.z

    def spawnModel(self):
        part = self.part
        with open(part, "r") as f:
            part_urdf = f.read()

        quat = tf.transformations.quaternion_from_euler(0, 0, 0)
        orient = Quaternion(quat[0], quat[1], quat[2], quat[3])
        pose = Pose(Point(x=0, y=-0.55, z=0.75), orient)
        self.spawn_model("part", part_urdf, '', pose, 'world')
        rospy.sleep(1)

    def deleteModel(self):
        self.delete_model("part")
        rospy.sleep(1)

    def shutdown_hook(self):
        self.deleteModel()
        print("Shutting down")


if __name__ == "__main__":
    print("Waiting for gazebo services...")
    rospy.init_node("part_spawner")
    rospy.wait_for_service("/gazebo/delete_model")
    rospy.wait_for_service("/gazebo/spawn_urdf_model")
    rospy.wait_for_service("/gazebo/get_model_state")
    rate = rospy.Rate(15)
    part_spawner = PartSpawner()
    rospy.on_shutdown(part_spawner.shutdown_hook)
    while not rospy.is_shutdown():
        if part_spawner.checkModel() == False:
            part_spawner.spawnModel()
        elif part_spawner.getPosition() < 0.05:
            part_spawner.deleteModel()
        rate.sleep()
