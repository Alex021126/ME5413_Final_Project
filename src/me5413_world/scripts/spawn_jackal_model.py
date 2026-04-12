#!/usr/bin/env python3

import math
import time

import rospy
from gazebo_msgs.srv import GetWorldProperties, SpawnModel
from geometry_msgs.msg import Point, Pose, Quaternion


def quaternion_from_euler(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class JackalSpawner:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "jackal")
        self.model_param = rospy.get_param("~model_param", "/robot_description")
        self.robot_namespace = rospy.get_param("~robot_namespace", "")
        self.reference_frame = rospy.get_param("~reference_frame", "world")
        self.max_retries = int(rospy.get_param("~max_retries", 5))
        self.retry_delay = float(rospy.get_param("~retry_delay", 2.0))
        self.post_spawn_wait = float(rospy.get_param("~post_spawn_wait", 5.0))

        x = float(rospy.get_param("~x", -22.5))
        y = float(rospy.get_param("~y", -7.5))
        z = float(rospy.get_param("~z", 0.1))
        roll = float(rospy.get_param("~roll", 0.0))
        pitch = float(rospy.get_param("~pitch", 0.0))
        yaw = float(rospy.get_param("~yaw", 0.0))

        self.initial_pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=quaternion_from_euler(roll, pitch, yaw),
        )

    def wait_for_service(self, service_name):
        rospy.loginfo("Waiting for service %s", service_name)
        rospy.wait_for_service(service_name)

    def model_exists(self, get_world_properties):
        try:
            world = get_world_properties()
            return self.model_name in world.model_names
        except rospy.ServiceException as exc:
            rospy.logwarn("Failed to query world properties: %s", exc)
            return False

    def run(self):
        self.wait_for_service("/gazebo/get_world_properties")
        self.wait_for_service("/gazebo/spawn_urdf_model")

        get_world_properties = rospy.ServiceProxy(
            "/gazebo/get_world_properties",
            GetWorldProperties,
        )
        spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

        if self.model_exists(get_world_properties):
            rospy.loginfo("Model %s already exists. Skipping spawn.", self.model_name)
            return 0

        model_xml = rospy.get_param(self.model_param)

        for attempt in range(1, self.max_retries + 1):
            if rospy.is_shutdown():
                return 1

            rospy.loginfo("Spawn attempt %d/%d for %s", attempt, self.max_retries, self.model_name)
            status_message = ""

            try:
                response = spawn_model(
                    self.model_name,
                    model_xml,
                    self.robot_namespace,
                    self.initial_pose,
                    self.reference_frame,
                )
                status_message = response.status_message
                if response.success:
                    rospy.loginfo("Spawn request accepted: %s", status_message)
                else:
                    rospy.logwarn("Spawn request returned failure: %s", status_message)
            except rospy.ServiceException as exc:
                status_message = str(exc)
                rospy.logwarn("Spawn request raised exception: %s", status_message)

            deadline = time.monotonic() + self.post_spawn_wait
            while time.monotonic() < deadline and not rospy.is_shutdown():
                if self.model_exists(get_world_properties):
                    rospy.loginfo("Model %s is now present in Gazebo.", self.model_name)
                    return 0
                time.sleep(0.2)

            if "already exist" in status_message.lower():
                rospy.loginfo("Model %s already exists after spawn request.", self.model_name)
                return 0

            rospy.logwarn(
                "Model %s did not appear after attempt %d. Retrying in %.1f s.",
                self.model_name,
                attempt,
                self.retry_delay,
            )
            time.sleep(self.retry_delay)

        rospy.logerr("Failed to spawn %s after %d attempts.", self.model_name, self.max_retries)
        return 1


if __name__ == "__main__":
    rospy.init_node("urdf_spawner")
    raise SystemExit(JackalSpawner().run())
