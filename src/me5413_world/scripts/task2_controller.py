#!/usr/bin/env python3
import subprocess

import rospy
import actionlib

from std_msgs.msg import Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class Task2Controller:
    def __init__(self):
        rospy.init_node("task2_controller")

        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        rospy.loginfo("Waiting for /move_base action server...")
        self.client.wait_for_server()

        self.unblock_pub = rospy.Publisher("/cmd_unblock", Bool, queue_size=1, latch=True)

        self.move_timeout = float(rospy.get_param("~move_timeout", 60.0))
        self.door_timeout = float(rospy.get_param("~door_timeout", 35.0))
        self.wait_after_unblock = float(rospy.get_param("~wait_after_unblock", 2.0))
        self.kill_default_goal_node = bool(rospy.get_param("~kill_default_goal_node", True))

        self.initial_pose = rospy.get_param("~initial_pose", {})
        self.lower_scan_waypoints = rospy.get_param("~lower_scan_waypoints", [])
        self.transition_waypoints = rospy.get_param("~transition_waypoints", [])
        self.door_candidates = rospy.get_param("~door_candidates", [])
        self.room_goals = rospy.get_param("~room_goals", {})
        self.manual_counts = rospy.get_param("~manual_counts", {})

        rospy.loginfo("Task2 Controller Ready")

    def kill_interfering_node(self):
        if not self.kill_default_goal_node:
            return
        try:
            subprocess.call(
                ["rosnode", "kill", "/me5413_world/goal_publisher_node"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            rospy.loginfo("Attempted to kill /me5413_world/goal_publisher_node")
        except Exception:
            rospy.logwarn("Could not kill /me5413_world/goal_publisher_node")

    def send_goal(self, name, x, y, qz, qw, timeout=None):
        if timeout is None:
            timeout = self.move_timeout

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation.z = float(qz)
        goal.target_pose.pose.orientation.w = float(qw)

        rospy.loginfo(f"Going to [{name}] -> x={x:.3f}, y={y:.3f}")
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(timeout))
        if not finished:
            rospy.logwarn(f"Goal timed out: {name}")
            self.client.cancel_goal()
            return False

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo(f"Reached: {name}")
            return True

        rospy.logwarn(f"Goal failed: {name}, state={state}")
        return False

    def publish_unblock(self):
        rospy.loginfo("Publishing /cmd_unblock = True")
        msg = Bool(data=True)
        for _ in range(5):
            self.unblock_pub.publish(msg)
            rospy.sleep(0.2)
        rospy.sleep(self.wait_after_unblock)

    def choose_target_room(self):
        """
        For now, choose the least frequent digit from manual_counts.
        Replace later if you add OCR/counting logic.
        """
        if not self.room_goals:
            rospy.logwarn("No room_goals configured. Skipping final room selection.")
            return None

        counts = {}
        for k, v in self.manual_counts.items():
            try:
                counts[str(k)] = int(v)
            except Exception:
                pass

        if not counts:
            rospy.logwarn("manual_counts missing or invalid. Falling back to smallest room key.")
            keys = sorted(self.room_goals.keys(), key=lambda x: int(x))
            return keys[0]

        target = min(counts.items(), key=lambda kv: (kv[1], int(kv[0])))[0]
        rospy.loginfo(f"Selected target room digit: {target} using manual_counts={counts}")

        if target not in self.room_goals:
            rospy.logwarn(f"Target digit {target} not found in room_goals. Falling back to first room.")
            keys = sorted(self.room_goals.keys(), key=lambda x: int(x))
            return keys[0]

        return target

    def run_waypoint_list(self, waypoint_list, timeout=None):
        for wp in waypoint_list:
            name = wp.get("name", "unnamed_waypoint")
            ok = self.send_goal(
                name=name,
                x=wp["x"],
                y=wp["y"],
                qz=wp["qz"],
                qw=wp["qw"],
                timeout=timeout,
            )
            if not ok:
                rospy.logerr(f"Stopping because route failed at: {name}")
                return False
        return True

    def run(self):
        rospy.sleep(5.0)
        self.kill_interfering_node()

        # Lower floor route
        if self.lower_scan_waypoints:
            rospy.loginfo("Starting lower floor waypoints...")
            ok = self.run_waypoint_list(self.lower_scan_waypoints, timeout=self.move_timeout)
            if not ok:
                return
        else:
            rospy.logwarn("No lower_scan_waypoints configured.")

        # Unlock door
        self.publish_unblock()

        # Transition route
        if self.transition_waypoints:
            rospy.loginfo("Starting transition waypoints...")
            ok = self.run_waypoint_list(self.transition_waypoints, timeout=max(self.move_timeout, 70.0))
            if not ok:
                return
        else:
            rospy.logwarn("No transition_waypoints configured.")

        # Try candidate doors one by one
        door_success = False
        if self.door_candidates:
            rospy.loginfo("Trying door candidates...")
            for wp in self.door_candidates:
                name = wp.get("name", "unnamed_door")
                ok = self.send_goal(
                    name=name,
                    x=wp["x"],
                    y=wp["y"],
                    qz=wp["qz"],
                    qw=wp["qw"],
                    timeout=self.door_timeout,
                )
                if ok:
                    door_success = True
                    break

            if not door_success:
                rospy.logerr("Could not pass any door candidate.")
                return
        else:
            rospy.logwarn("No door_candidates configured.")

        # Final room
        target_room_key = self.choose_target_room()
        if target_room_key is None:
            rospy.logwarn("No final room selected. Ending after door stage.")
            return

        final_wp = self.room_goals[target_room_key]
        final_name = final_wp.get("name", f"room_{target_room_key}")
        ok = self.send_goal(
            name=final_name,
            x=final_wp["x"],
            y=final_wp["y"],
            qz=final_wp["qz"],
            qw=final_wp["qw"],
            timeout=max(self.move_timeout, 70.0),
        )

        if ok:
            rospy.loginfo("TASK COMPLETE")
        else:
            rospy.logerr("Failed final room goal.")


if __name__ == "__main__":
    try:
        node = Task2Controller()
        node.run()
    except rospy.ROSInterruptException:
        pass