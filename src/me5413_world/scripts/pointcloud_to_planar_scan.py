#!/usr/bin/env python3

import math

import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import Imu, LaserScan, PointCloud2


def euler_from_quaternion(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class PointCloudToPlanarScan:
    def __init__(self):
        self.cloud_topic = rospy.get_param("~cloud_topic", "/mid/points")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.scan_topic = rospy.get_param("~scan_topic", "/front/scan")
        self.min_height = rospy.get_param("~min_height", -0.20)
        self.max_height = rospy.get_param("~max_height", 0.20)
        self.min_range = rospy.get_param("~min_range", 0.50)
        self.max_range = rospy.get_param("~max_range", 12.0)
        self.angle_min = rospy.get_param("~angle_min", -math.pi)
        self.angle_max = rospy.get_param("~angle_max", math.pi)
        self.num_beams = max(1, int(rospy.get_param("~num_beams", 720)))
        self.use_inf = rospy.get_param("~use_inf", True)
        self.flat_band_scale = rospy.get_param("~flat_band_scale", 0.5)
        self.low_layer_min_height = rospy.get_param("~low_layer_min_height", self.min_height - 0.25)
        self.low_layer_max_height = rospy.get_param("~low_layer_max_height", self.min_height)
        self.mid_layer_min_height = rospy.get_param("~mid_layer_min_height", self.min_height)
        self.mid_layer_max_height = rospy.get_param("~mid_layer_max_height", self.max_height)
        self.high_layer_min_height = rospy.get_param("~high_layer_min_height", self.max_height)
        self.high_layer_max_height = rospy.get_param("~high_layer_max_height", self.max_height + 0.35)
        self.slope_pitch_enter_threshold = rospy.get_param("~slope_pitch_enter_threshold", 0.12)
        self.slope_pitch_exit_threshold = rospy.get_param("~slope_pitch_exit_threshold", 0.08)
        self.slope_transition_hold_time = rospy.get_param("~slope_transition_hold_time", 1.0)
        self.layer_match_tolerance = rospy.get_param("~layer_match_tolerance", 0.25)
        self.supporting_layers_required = max(2, int(rospy.get_param("~supporting_layers_required", 2)))
        self.flat_support_layers_required = max(2, int(rospy.get_param("~flat_support_layers_required", 2)))
        self.allow_single_layer_low_obstacles_flat = rospy.get_param(
            "~allow_single_layer_low_obstacles_flat",
            True,
        )
        self.allow_single_layer_low_obstacles_slope = rospy.get_param(
            "~allow_single_layer_low_obstacles_slope",
            False,
        )
        self.low_obstacle_max_range = rospy.get_param("~low_obstacle_max_range", 2.5)
        self.low_obstacle_neighbor_diff = rospy.get_param("~low_obstacle_neighbor_diff", 0.20)
        self.low_obstacle_neighbor_window = max(1, int(rospy.get_param("~low_obstacle_neighbor_window", 2)))
        self.low_obstacle_min_neighbors = max(1, int(rospy.get_param("~low_obstacle_min_neighbors", 1)))
        self.current_pitch = 0.0
        self.slope_mode = False
        self.last_stamp = None
        self.last_slope_time = rospy.Time(0)
        self.global_min_height = min(
            self.low_layer_min_height,
            self.mid_layer_min_height,
            self.high_layer_min_height,
        )
        self.global_max_height = max(
            self.low_layer_max_height,
            self.mid_layer_max_height,
            self.high_layer_max_height,
        )

        mid_center = 0.5 * (self.mid_layer_min_height + self.mid_layer_max_height)
        mid_half_span = 0.5 * (self.mid_layer_max_height - self.mid_layer_min_height)
        flat_half_span = max(0.02, mid_half_span * self.flat_band_scale)
        self.flat_min_height = mid_center - flat_half_span
        self.flat_max_height = mid_center + flat_half_span

        self.angle_increment = (self.angle_max - self.angle_min) / float(self.num_beams)
        self.publisher = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        self.imu_subscriber = rospy.Subscriber(
            self.imu_topic,
            Imu,
            self.imu_callback,
            queue_size=1,
        )
        self.subscriber = rospy.Subscriber(
            self.cloud_topic,
            PointCloud2,
            self.cloud_callback,
            queue_size=1,
        )

    def imu_callback(self, imu_msg):
        _, pitch, _ = euler_from_quaternion(
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w,
        )
        self.current_pitch = pitch
        stamp = imu_msg.header.stamp if imu_msg.header.stamp != rospy.Time() else rospy.Time.now()
        abs_pitch = abs(pitch)
        if abs_pitch >= self.slope_pitch_enter_threshold:
            self.last_slope_time = stamp
            new_slope_mode = True
        elif self.slope_mode:
            hold_time = (stamp - self.last_slope_time).to_sec()
            new_slope_mode = (
                abs_pitch >= self.slope_pitch_exit_threshold
                or hold_time < self.slope_transition_hold_time
            )
        else:
            new_slope_mode = False
        if new_slope_mode != self.slope_mode:
            mode_label = "slope-aware multi-layer" if new_slope_mode else "flat fused-layer"
            rospy.loginfo(
                "pointcloud_to_planar_scan switched to %s mode (pitch=%.3f rad)",
                mode_label,
                pitch,
            )
        self.slope_mode = new_slope_mode

    def update_range_bin(self, ranges, angle, distance):
        index = int((angle - self.angle_min) / self.angle_increment)
        if 0 <= index < self.num_beams and distance < ranges[index]:
            ranges[index] = distance

    def get_supported_low_range(self, low_distance, mid_distance, high_distance):
        if not math.isfinite(low_distance):
            return None

        supported = [low_distance]
        for candidate in (mid_distance, high_distance):
            if math.isfinite(candidate) and abs(candidate - low_distance) <= self.layer_match_tolerance:
                supported.append(candidate)

        if len(supported) >= self.supporting_layers_required:
            return min(supported)

        return None

    def get_consistent_range(self, candidates, required_layers):
        finite = sorted(value for value in candidates if math.isfinite(value))
        if len(finite) < required_layers:
            return None

        for start, base in enumerate(finite):
            matched = [base]
            for candidate in finite[start + 1 :]:
                if candidate - base <= self.layer_match_tolerance:
                    matched.append(candidate)
                else:
                    break
            if len(matched) >= required_layers:
                return min(matched)

        return None

    def is_confirmed_low_obstacle(self, index, low_ranges, mid_ranges, high_ranges):
        low_distance = low_ranges[index]
        if not math.isfinite(low_distance) or low_distance > self.low_obstacle_max_range:
            return False

        matches = 0
        start = max(0, index - self.low_obstacle_neighbor_window)
        end = min(self.num_beams, index + self.low_obstacle_neighbor_window + 1)
        for neighbor in range(start, end):
            if neighbor == index:
                continue
            if math.isfinite(mid_ranges[neighbor]) or math.isfinite(high_ranges[neighbor]):
                continue
            neighbor_distance = low_ranges[neighbor]
            if not math.isfinite(neighbor_distance):
                continue
            if abs(neighbor_distance - low_distance) <= self.low_obstacle_neighbor_diff:
                matches += 1

        return matches >= self.low_obstacle_min_neighbors

    def cloud_callback(self, cloud_msg):
        flat_ranges = [math.inf] * self.num_beams
        low_ranges = [math.inf] * self.num_beams
        mid_ranges = [math.inf] * self.num_beams
        high_ranges = [math.inf] * self.num_beams

        for point in point_cloud2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            x, y, z = point
            if z < self.global_min_height or z > self.global_max_height:
                continue

            distance = math.hypot(x, y)
            if distance < self.min_range or distance > self.max_range:
                continue

            angle = math.atan2(y, x)
            if angle < self.angle_min or angle >= self.angle_max:
                continue

            if self.flat_min_height <= z <= self.flat_max_height:
                self.update_range_bin(flat_ranges, angle, distance)
            if self.low_layer_min_height <= z < self.low_layer_max_height:
                self.update_range_bin(low_ranges, angle, distance)
            if self.mid_layer_min_height <= z <= self.mid_layer_max_height:
                self.update_range_bin(mid_ranges, angle, distance)
            if self.high_layer_min_height < z <= self.high_layer_max_height:
                self.update_range_bin(high_ranges, angle, distance)

        if self.slope_mode:
            ranges = []
            for low, mid, high in zip(low_ranges, mid_ranges, high_ranges):
                slope_range = self.get_consistent_range(
                    (low, mid, high),
                    self.supporting_layers_required,
                )
                if slope_range is not None:
                    ranges.append(slope_range)
                elif self.allow_single_layer_low_obstacles_slope and self.is_confirmed_low_obstacle(
                    len(ranges),
                    low_ranges,
                    mid_ranges,
                    high_ranges,
                ):
                    ranges.append(low)
                else:
                    ranges.append(math.inf)
        else:
            ranges = []
            for low, mid, high, flat in zip(low_ranges, mid_ranges, high_ranges, flat_ranges):
                flat_range = self.get_consistent_range(
                    (low, flat, mid, high),
                    self.flat_support_layers_required,
                )
                if flat_range is not None:
                    ranges.append(flat_range)
                elif self.allow_single_layer_low_obstacles_flat and self.is_confirmed_low_obstacle(
                    len(ranges),
                    low_ranges,
                    mid_ranges,
                    high_ranges,
                ):
                    ranges.append(low)
                else:
                    ranges.append(math.inf)

        scan_msg = LaserScan()
        scan_msg.header = cloud_msg.header
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_max
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_time = 0.0
        if self.last_stamp is not None:
            scan_time = max(0.0, (cloud_msg.header.stamp - self.last_stamp).to_sec())
        self.last_stamp = cloud_msg.header.stamp
        scan_msg.scan_time = scan_time
        scan_msg.time_increment = scan_time / float(self.num_beams) if self.num_beams > 0 else 0.0
        scan_msg.range_min = self.min_range
        scan_msg.range_max = self.max_range
        if self.use_inf:
            scan_msg.ranges = [value if math.isfinite(value) else math.inf for value in ranges]
        else:
            scan_msg.ranges = [value if math.isfinite(value) else self.max_range for value in ranges]

        self.publisher.publish(scan_msg)


if __name__ == "__main__":
    rospy.init_node("pointcloud_to_planar_scan")
    PointCloudToPlanarScan()
    rospy.spin()
