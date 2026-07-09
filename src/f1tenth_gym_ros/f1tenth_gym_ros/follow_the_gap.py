import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class FollowTheGap(Node):
    def __init__(self):
        super().__init__('follow_the_gap')

        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
        self.pub_drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.max_range = 10.0
        self.smoothing_window = 5

        self.bubble_radius = 24
        self.close_obstacle_threshold = 2.0
        self.critical_obstacle_threshold = 1.0

        self.front_angle_deg = 140
        self.max_steering = 0.42

        self.speed_straight = 5.0
        self.speed_medium = 3.0
        self.speed_curve = 1.1
        self.speed_danger = 0.45

        self.finished = False
        self.x = None
        self.y = None
        self.start_x = None
        self.start_y = None

        self.armed_lap_counter = False
        self.rearm_distance = 8.0
        self.finish_radius = 1.5

        self.lap_count = 0
        self.max_laps = 10

        self.start_time = time.time()
        self.last_lap_time = self.start_time
        self.best_lap_time = None

        self.last_steering = 0.0

        self.get_logger().info('Follow The Gap estable iniciado.')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.start_time = time.time()
            self.last_lap_time = self.start_time

            self.get_logger().info(
                f'Punto inicial detectado: x={self.start_x:.2f}, y={self.start_y:.2f}'
            )
            return

        distance_from_start = math.hypot(self.x - self.start_x, self.y - self.start_y)

        if not self.armed_lap_counter and distance_from_start > self.rearm_distance:
            self.armed_lap_counter = True
            self.get_logger().info('Contador de vuelta armado.')

        if self.armed_lap_counter and distance_from_start < self.finish_radius:
            self.lap_count += 1
            self.armed_lap_counter = False

            now = time.time()
            lap_time = now - self.last_lap_time
            total_time = now - self.start_time
            self.last_lap_time = now

            if self.best_lap_time is None or lap_time < self.best_lap_time:
                self.best_lap_time = lap_time

            self.get_logger().info(
                f'Vuelta {self.lap_count}/{self.max_laps} completada | '
                f'Tiempo vuelta: {lap_time:.2f} s | '
                f'Mejor vuelta: {self.best_lap_time:.2f} s | '
                f'Tiempo total: {total_time:.2f} s'
            )

            if self.lap_count >= self.max_laps:
                self.finished = True
                self.get_logger().info('10 vueltas completadas. Deteniendo vehículo.')
                self.publish_drive(0.0, 0.0)

    def scan_callback(self, scan_msg):
        if self.finished:
            self.publish_drive(0.0, 0.0)
            return

        ranges = np.array(scan_msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        ranges = np.clip(ranges, 0.0, self.max_range)

        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        total_points = len(ranges)
        center_index = total_points // 2

        front_angle_rad = math.radians(self.front_angle_deg)
        half_window = int(front_angle_rad / angle_increment / 2)

        start = max(0, center_index - half_window)
        end = min(total_points, center_index + half_window)

        proc_ranges = ranges[start:end].copy()
        proc_ranges = self.smooth_ranges(proc_ranges)
        proc_ranges = self.apply_safety_bubbles(proc_ranges)

        gap_start, gap_end = self.find_max_gap(proc_ranges)

        if gap_start == gap_end:
            self.publish_drive(0.0, 0.0)
            return

        best_index = self.find_best_point(proc_ranges, gap_start, gap_end)

        lidar_index = start + best_index
        steering_angle = angle_min + lidar_index * angle_increment
        steering_angle = max(-self.max_steering, min(self.max_steering, steering_angle))

        steering_angle = 0.75 * steering_angle + 0.25 * self.last_steering
        self.last_steering = steering_angle

        very_front_window = ranges[
            max(0, center_index - 12):min(len(ranges), center_index + 12)
        ]

        front_window = ranges[
            max(0, center_index - 28):min(len(ranges), center_index + 28)
        ]

        wide_front_window = ranges[
            max(0, center_index - 70):min(len(ranges), center_index + 70)
        ]

        very_front_distance = np.min(very_front_window)
        min_front_distance = np.min(front_window)
        avg_wide_front_distance = np.mean(wide_front_window)

        speed = self.calculate_speed(
            steering_angle,
            min_front_distance,
            avg_wide_front_distance,
            very_front_distance
        )

        self.publish_drive(speed, steering_angle)

    def smooth_ranges(self, ranges):
        kernel = np.ones(self.smoothing_window) / self.smoothing_window
        return np.convolve(ranges, kernel, mode='same')

    def apply_safety_bubbles(self, ranges):
        protected_ranges = ranges.copy()

        close_indices = np.where(
            (protected_ranges > 0.0) &
            (protected_ranges < self.close_obstacle_threshold)
        )[0]

        for idx in close_indices:
            distance = protected_ranges[idx]

            if distance < 0.8:
                radius = 46
            elif distance < self.critical_obstacle_threshold:
                radius = 40
            elif distance < 1.4:
                radius = 34
            elif distance < 1.8:
                radius = 28
            else:
                radius = self.bubble_radius

            bubble_start = max(0, idx - radius)
            bubble_end = min(len(protected_ranges), idx + radius)
            protected_ranges[bubble_start:bubble_end] = 0.0

        return protected_ranges

    def find_max_gap(self, ranges):
        max_start = 0
        max_end = 0
        current_start = None

        for i, value in enumerate(ranges):
            if value > 0.8:
                if current_start is None:
                    current_start = i
            else:
                if current_start is not None:
                    if i - current_start > max_end - max_start:
                        max_start = current_start
                        max_end = i
                    current_start = None

        if current_start is not None:
            if len(ranges) - current_start > max_end - max_start:
                max_start = current_start
                max_end = len(ranges)

        return max_start, max_end

    def find_best_point(self, ranges, gap_start, gap_end):
        gap_length = gap_end - gap_start

        if gap_length <= 0:
            return gap_start

        safety_margin = min(14, gap_length // 4)

        safe_start = gap_start + safety_margin
        safe_end = gap_end - safety_margin

        if safe_start >= safe_end:
            safe_start = gap_start
            safe_end = gap_end

        gap_center = (safe_start + safe_end) // 2

        gap = ranges[safe_start:safe_end]

        if len(gap) == 0:
            return gap_center

        best_local = np.argmax(gap)
        farthest = safe_start + best_local

        return int(0.82 * gap_center + 0.18 * farthest)

    def calculate_speed(self, steering_angle, front_distance, avg_front_distance, very_front_distance):
        abs_steer = abs(steering_angle)

        if very_front_distance < 0.75:
            return 0.0

        if very_front_distance < 1.05:
            return 0.35

        if front_distance < 1.4:
            return 0.7

        if front_distance < 1.8:
            return self.speed_danger

        if abs_steer < 0.07 and avg_front_distance > 5.5:
            return self.speed_straight

        if abs_steer < 0.15 and avg_front_distance > 3.8:
            return 3.6

        if abs_steer < 0.25:
            return self.speed_medium

        return self.speed_curve

    def publish_drive(self, speed, steering_angle):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.pub_drive.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = AckermannDriveStamped()
        stop.drive.speed = 0.0
        stop.drive.steering_angle = 0.0
        node.pub_drive.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()