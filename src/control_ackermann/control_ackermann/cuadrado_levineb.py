#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class CuadradoLevineB(Node):
    def __init__(self):
        super().__init__('cuadrado_levineb')

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.pub_drive = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.x = None
        self.y = None
        self.yaw = None

        # Ruta cerrada desde (0,0) hasta volver a (0,0)
        # Ajustes:
        # - Primera esquina empieza más tarde.
        # - Cuarta esquina empieza un poco antes.
        # - Se agregan puntos de salida para suavizar las curvas.
        self.waypoints = [
            (8.7, 0.0),       # más tarde que antes, evita girar muy temprano
            (9.35, 1.6),      # salida de primera curva
            (9.35, 8.15),     # antes de segunda esquina
            (7.7, 8.5),       # salida segunda curva
            (-13.2, 8.5),     # antes de tercera esquina
            (-13.55, 6.8),    # salida tercera curva
            (-13.55, 0.15),   # cuarta esquina un poco antes
            (-11.6, -0.25),   # salida cuarta curva
            (0.0, 0.0),       # regreso al inicio
        ]

        self.wp_index = 0

        self.speed_straight = 1.0
        self.speed_curve = 0.45

        self.steering_gain = 0.65
        self.max_steering = 0.32

        self.distance_tolerance = 0.35
        self.slow_distance = 2.0

        self.timer = self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def quaternion_to_yaw(self, x, y, z, w):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_drive(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = speed
        msg.drive.steering_angle = steering
        self.pub_drive.publish(msg)

    def control_loop(self):
        if self.x is None or self.y is None or self.yaw is None:
            return

        if self.wp_index >= len(self.waypoints):
            self.publish_drive(0.0, 0.0)
            return

        target_x, target_y = self.waypoints[self.wp_index]

        dx = target_x - self.x
        dy = target_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)

        desired_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(desired_yaw - self.yaw)

        steering = self.steering_gain * yaw_error
        steering = max(-self.max_steering, min(self.max_steering, steering))

        if distance < self.distance_tolerance:
            self.get_logger().info(
                f'Waypoint {self.wp_index + 1} alcanzado: '
                f'x={target_x:.2f}, y={target_y:.2f}'
            )
            self.wp_index += 1
            return

        speed = self.speed_straight

        if distance < self.slow_distance or abs(yaw_error) > math.radians(15):
            speed = self.speed_curve

        self.publish_drive(speed, steering)


def main(args=None):
    rclpy.init(args=args)
    nodo = CuadradoLevineB()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()