# MIT License
#
# Copyright (c) 2020 Hongrui Zheng
#
# Modificado para soportar 3 agentes:
#   - ego_racecar
#   - opp_racecar
#   - opp_racecar_2
#
# Uso esperado:
#   num_agent: 3
#   sx, sy, stheta
#   sx1, sy1, stheta1
#   sx2, sy2, stheta2

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler


class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        # Ego parameters
        self.declare_parameter('ego_namespace', '')
        self.declare_parameter('ego_odom_topic', '')
        self.declare_parameter('ego_opp_odom_topic', '')
        self.declare_parameter('ego_scan_topic', '')
        self.declare_parameter('ego_drive_topic', '')

        # Opponent 1 parameters
        self.declare_parameter('opp_namespace', '')
        self.declare_parameter('opp_odom_topic', '')
        self.declare_parameter('opp_ego_odom_topic', '')
        self.declare_parameter('opp_scan_topic', '')
        self.declare_parameter('opp_drive_topic', '')

        # Opponent 2 parameters
        self.declare_parameter('opp2_namespace', 'opp_racecar_2')
        self.declare_parameter('opp2_odom_topic', 'odom')
        self.declare_parameter('opp2_ego_odom_topic', 'opp_odom')
        self.declare_parameter('opp2_scan_topic', 'opp2_scan')
        self.declare_parameter('opp2_drive_topic', 'opp2_drive')

        # General parameters
        self.declare_parameter('scan_distance_to_base_link', 0.0)
        self.declare_parameter('scan_fov', 0.0)
        self.declare_parameter('scan_beams', 0)

        self.declare_parameter('map_path', '')
        self.declare_parameter('map_img_ext', '')

        self.declare_parameter('num_agent', 0)

        # Ego starting pose
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)

        # Opponent 1 starting pose
        self.declare_parameter('sx1', 0.0)
        self.declare_parameter('sy1', 0.0)
        self.declare_parameter('stheta1', 0.0)

        # Opponent 2 starting pose
        self.declare_parameter('sx2', 0.0)
        self.declare_parameter('sy2', 0.0)
        self.declare_parameter('stheta2', 0.0)

        self.declare_parameter('kb_teleop', False)

        # Check number of agents
        num_agents = self.get_parameter('num_agent').value

        if type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        if num_agents < 1 or num_agents > 3:
            raise ValueError('num_agents should be 1, 2 or 3.')

        self.num_agents = num_agents
        self.has_opp = num_agents >= 2
        self.has_opp2 = num_agents >= 3

        # Environment backend
        self.env = gym.make(
            'f110_gym:f110-v0',
            map=self.get_parameter('map_path').value,
            map_ext=self.get_parameter('map_img_ext').value,
            num_agents=num_agents
        )

        # LaserScan parameters
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        self.angle_inc = scan_fov / scan_beams
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value

        # Namespaces and topics
        self.ego_namespace = self.get_parameter('ego_namespace').value
        self.ego_scan_topic = self.get_parameter('ego_scan_topic').value
        self.ego_drive_topic = self.get_parameter('ego_drive_topic').value
        self.ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value

        if self.has_opp:
            self.opp_namespace = self.get_parameter('opp_namespace').value
            self.opp_scan_topic = self.get_parameter('opp_scan_topic').value
            self.opp_drive_topic = self.get_parameter('opp_drive_topic').value
            self.opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            self.ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
            self.opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value

        if self.has_opp2:
            self.opp2_namespace = self.get_parameter('opp2_namespace').value
            self.opp2_scan_topic = self.get_parameter('opp2_scan_topic').value
            self.opp2_drive_topic = self.get_parameter('opp2_drive_topic').value
            self.opp2_odom_topic = self.opp2_namespace + '/' + self.get_parameter('opp2_odom_topic').value
            self.opp2_ego_odom_topic = self.opp2_namespace + '/' + self.get_parameter('opp2_ego_odom_topic').value

        # Starting poses
        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value

        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_drive_published = False
        self.ego_collision = False

        poses = [[sx, sy, stheta]]

        if self.has_opp:
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value

            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_drive_published = False
            self.opp_collision = False

            poses.append([sx1, sy1, stheta1])

        if self.has_opp2:
            sx2 = self.get_parameter('sx2').value
            sy2 = self.get_parameter('sy2').value
            stheta2 = self.get_parameter('stheta2').value

            self.opp2_pose = [sx2, sy2, stheta2]
            self.opp2_speed = [0.0, 0.0, 0.0]
            self.opp2_requested_speed = 0.0
            self.opp2_steer = 0.0
            self.opp2_drive_published = False
            self.opp2_collision = False

            poses.append([sx2, sy2, stheta2])

        self.obs, _, self.done, _ = self.env.reset(np.array(poses))
        self._update_sim_state()

        # Timers
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        self.timer = self.create_timer(0.004, self.timer_callback)

        # TF broadcaster
        self.br = TransformBroadcaster(self)

        # Publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, self.ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, self.ego_odom_topic, 10)

        if self.has_opp:
            self.opp_scan_pub = self.create_publisher(LaserScan, self.opp_scan_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, self.opp_odom_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, self.ego_opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, self.opp_ego_odom_topic, 10)

        if self.has_opp2:
            self.opp2_scan_pub = self.create_publisher(LaserScan, self.opp2_scan_topic, 10)
            self.opp2_odom_pub = self.create_publisher(Odometry, self.opp2_odom_topic, 10)
            self.opp2_ego_odom_pub = self.create_publisher(Odometry, self.opp2_ego_odom_topic, 10)

        # Subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            self.ego_drive_topic,
            self.drive_callback,
            10
        )

        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10
        )

        if self.has_opp:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                self.opp_drive_topic,
                self.opp_drive_callback,
                10
            )

            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10
            )

        if self.has_opp2:
            self.opp2_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                self.opp2_drive_topic,
                self.opp2_drive_callback,
                10
            )

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10
            )

        self.get_logger().info(f'gym_bridge iniciado con {self.num_agents} agente(s).')

    def drive_callback(self, drive_msg):
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.ego_drive_published = True

    def opp_drive_callback(self, drive_msg):
        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = drive_msg.drive.steering_angle
        self.opp_drive_published = True

    def opp2_drive_callback(self, drive_msg):
        self.opp2_requested_speed = drive_msg.drive.speed
        self.opp2_steer = drive_msg.drive.steering_angle
        self.opp2_drive_published = True

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')

        poses = [[rx, ry, rtheta]]

        if self.has_opp:
            poses.append([self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]])

        if self.has_opp2:
            poses.append([self.obs['poses_x'][2], self.obs['poses_y'][2], self.obs['poses_theta'][2]])

        self.obs, _, self.done, _ = self.env.reset(np.array(poses))
        self._update_sim_state()

    def opp_reset_callback(self, pose_msg):
        if not self.has_opp:
            return

        rx = pose_msg.pose.position.x
        ry = pose_msg.pose.position.y
        rqx = pose_msg.pose.orientation.x
        rqy = pose_msg.pose.orientation.y
        rqz = pose_msg.pose.orientation.z
        rqw = pose_msg.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')

        poses = [list(self.ego_pose), [rx, ry, rtheta]]

        if self.has_opp2:
            poses.append(list(self.opp2_pose))

        self.obs, _, self.done, _ = self.env.reset(np.array(poses))
        self._update_sim_state()

    def teleop_callback(self, twist_msg):
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self):
        ego_control = [self.ego_steer, self.ego_requested_speed]
        controls = [ego_control]

        if self.has_opp:
            opp_control = [self.opp_steer, self.opp_requested_speed]
            controls.append(opp_control)

        if self.has_opp2:
            opp2_control = [self.opp2_steer, self.opp2_requested_speed]
            controls.append(opp2_control)

        any_drive_published = self.ego_drive_published

        if self.has_opp:
            any_drive_published = any_drive_published or self.opp_drive_published

        if self.has_opp2:
            any_drive_published = any_drive_published or self.opp2_drive_published

        if any_drive_published:
            self.obs, _, self.done, _ = self.env.step(np.array(controls))

        self._update_sim_state()

    def timer_callback(self):
        ts = self.get_clock().now().to_msg()

        # Publish scans
        self._publish_scan(
            self.ego_scan_pub,
            self.ego_namespace + '/laser',
            self.ego_scan,
            ts
        )

        if self.has_opp:
            self._publish_scan(
                self.opp_scan_pub,
                self.opp_namespace + '/laser',
                self.opp_scan,
                ts
            )

        if self.has_opp2:
            self._publish_scan(
                self.opp2_scan_pub,
                self.opp2_namespace + '/laser',
                self.opp2_scan,
                ts
            )

        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _publish_scan(self, pub, frame_id, ranges, ts):
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.0
        scan.range_max = 30.0
        scan.ranges = ranges
        pub.publish(scan)

    def _update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])
            self.opp_pose[0] = self.obs['poses_x'][1]
            self.opp_pose[1] = self.obs['poses_y'][1]
            self.opp_pose[2] = self.obs['poses_theta'][1]
            self.opp_speed[0] = self.obs['linear_vels_x'][1]
            self.opp_speed[1] = self.obs['linear_vels_y'][1]
            self.opp_speed[2] = self.obs['ang_vels_z'][1]

        if self.has_opp2:
            self.opp2_scan = list(self.obs['scans'][2])
            self.opp2_pose[0] = self.obs['poses_x'][2]
            self.opp2_pose[1] = self.obs['poses_y'][2]
            self.opp2_pose[2] = self.obs['poses_theta'][2]
            self.opp2_speed[0] = self.obs['linear_vels_x'][2]
            self.opp2_speed[1] = self.obs['linear_vels_y'][2]
            self.opp2_speed[2] = self.obs['ang_vels_z'][2]

    def _create_odom_msg(self, namespace, pose, speed, ts):
        odom = Odometry()
        odom.header.stamp = ts
        odom.header.frame_id = 'map'
        odom.child_frame_id = namespace + '/base_link'
        odom.pose.pose.position.x = pose[0]
        odom.pose.pose.position.y = pose[1]

        quat = euler.euler2quat(0.0, 0.0, pose[2], axes='sxyz')
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.pose.orientation.w = quat[0]

        odom.twist.twist.linear.x = speed[0]
        odom.twist.twist.linear.y = speed[1]
        odom.twist.twist.angular.z = speed[2]

        return odom

    def _publish_odom(self, ts):
        ego_odom = self._create_odom_msg(self.ego_namespace, self.ego_pose, self.ego_speed, ts)
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = self._create_odom_msg(self.opp_namespace, self.opp_pose, self.opp_speed, ts)
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

        if self.has_opp2:
            opp2_odom = self._create_odom_msg(self.opp2_namespace, self.opp2_pose, self.opp2_speed, ts)
            self.opp2_odom_pub.publish(opp2_odom)
            self.opp2_ego_odom_pub.publish(ego_odom)

    def _publish_body_transform(self, namespace, pose, ts):
        t = Transform()
        t.translation.x = pose[0]
        t.translation.y = pose[1]
        t.translation.z = 0.0

        quat = euler.euler2quat(0.0, 0.0, pose[2], axes='sxyz')
        t.rotation.x = quat[1]
        t.rotation.y = quat[2]
        t.rotation.z = quat[3]
        t.rotation.w = quat[0]

        ts_msg = TransformStamped()
        ts_msg.transform = t
        ts_msg.header.stamp = ts
        ts_msg.header.frame_id = 'map'
        ts_msg.child_frame_id = namespace + '/base_link'
        self.br.sendTransform(ts_msg)

    def _publish_transforms(self, ts):
        self._publish_body_transform(self.ego_namespace, self.ego_pose, ts)

        if self.has_opp:
            self._publish_body_transform(self.opp_namespace, self.opp_pose, ts)

        if self.has_opp2:
            self._publish_body_transform(self.opp2_namespace, self.opp2_pose, ts)

    def _publish_wheel_pair(self, namespace, steer, ts):
        wheel_ts = TransformStamped()
        wheel_quat = euler.euler2quat(0.0, 0.0, steer, axes='sxyz')
        wheel_ts.transform.rotation.x = wheel_quat[1]
        wheel_ts.transform.rotation.y = wheel_quat[2]
        wheel_ts.transform.rotation.z = wheel_quat[3]
        wheel_ts.transform.rotation.w = wheel_quat[0]
        wheel_ts.header.stamp = ts

        wheel_ts.header.frame_id = namespace + '/front_left_hinge'
        wheel_ts.child_frame_id = namespace + '/front_left_wheel'
        self.br.sendTransform(wheel_ts)

        wheel_ts.header.frame_id = namespace + '/front_right_hinge'
        wheel_ts.child_frame_id = namespace + '/front_right_wheel'
        self.br.sendTransform(wheel_ts)

    def _publish_wheel_transforms(self, ts):
        self._publish_wheel_pair(self.ego_namespace, self.ego_steer, ts)

        if self.has_opp:
            self._publish_wheel_pair(self.opp_namespace, self.opp_steer, ts)

        if self.has_opp2:
            self._publish_wheel_pair(self.opp2_namespace, self.opp2_steer, ts)

    def _publish_laser_transform(self, namespace, ts):
        scan_ts = TransformStamped()
        scan_ts.transform.translation.x = self.scan_distance_to_base_link
        scan_ts.transform.rotation.w = 1.0
        scan_ts.header.stamp = ts
        scan_ts.header.frame_id = namespace + '/base_link'
        scan_ts.child_frame_id = namespace + '/laser'
        self.br.sendTransform(scan_ts)

    def _publish_laser_transforms(self, ts):
        self._publish_laser_transform(self.ego_namespace, ts)

        if self.has_opp:
            self._publish_laser_transform(self.opp_namespace, ts)

        if self.has_opp2:
            self._publish_laser_transform(self.opp2_namespace, ts)


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    rclpy.spin(gym_bridge)


if __name__ == '__main__':
    main()
