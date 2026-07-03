import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

import sys
import select
import termios
import tty


class TecladoAckermann(Node):
    def __init__(self):
        super().__init__('teclado_ackermann')

        self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.speed = 0.0
        self.steering = 0.0

        self.max_speed = 3.0
        self.speed_step = 0.3

        self.max_steering = 0.42
        self.steering_step = 0.06

        self.timer = self.create_timer(0.05, self.loop)

        self.get_logger().info('Control listo:')
        self.get_logger().info('W/S: acelerar/frenar | A/D: girar | ESPACIO: parar | Q: salir')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.01)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def loop(self):
        key = self.get_key()

        if key == 'w':
            self.speed += self.speed_step
        elif key == 's':
            self.speed -= self.speed_step
        elif key == 'a':
            self.steering += self.steering_step
        elif key == 'd':
            self.steering -= self.steering_step
        elif key == ' ':
            self.speed = 0.0
            self.steering = 0.0
        elif key == 'q':
            self.get_logger().info('Saliendo...')
            rclpy.shutdown()
            return
        elif key == '':
            self.steering *= 0.85

        self.speed = max(-self.max_speed, min(self.max_speed, self.speed))
        self.steering = max(-self.max_steering, min(self.max_steering, self.steering))

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TecladoAckermann()
    node.settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        node.pub.publish(stop_msg)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()