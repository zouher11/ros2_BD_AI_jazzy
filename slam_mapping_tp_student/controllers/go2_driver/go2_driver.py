#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from controller import Supervisor
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster


class Go2WebotsDriver(Node):
    def __init__(self, robot):
        super().__init__('go2_driver')
        self.robot = robot
        self.timestep = int(self.robot.getBasicTimeStep())

        self.wheel_radius = 0.05
        self.wheel_base = 0.36
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.left_motor = self.robot.getDevice('left_wheel_motor')
        self.right_motor = self.robot.getDevice('right_wheel_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.left_sensor = self.robot.getDevice('left_wheel_sensor')
        self.right_sensor = self.robot.getDevice('right_wheel_sensor')
        if self.left_sensor and self.right_sensor:
            self.left_sensor.enable(self.timestep)
            self.right_sensor.enable(self.timestep)
        self.prev_left = 0.0
        self.prev_right = 0.0
        self.first_odom = True

        self.lidar = self.robot.getDevice('lidar')
        if self.lidar:
            self.lidar.enable(self.timestep)

        self.camera = self.robot.getDevice('camera')
        if self.camera:
            self.camera.enable(self.timestep * 2)

        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cam_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.target_v = 0.0
        self.target_w = 0.0

        pedestrian = self.robot.getFromDef('DYNAMIC_PEDESTRIAN')
        cart = self.robot.getFromDef('DYNAMIC_CART')

        if pedestrian is None or cart is None:
            raise RuntimeError("Dynamic obstacles not found")

        self.pedestrian_position = pedestrian.getField('translation')
        self.cart_position = cart.getField('translation')

    def cmd_cb(self, msg):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        v_l = (self.target_v - self.target_w * self.wheel_base / 2.0) / self.wheel_radius
        v_r = (self.target_v + self.target_w * self.wheel_base / 2.0) / self.wheel_radius
        max_v = self.left_motor.getMaxVelocity()
        self.left_motor.setVelocity(max(min(v_l, max_v), -max_v))
        self.right_motor.setVelocity(max(min(v_r, max_v), -max_v))

    def get_ros_time(self, sim_time):
        t = Time()
        t.sec = int(sim_time)
        t.nanosec = int((sim_time - t.sec) * 1e9)
        return t

    def step(self):
        sim_time = self.robot.getTime()

        self.pedestrian_position.setSFVec3f([
            -1.8 + 1.15 * math.sin(0.45 * sim_time),
            0.85,
            -1.5
        ])

        self.cart_position.setSFVec3f([
            1.8 + 1.05 * math.sin(0.35 * sim_time + 1.0),
            0.30,
            1.5
        ])
        now = self.get_ros_time(sim_time)

        clock_msg = Clock()
        clock_msg.clock = now
        self.clock_pub.publish(clock_msg)

        if self.left_sensor and self.right_sensor:
            cur_l = self.left_sensor.getValue()
            cur_r = self.right_sensor.getValue()
            if self.first_odom:
                self.prev_left = cur_l
                self.prev_right = cur_r
                self.first_odom = False
            dl = (cur_l - self.prev_left) * self.wheel_radius
            dr = (cur_r - self.prev_right) * self.wheel_radius
            self.prev_left = cur_l
            self.prev_right = cur_r
            dc = (dl + dr) / 2.0
            dth = (dr - dl) / self.wheel_base
        else:
            dt = self.timestep / 1000.0
            dc = self.target_v * dt
            dth = self.target_w * dt

        self.x += dc * math.cos(self.th + dth / 2.0)
        self.y += dc * math.sin(self.th + dth / 2.0)
        self.th += dth

        qz = math.sin(self.th / 2.0)
        qw = math.cos(self.th / 2.0)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.target_v
        odom.twist.twist.angular.z = self.target_w
        self.odom_pub.publish(odom)

        if self.lidar:
            ranges = list(self.lidar.getRangeImage())
            if ranges:
                scan = LaserScan()
                scan.header.stamp = now
                scan.header.frame_id = 'base_link'
                fov = self.lidar.getFov()
                scan.angle_min = -fov / 2.0
                scan.angle_max = fov / 2.0
                scan.angle_increment = fov / max(len(ranges) - 1, 1)
                scan.range_min = self.lidar.getMinRange()
                scan.range_max = self.lidar.getMaxRange()
                scan.ranges = [float(r) if not math.isinf(r) else float('inf') for r in reversed(ranges)]
                self.scan_pub.publish(scan)


def main():
    rclpy.init()
    robot = Supervisor()
    driver = Go2WebotsDriver(robot)
    while robot.step(driver.timestep) != -1:
        rclpy.spin_once(driver, timeout_sec=0.0)
        driver.step()
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
