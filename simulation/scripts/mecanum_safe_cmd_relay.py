#!/usr/bin/env python3
"""Deadman and slow acceleration limit: ROS /cmd_vel -> /cmd_vel_gazebo."""
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SafeRelay(Node):
    def __init__(self):
        super().__init__('mecanum_safe_cmd_relay')
        self.last = -1000.0
        self.target = [0.,0.,0.]
        self.value = [0.,0.,0.]
        self.sub = self.create_subscription(Twist,'/cmd_vel',self.cb,10)
        self.pub = self.create_publisher(Twist,'/cmd_vel_gazebo',10)
        self.timer = self.create_timer(0.05,self.tick)
        self.get_logger().info('Safe relay active, max vx=0.08m/s, vy=0.06m/s, wz=0.2rad/s')
    def cb(self, msg):
        v = [msg.linear.x,msg.linear.y,msg.angular.z]
        if not all(math.isfinite(i) for i in v):
            return
        limits = [0.48,0.6,1.0]
        self.target = [max(-b,min(b,a)) for a,b in zip(v,limits)]
        self.last = time.monotonic()
    def tick(self):
        goal = self.target if time.monotonic()-self.last < 0.3 else [0.,0.,0.]
        # Linear acceleration limit = 0.1 m/s², yaw = 0.3 rad/s².
        for i in range(3):
            step = 0.05 * (0.3 if i == 2 else 0.1)
            diff = max(-step,min(step,goal[i]-self.value[i]))
            self.value[i] += diff
        m=Twist()
        m.linear.x,m.linear.y,m.angular.z=self.value
        self.pub.publish(m)

def main():
    rclpy.init()
    n=SafeRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
