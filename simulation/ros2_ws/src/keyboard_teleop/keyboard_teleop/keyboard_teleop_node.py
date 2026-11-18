#!/usr/bin/env python3
"""
keyboard_teleop_node.py
========================
Dieu khien robot bang ban phim, publish geometry_msgs/Twist len /cmd_vel.

Thiet ke:
  - Xoay (a/d): TOC DO GOC CO DINH, khong tang/giam dan -- nhan la xoay
    ngay o dung 1 muc angular_speed da cau hinh
  - Tien/lui (w/s): dung DUNG MUC TOC DO dang duoc CHON qua phim 1/2/3
    (cham / binh thuong / nhanh)
  - Ngang trai/phai (q/e): TOC DO NGANG (vy) CO DINH, khong doi theo
    muc 1/2/3 -- nhan la strafe ngay o dung strafe_speed
  - x hoac space: DUNG TAT CA, ve 0 (ca linear va angular)
  - z: thoat, tu dong gui lenh dung truoc khi ra

Doc TUNG PHIM ngay khi nhan (raw mode qua termios/tty), KHONG can Enter.
Raw mode duoc bat 1 LAN DUY NHAT truoc vong lap (khong bat/tat lien tuc).

QUAN TRONG -- FIX BUG "phai giu phim moi chay lien tuc":
  Ban dau os.read(fd, 1) la lenh BLOCKING -- no treo vo thoi han cho
  toi khi co phim moi. Trong luc treo do, rclpy.spin_once() KHONG duoc
  goi lai, nen timer publish lien tuc (create_timer) KHONG chay duoc,
  dan toi /cmd_vel chi duoc gui DUNG 1 LAN tai thoi diem nhan phim, roi
  ngung hoan toan cho toi khi co phim tiep theo. Vi ben serial_bridge
  co co che an toan (neu qua cmd_timeout_sec khong co /cmd_vel moi se
  tu dong gui lenh dung), nen trieu chung la "an phim chay 1 chut roi
  dung ngay, phai giu phim moi chay lien tuc" (giu phim = OS tu dong
  lap lai ky tu, vo tinh "danh thuc" os.read() lien tuc).

  FIX: dung select.select([fd], [], [], TIMEOUT_NGAN) de KIEM TRA co
  phim dang cho khong, thay vi doc thang bang os.read() (luon chan).
  Neu khong co phim trong khoang timeout ngan, vong lap van tiep tuc
  ngay, cho phep spin_once() chay timer publish deu dan -- /cmd_vel
  se duoc gui LIEN TUC theo dung publish_rate_hz du CO nhan phim hay
  KHONG, dung nhu thiet ke ban dau mong muon.

QUAN TRONG: chay TRUC TIEP qua "ros2 run" trong 1 terminal that,
KHONG chay qua ros2 launch (can TTY that de doc phim).
"""

import os
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


SPEED_LABELS = {'slow': 'CHAM', 'normal': 'BINH THUONG', 'fast': 'NHANH'}

BANNER = """
─────────────────────────────
Dieu khien robot bang ban phim
  w : tien (o muc toc do dang chon)
  x : lui  (o muc toc do dang chon)
  a : xoay TRAI  (toc do goc CO DINH)
  d : xoay PHAI  (toc do goc CO DINH)
  q : ngang TRAI (vy CO DINH)
  e : ngang PHAI (vy CO DINH)
  1 : chon muc toc do CHAM
  2 : chon muc toc do BINH THUONG
  3 : chon muc toc do NHANH
  s / space : DUNG TAT CA (ve 0)
  z : thoat
─────────────────────────────
"""


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('strafe_speed', 0.4)
        self.declare_parameter('slow_speed', 0.08)
        self.declare_parameter('normal_speed', 0.3)
        self.declare_parameter('fast_speed', 0.5)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.angular_speed = self.get_parameter('angular_speed').value
        self.strafe_speed = self.get_parameter('strafe_speed').value
        self.speed_levels = {
            'slow': self.get_parameter('slow_speed').value,
            'normal': self.get_parameter('normal_speed').value,
            'fast': self.get_parameter('fast_speed').value,
        }
        rate = self.get_parameter('publish_rate_hz').value

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_level = 'normal'
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        # Publish LIEN TUC theo timer -- day la ly do bug xay ra truoc
        # day: neu vong lap doc phim bi CHAN (blocking), timer nay
        # khong duoc rclpy xu ly du no ton tai.
        self.create_timer(1.0 / rate, self._publish_cmd_vel)

        self.get_logger().info(BANNER)
        self._print_status()

    def _publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = self.linear_y
        msg.angular.z = self.angular_z
        self.pub_cmd_vel.publish(msg)

    def _print_status(self):
        label = SPEED_LABELS[self.current_level]
        speed = self.speed_levels[self.current_level]
        print(
            f'\r[Muc: {label} ({speed:.2f} m/s)]  '
            f'v={self.linear_x:+.3f} m/s   vy={self.linear_y:+.3f} m/s   '
            f'w={self.angular_z:+.3f} rad/s     ',
            end='', flush=True,
        )

    def handle_key(self, key: str) -> bool:
        """Xu ly 1 phim vua nhan. Tra ve False neu can thoat vong lap."""

        if key == '1':
            self.current_level = 'slow'
        elif key == '2':
            self.current_level = 'normal'
        elif key == '3':
            self.current_level = 'fast'

        elif key == 'w':
            self.linear_x = self.speed_levels[self.current_level]
        elif key == 'x':
            self.linear_x = -self.speed_levels[self.current_level]

        elif key == 'a':
            self.angular_z = self.angular_speed
        elif key == 'd':
            self.angular_z = -self.angular_speed

        elif key == 'q':
            self.linear_y = self.strafe_speed
        elif key == 'e':
            self.linear_y = -self.strafe_speed

        elif key in ('s', ' '):
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0

        elif key == 'z':
            return False

        self._publish_cmd_vel()
        self._print_status()
        return True


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    if not sys.stdin.isatty():
        node.get_logger().error(
            'stdin khong phai TTY that -- node nay PHAI chay truc tiep '
            'qua "ros2 run" trong 1 terminal, KHONG chay qua ros2 launch.'
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    # Timeout ngan cho select() -- vua du de vong lap quay lai thuong
    # xuyen, cho spin_once() co co hoi chay timer publish DU KHONG CO
    # phim nao duoc nhan trong khoang thoi gian do.
    SELECT_TIMEOUT_SEC = 0.02  # 20ms

    tty.setraw(fd)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            # KIEM TRA co phim dang cho khong, KHONG chan neu chua co --
            # day la fix chinh cho bug "phai giu phim moi chay lien tuc".
            ready, _, _ = select.select([fd], [], [], SELECT_TIMEOUT_SEC)
            if not ready:
                continue  # khong co phim -> quay lai dau vong lap, de
                          # spin_once() tiep tuc xu ly timer publish

            raw_byte = os.read(fd, 1)
            key = raw_byte.decode('utf-8', errors='ignore')
            if not node.handle_key(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        stop_msg = Twist()
        node.pub_cmd_vel.publish(stop_msg)
        print('\nDa gui lenh dung. Thoat.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
