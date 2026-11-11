#!/usr/bin/env python3
"""
rssi_sim.py - gia lap RSSI cua mot Access Point cho robot trong Gazebo/Nav2.

  map_server (/map) ─┐
  TF map->base_footprint ─┼─► [rssi_sim] ─► /wifi/rssi_dbm        (Float32, 5 Hz)
  /clicked_point (RViz) ─┘                 ─► /wifi/ap_marker      (vi tri AP)
                                           ─► /wifi/truth_map      (heatmap "dap an dung")
                                           ─► CSV (tuy chon)

RSSI = P0 - 10 n log10(d) - wall_db * (so tuong chan) + shadowing(co dinh theo vi tri)
       + fading(nhieu moi mau)

Chay:
    python3 rssi_sim.py --ros-args -p use_sim_time:=true -p ap_x:=1.0 -p ap_y:=2.0 \
        -p csv:=/tmp/rssi_log.csv
Doi vi tri AP luc dang chay: RViz -> "Publish Point" -> bam len map.
"""
import csv
import math
import sys

import numpy as np
from scipy.ndimage import gaussian_filter


class RssiModel:
    def __init__(self, grid, res, origin, ap_xy, p0=-40.0, n=2.0, wall_db=5.0,
                 sigma_shadow=4.0, corr_len=1.5, sigma_fast=2.0, seed=0):
        self.g = grid                      # True = o dac (tuong)
        self.H, self.W = grid.shape
        self.res, self.ox, self.oy = res, origin[0], origin[1]
        self.ap = np.array(ap_xy, float)
        self.p0, self.n, self.wall_db, self.sf = p0, n, wall_db, sigma_fast
        self.rng = np.random.default_rng(seed)
        w = self.rng.standard_normal(grid.shape)
        s = gaussian_filter(w, max(corr_len / res, 1e-3))
        self.shadow = s / s.std() * sigma_shadow    # co dinh theo vi tri

    def cell(self, x, y):
        r = int(math.floor((y - self.oy) / self.res))
        c = int(math.floor((x - self.ox) / self.res))
        return r, c

    def inside(self, r, c):
        return 0 <= r < self.H and 0 <= c < self.W

    def walls_between(self, x, y):
        d = math.hypot(x - self.ap[0], y - self.ap[1])
        k = max(int(d / (self.res * 0.5)), 2)
        xs = np.linspace(self.ap[0], x, k)
        ys = np.linspace(self.ap[1], y, k)
        r = np.floor((ys - self.oy) / self.res).astype(int)
        c = np.floor((xs - self.ox) / self.res).astype(int)
        ok = (r >= 0) & (r < self.H) & (c >= 0) & (c < self.W)
        occ = np.zeros(k, bool)
        occ[ok] = self.g[r[ok], c[ok]]
        idx = np.nonzero(occ[1:] & ~occ[:-1])[0]     # cac cho tia buoc vao o dac
        if len(idx) == 0:
            return 0
        # map thuc te co mep tuong nham/cham nhieu: cac lan buoc vao cach nhau < 0.4 m
        # doc theo tia chi tinh la MOT tuong
        gap = max(int(0.4 / (self.res * 0.5)), 1)
        n, last = 1, idx[0]
        for i in idx[1:]:
            if i - last >= gap:
                n += 1
            last = i
        return min(n, 4)     # >4 tuong: tin hieu chu yeu di vong/nhieu duong, khong cong don mai

    def mean_rssi(self, x, y):
        d = max(math.hypot(x - self.ap[0], y - self.ap[1]), 1.0)
        r, c = self.cell(x, y)
        sh = self.shadow[r, c] if self.inside(r, c) else 0.0
        return (self.p0 - 10 * self.n * math.log10(d)
                - self.wall_db * self.walls_between(x, y) + sh)

    def sample(self, x, y):
        return self.mean_rssi(x, y) + self.rng.normal(0, self.sf)

    def truth_grid(self, free, step_m=0.25):
        """Luoi RSSI trung binh (khong nhieu nhanh) tren cac o trong, do phan giai step_m."""
        st = max(int(round(step_m / self.res)), 1)
        rows = range(0, self.H, st)
        cols = range(0, self.W, st)
        out = np.full((len(rows), len(cols)), np.nan)
        for i, r in enumerate(rows):
            for j, c in enumerate(cols):
                if free[r, c]:
                    x = self.ox + (c + 0.5) * self.res
                    y = self.oy + (r + 0.5) * self.res
                    out[i, j] = self.mean_rssi(x, y)
        return out, st * self.res

    def truth_points(self, free, step_m=0.25):
        """Giong truth_grid nhung tra ve (x, y, rssi) cua tung o trong."""
        st = max(int(round(step_m / self.res)), 1)
        xs, ys, vs = [], [], []
        for r in range(0, self.H, st):
            for c in range(0, self.W, st):
                if free[r, c]:
                    x = self.ox + (c + 0.5) * self.res
                    y = self.oy + (r + 0.5) * self.res
                    xs.append(x); ys.append(y); vs.append(self.mean_rssi(x, y))
        return np.array(xs), np.array(ys), np.array(vs)


# ---------------------------------------------------------------- ROS 2
def main(args=None):
    import rclpy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float32
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker

    class RssiSim(Node):
        def __init__(self):
            super().__init__('rssi_sim')
            P = self.declare_parameter
            P('ap_x', 0.0); P('ap_y', 0.0); P('ap_z', 2.5)
            P('p0', -40.0); P('n', 2.0); P('wall_db', 5.0)
            P('sigma_shadow', 4.0); P('corr_len', 1.5); P('sigma_fast', 2.0)
            P('seed', 0); P('rate_hz', 5.0); P('csv', '')
            P('map_frame', 'map'); P('base_frame', 'base_footprint')
            P('truth_step', 0.25); P('thr_dbm', -67.0); P('truth_csv', '')
            P('hide_truth', False)   # true: giau dap an, chi hien/ghi khi bam Ctrl+C

            self.model = None
            self.free = None
            self.map_msg = None
            self.csv_f = None
            self.csv_w = None
            path = self.get_parameter('csv').value
            if path:
                self.csv_f = open(path, 'w', newline='')
                self.csv_w = csv.writer(self.csv_f)
                self.csv_w.writerow(['t_s', 'x', 'y', 'rssi_dbm', 'rssi_mean_dbm'])

            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
            self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
            self.pub = self.create_publisher(Float32, '/wifi/rssi_dbm', 10)
            self.pub_marker = self.create_publisher(Marker, '/wifi/ap_marker', 10)
            self.pub_truth = self.create_publisher(OccupancyGrid, '/wifi/truth_map', latched)
            self.tf = Buffer()
            TransformListener(self.tf, self)
            self.create_timer(1.0 / float(self.get_parameter('rate_hz').value), self.tick)
            self.create_timer(1.0, self.pub_ap_marker)
            self.get_logger().info('rssi_sim doi /map tu map_server ...')

        # -- map -> model
        def on_map(self, m):
            g = np.array(m.data, np.int8).reshape(m.info.height, m.info.width)
            self.map_msg = m
            self.free = (g == 0)
            self.walls = g > 50           # o chua biet (-1) khong tinh la tuong
            o = m.info.origin.position
            gp = self.get_parameter
            self.model = RssiModel(
                self.walls, m.info.resolution, (o.x, o.y),
                (gp('ap_x').value, gp('ap_y').value), p0=gp('p0').value, n=gp('n').value,
                wall_db=gp('wall_db').value, sigma_shadow=gp('sigma_shadow').value,
                corr_len=gp('corr_len').value, sigma_fast=gp('sigma_fast').value,
                seed=int(gp('seed').value))
            self.get_logger().info(
                f'Nhan map {m.info.width}x{m.info.height}, {m.info.resolution} m/o. '
                f'AP o ({self.model.ap[0]:.2f}, {self.model.ap[1]:.2f})')
            self.publish_truth()

        # -- doi vi tri AP bang RViz "Publish Point"
        def on_click(self, msg):
            if self.model is None:
                return
            self.model.ap = np.array([msg.point.x, msg.point.y], float)
            self.get_logger().info(f'AP chuyen toi ({msg.point.x:.2f}, {msg.point.y:.2f})')
            self.publish_truth()

        def publish_truth(self, force=False):
            if self.model is None:
                return
            if self.get_parameter('hide_truth').value and not force:
                return                      # dang giau dap an: khong in log, khong ghi file, khong publish topic
            step = float(self.get_parameter('truth_step').value)
            grid, res = self.model.truth_grid(self.free, step)
            thr = float(self.get_parameter('thr_dbm').value)
            ok = np.isfinite(grid)
            self.get_logger().info(
                f'DAP AN DUNG (toan bo khong gian trong, {ok.sum()*res*res:.1f} m2): '
                f'{np.mean(grid[ok] >= thr)*100:.1f}% dien tich dat >= {thr:.0f} dBm, '
                f'RSSI thap nhat {np.nanmin(grid):.1f} dBm')
            tpath = self.get_parameter('truth_csv').value
            if tpath:
                tx, ty, tv = self.model.truth_points(self.free, step)
                np.savetxt(tpath, np.c_[tx, ty, tv], delimiter=',',
                           header='x,y,rssi_true_dbm', comments='', fmt='%.3f')
                self.get_logger().info(f'Da ghi dap an dung ({len(tx)} o) vao {tpath}')
            # RSSI -90..-30 dBm -> 0..100 (RViz Map, scheme costmap)
            v = np.clip((grid + 90.0) / 60.0 * 100.0, 0, 100)
            v = np.where(np.isnan(grid), -1, v).astype(np.int8)
            m = OccupancyGrid()
            m.header.frame_id = self.get_parameter('map_frame').value
            m.header.stamp = self.get_clock().now().to_msg()
            m.info.resolution = float(res)
            m.info.height, m.info.width = v.shape
            m.info.origin = self.map_msg.info.origin
            m.data = v.flatten().tolist()
            self.pub_truth.publish(m)

        def pub_ap_marker(self):
            if self.model is None:
                return
            mk = Marker()
            mk.header.frame_id = self.get_parameter('map_frame').value
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns, mk.id, mk.type, mk.action = 'ap', 0, Marker.SPHERE, Marker.ADD
            mk.pose.position.x = float(self.model.ap[0])
            mk.pose.position.y = float(self.model.ap[1])
            mk.pose.position.z = float(self.get_parameter('ap_z').value)
            mk.pose.orientation.w = 1.0
            mk.scale.x = mk.scale.y = mk.scale.z = 0.3
            mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.4, 0.0, 1.0
            self.pub_marker.publish(mk)

        def tick(self):
            if self.model is None:
                return
            try:
                t = self.tf.lookup_transform(self.get_parameter('map_frame').value,
                                             self.get_parameter('base_frame').value,
                                             rclpy.time.Time())
            except Exception:
                return
            x, y = t.transform.translation.x, t.transform.translation.y
            mean = self.model.mean_rssi(x, y)
            val = mean + float(self.model.rng.normal(0, self.model.sf))
            self.pub.publish(Float32(data=float(val)))
            if self.csv_w:
                ts = self.get_clock().now().nanoseconds * 1e-9
                self.csv_w.writerow([f'{ts:.3f}', f'{x:.3f}', f'{y:.3f}',
                                     f'{val:.2f}', f'{mean:.2f}'])
                self.csv_f.flush()

    rclpy.init(args=args)
    node = RssiSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.csv_f:
            node.csv_f.close()
        if node.get_parameter('hide_truth').value:       # khao sat xong -> moi hien dap an
            try:
                node.publish_truth(force=True)
            except Exception as e:                       # khong de loi lam mat file csv da dong
                node.get_logger().error(f'Khong ghi duoc dap an: {e}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv)
