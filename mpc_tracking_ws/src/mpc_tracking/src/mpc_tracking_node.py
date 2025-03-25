#!/usr/bin/env python3
import rospy
import math
import numpy as np
import json
import casadi as ca
from sensor_msgs.msg import LaserScan
from scipy.interpolate import CubicSpline
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from utils.msg import localisation

# 쿼터니언에서 yaw 각도를 계산하는 함수
def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

# 비선형 MPC 컨트롤러 클래스
class NonlinearMPCController:
    def __init__(self, dt=0.25, horizon=8, wheelbase=0.26):
        self.dt = dt
        self.T = horizon
        self.wb = wheelbase
        self.scenario = 'driving'
        self.set_scenario(self.scenario)
        self.nx = 3  # 상태 변수: (x, y, yaw)
        self.nu = 2  # 제어 입력: (v, steer)

    def set_scenario(self, scenario):
        self.scenario = scenario
        if scenario == "parking":
            rospy.loginfo("[MPC] => parking param set")
            self.T = 12
            self.Qx = 40.0
            self.Qy = 70.0
            self.Qyaw = 2
            self.Rv = 0.005
            self.Rsteer = 0.005
            self.Rdv = 0.001
            self.Rdsteer = 0.001
            self.v_min = -0.25
            self.v_max = 0.2
            self.max_steer = math.radians(25.0)
            self.min_steer = -self.max_steer
        else:
            self.T = 12
            self.Qx = 700.0
            self.Qy = 700.0
            self.Qyaw = 500.0
            self.Rv = 0.01
            self.Rsteer = 0.01
            self.Rdv = 0.005
            self.Rdsteer = 0.005
            self.v_min = -0.2
            self.v_max = 0.3
            self.max_steer = math.radians(25.0)
            self.min_steer = -self.max_steer

    def solve_mpc(self, x0, xref):
        T = self.T
        dt = self.dt
        wb = self.wb
        n_vars = (self.nx)*(T+1) + (self.nu)*T
        opt_x = ca.SX.sym('opt_x', n_vars)

        def sidx(k): return self.nx*k
        def cidx(k): return self.nx*(T+1) + self.nu*k

        obj = 0.0
        g = []
        lbg = []
        ubg = []

        g += [opt_x[sidx(0)+0] - x0[0], opt_x[sidx(0)+1] - x0[1], opt_x[sidx(0)+2] - x0[2]]
        lbg += [0.0, 0.0, 0.0]
        ubg += [0.0, 0.0, 0.0]

        def f(st, con):
            x, y, yaw = st[0], st[1], st[2]
            v, steer = con[0], con[1]
            dx = v * ca.cos(yaw)
            dy = v * ca.sin(yaw)
            dyaw = (v/wb)*steer
            return ca.vertcat(dx, dy, dyaw)

        for k in range(T):
            st_k = opt_x[sidx(k):sidx(k)+3]
            con_k = opt_x[cidx(k):cidx(k)+2]
            st_next = opt_x[sidx(k+1):sidx(k+1)+3]
            k1 = f(st_k, con_k)
            k2 = f(st_k+(dt/2)*k1, con_k)
            k3 = f(st_k+(dt/2)*k2, con_k)
            k4 = f(st_k+dt*k3, con_k)
            st_rk4 = st_k+(dt/6)*(k1+2*k2+2*k3+k4)
            g += [st_next - st_rk4]
            lbg += [0.0, 0.0, 0.0]
            ubg += [0.0, 0.0, 0.0]

        for k in range(T+1):
            xk = opt_x[sidx(k)+0]
            yk = opt_x[sidx(k)+1]
            yawk = opt_x[sidx(k)+2]
            xr = xref[0, k]
            yr = xref[1, k]
            yr_yaw = xref[2, k]
            obj += self.Qx*(xk - xr)**2 + self.Qy*(yk - yr)**2 + self.Qyaw*(yawk - yr_yaw)**2

        for k in range(T):
            vk = opt_x[cidx(k)+0]
            steer_k = opt_x[cidx(k)+1]
            obj += self.Rv*(vk**2) + self.Rsteer*(steer_k**2)
            if k < T-1:
                v_next = opt_x[cidx(k+1)+0]
                s_next = opt_x[cidx(k+1)+1]
                obj += self.Rdv*((v_next - vk)**2) + self.Rdsteer*((s_next - steer_k)**2)

        lbx = [-1e6]*((T+1)*self.nx) + [self.v_min, self.min_steer]*T
        ubx = [1e6]*((T+1)*self.nx) + [self.v_max, self.max_steer]*T

        nlp = {'f': obj, 'x': opt_x, 'g': ca.vertcat(*g)}
        solver = ca.nlpsol('solver', 'ipopt', nlp, {
            'ipopt': {'max_iter': 200, 'print_level': 0, 'acceptable_tol': 1e-6, 'acceptable_obj_change_tol': 1e-6},
            'print_time': 0
        })

        x_init = []
        for k in range(T+1):
            alpha = k/float(T+1)
            xg = x0[0]*(1-alpha) + xref[0, -1]*alpha
            yg = x0[1]*(1-alpha) + xref[1, -1]*alpha
            yawg = x0[2]*(1-alpha) + xref[2, -1]*alpha
            x_init += [xg, yg, yawg]
        x_init += [0.0, 0.0]*T

        sol = solver(x0=x_init, lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg)
        if sol['f'].full()[0] > 1e6 or 'Solve_Succeeded' not in solver.stats()['return_status']:
            rospy.logwarn(f"[MPC] solver fail: {solver.stats()['return_status']}")
            return None, None

        solx = sol['x'].full().flatten()
        all_controls = solx[self.nx*(T+1):].reshape((T, self.nu)).T
        return (all_controls[0, :], all_controls[1, :]), None

# 상태 구조체
class StateStruct:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

# ROS 노드 클래스
class MPCNode:
    def __init__(self):
        rospy.init_node("mpc_lateral_node", anonymous=True)
        self.mpc = NonlinearMPCController(dt=0.25, horizon=10, wheelbase=0.26)
        self.in_parking_mode = False
        self.in_exit_parking_mode = False
        self.parking_path = []
        self.parking_idx = 0
        self.exit_parking_path = []
        self.exit_parking_idx = 0

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.loc_ok = False
        self.yaw_ok = False

        self.global_path = []
        self.global_path_ok = False
        self.global_idx = 0

        # 주차 및 입구 노드 좌표
        self.graph_nodes = {
            "231": (10.06, 0.91),
            "233": (10.82, 0.92),
            "235": (11.58, 0.93),
            "237": (12.34, 0.93),
            "239": (13.10, 0.93),
            "900": (9.34, 0.54),
            "901": (10.04, 0.54),
            "902": (10.79, 0.54),
            "903": (11.6, 0.54),
            "904": (12.44, 0.54),
            "910": (9.34, 1.3),
            "911": (10.04, 1.3),
            "912": (10.79, 1.3),
            "913": (11.6, 1.3),
            "914": (12.44, 1.3),
        }

        self.entry_to_parking = {
            "231": ["900", "910"],
            "233": ["901", "911"],
            "235": ["902", "912"],
            "237": ["903", "913"],
            "239": ["904", "914"],
        }

        self.parking_spaces = []
        for entry, parkings in self.entry_to_parking.items():
            for parking in parkings:
                x_end, y_end = self.graph_nodes[parking]
                x_start, y_start = self.graph_nodes[entry]
                dx = x_end - x_start
                dy = y_end - y_start
                theta = math.atan2(dy, dx)
                phi = theta + math.pi / 2
                self.parking_spaces.append({
                    "id": parking,
                    "center": (x_end, y_end),
                    "phi": phi,
                    "l": 0.765,
                    "w": 0.39,
                    "occupied": False
                })

        # 고속도로 차선 변경 관련 변수
        self.highway_mode = False
        self.obstacle_detected = False
        self.in_lane_change_mode = False
        self.lane_change_path = []
        self.lane_change_idx = 0

        self.changeable_nodes = {
            483: (13.56, 10.75), 484: (13.18, 10.75), 485: (12.8, 10.74),
            486: (12.42, 10.77), 487: (12.04, 10.81), 488: (11.66, 10.87),
            489: (11.3, 10.99), 490: (10.95, 11.14), 491: (10.61, 11.32),
            492: (10.28, 11.51), 493: (9.94, 11.68), 494: (9.6, 11.86),
            495: (9.25, 12.01), 496: (8.88, 12.11), 497: (8.5, 12.15),
            498: (8.13, 12.20), 499: (7.74, 12.20), 500: (7.36, 12.20),
            501: (6.98, 12.20),
        }

        # ROS 구독자 및 발행자
        self.path_sub = rospy.Subscriber("/global_path", Path, self.path_callback, queue_size=1)
        self.loc_sub = rospy.Subscriber("/automobile/localisation", localisation, self.loc_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber("/automobile/IMU", Imu, self.imu_callback, queue_size=1)
        self.park_sub = rospy.Subscriber("/parking_signal", Bool, self.park_signal_cb, queue_size=1)
        self.highway_sub = rospy.Subscriber("/highway_signal", Bool, self.highway_signal_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher("/automobile/command", String, queue_size=10)
        self.parking_path_pub = rospy.Publisher("/parking_path", Path, queue_size=1)
        self.xref_pub = rospy.Publisher("/mpc_xref", Path, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan_depth", LaserScan, self.laserscan_callback, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def park_signal_cb(self, msg: Bool):
        if msg.data:
            self.parking_signal = True
            rospy.loginfo("[Parking] => parking signal ON")
        else:
            self.parking_signal = False
            rospy.loginfo("[Parking] => parking signal OFF")

    def highway_signal_cb(self, msg: Bool):
        self.highway_mode = msg.data
        rospy.loginfo(f"[Highway] => Highway mode {'ON' if self.highway_mode else 'OFF'}")

    def path_callback(self, msg: Path):
        self.global_path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.global_path_ok = len(self.global_path) > 2

    def loc_callback(self, msg: localisation):
        x_c, y_c = msg.posA, msg.posB
        theta = self.yaw
        wb = 0.26
        self.x = x_c - 0.5 * wb * math.cos(theta)
        self.y = y_c - 0.5 * wb * math.sin(theta)
        self.loc_ok = True

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.yaw_ok = True

    def laserscan_callback(self, msg: LaserScan):
        if not self.loc_ok or not self.yaw_ok:
            return

        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        global_points = []
        for i, r in enumerate(ranges):
            if msg.range_min < r < msg.range_max:
                alpha = angle_min + i * angle_increment
                x_local = r * math.cos(alpha)
                y_local = r * math.sin(alpha)
                x_global = self.x + x_local * math.cos(self.yaw) - y_local * math.sin(self.yaw)
                y_global = self.y + x_local * math.sin(self.yaw) + y_local * math.cos(self.yaw)
                global_points.append((x_global, y_global))

        for space in self.parking_spaces:
            if space["occupied"]:
                continue
            cx, cy = space["center"]
            phi = space["phi"]
            l, w = space["l"], space["w"]
            for px, py in global_points:
                dx = px - cx
                dy = py - cy
                x_local = dx * math.cos(phi) + dy * math.sin(phi)
                y_local = -dx * math.sin(phi) + dy * math.cos(phi)
                if -l / 2 <= x_local <= l / 2 and -w / 2 <= y_local <= w / 2:
                    space["occupied"] = True
                    rospy.loginfo(f"[Parking] Space {space['id']} is now occupied.")
                    break

        if self.highway_mode:
            angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
            front_idx = np.where((angles >= -0.1745) & (angles <= 0.1745))[0]
            front_ranges = np.array(msg.ranges)[front_idx]
            if any(r < 1.0 and r > msg.range_min for r in front_ranges):
                self.obstacle_detected = True
                rospy.loginfo("[Obstacle] Detected ahead in highway mode.")
            else:
                self.obstacle_detected = False

    def apply_cubic_spline(self, path_coords):
        if len(path_coords) < 3:
            return path_coords
        path_array = np.array(path_coords)
        distances = np.sqrt(np.sum(np.diff(path_array, axis=0) ** 2, axis=1))
        t = np.concatenate(([0], np.cumsum(distances)))
        cs_x = CubicSpline(t, path_array[:, 0])
        cs_y = CubicSpline(t, path_array[:, 1])
        t_new = np.linspace(0, t[-1], num=100)
        new_x = cs_x(t_new)
        new_y = cs_y(t_new)
        return list(zip(new_x, new_y))

    def build_parking_path(self, start_node, end_node):
        import heapq
        dist = {n: float('inf') for n in self.graph_nodes.keys()}
        prev = {n: None for n in self.graph_nodes.keys()}
        dist[start_node] = 0.0
        pq = [(0.0, start_node)]
        while pq:
            cur_dist, cur_node = heapq.heappop(pq)
            if cur_dist > dist[cur_node]:
                continue
            if cur_node == end_node:
                break
            neighbors = self.entry_to_parking.get(cur_node, []) if cur_node in self.entry_to_parking else []
            for nxt in neighbors:
                cost = math.hypot(self.graph_nodes[nxt][0] - self.graph_nodes[cur_node][0],
                                  self.graph_nodes[nxt][1] - self.graph_nodes[cur_node][1])
                alt = dist[cur_node] + cost
                if alt < dist[nxt]:
                    dist[nxt] = alt
                    prev[nxt] = cur_node
                    heapq.heappush(pq, (alt, nxt))
        path_nodes = []
        n = end_node
        while n is not None:
            path_nodes.append(n)
            n = prev[n]
        path_nodes.reverse()
        raw_path = [self.graph_nodes[n] for n in path_nodes]
        return self.apply_cubic_spline(raw_path)

    def build_xref(self, path_xy, near_i, st, is_parking=False):
        T = self.mpc.T
        xref = np.zeros((3, T + 1))
        n = len(path_xy)
        for i in range(T + 1):
            idx = min(near_i + i, n - 1)
            xref[0, i] = path_xy[idx][0]
            xref[1, i] = path_xy[idx][1]
            if i == 0:
                xref[2, i] = st.yaw
            else:
                if is_parking and idx == n - 1:
                    xref[2, i] = 0.0
                else:
                    dx = path_xy[idx][0] - path_xy[idx - 1][0]
                    dy = path_xy[idx][1] - path_xy[idx - 1][1]
                    prev_yaw = xref[2, i - 1]
                    new_yaw = math.atan2(dy, dx)
                    xref[2, i] = prev_yaw + math.atan2(math.sin(new_yaw - prev_yaw),
                                                       math.cos(new_yaw - prev_yaw))
        return xref

    def get_nearest_changeable_node(self, x, y):
        min_dist = float('inf')
        nearest_node_id = None
        for node_id, (nx, ny) in self.changeable_nodes.items():
            dist = math.hypot(x - nx, y - ny)
            if dist < min_dist:
                min_dist = dist
                nearest_node_id = node_id
        next_node_id = nearest_node_id + 2 if nearest_node_id < max(self.changeable_nodes.keys()) else nearest_node_id
        return next_node_id

    def build_lane_change_path(self, x, y, nearest_node_id):
        target_node = self.changeable_nodes[nearest_node_id]
        path = [(x, y), target_node]
        return self.apply_cubic_spline(path)

    def control_loop(self, event):
        if not (self.loc_ok and self.yaw_ok) or not self.global_path_ok:
            return

        st = StateStruct(self.x, self.y, self.yaw)

        if not self.in_parking_mode and not self.in_exit_parking_mode and not self.in_lane_change_mode:
            near_i = self.get_nearest_idx(st.x, st.y, self.global_path, self.global_idx)
            self.global_idx = near_i
            xref = self.build_xref(self.global_path, near_i, st, is_parking=False)
            self.visualize_xref(xref)
            self.mpc.set_scenario("driving")
            x0 = np.array([st.x, st.y, st.yaw])
            (v_traj, steer_traj), _ = self.mpc.solve_mpc(x0, xref)
            v_cmd = v_traj[0] if v_traj is not None else 0.0
            s_cmd = steer_traj[0] if steer_traj is not None else 0.0

            if self.highway_mode and self.obstacle_detected:
                nearest_node_id = self.get_nearest_changeable_node(st.x, st.y)
                self.lane_change_path = self.build_lane_change_path(st.x, st.y, nearest_node_id)
                self.in_lane_change_mode = True
                self.lane_change_idx = 0
                rospy.loginfo("[Lane Change] Starting lane change to avoid obstacle.")

        elif self.in_lane_change_mode:
            near_i = self.get_nearest_idx(st.x, st.y, self.lane_change_path, self.lane_change_idx)
            self.lane_change_idx = near_i
            xref = self.build_xref(self.lane_change_path, near_i, st, is_parking=False)
            self.visualize_xref(xref)
            self.mpc.set_scenario("driving")
            x0 = np.array([st.x, st.y, st.yaw])
            (v_traj, steer_traj), _ = self.mpc.solve_mpc(x0, xref)
            v_cmd = v_traj[0] if v_traj is not None else 0.0
            s_cmd = steer_traj[0] if steer_traj is not None else 0.0

            last_x, last_y = self.lane_change_path[-1]
            dist_end = math.hypot(st.x - last_x, st.y - last_y)
            if dist_end < 0.15:
                rospy.loginfo("[Lane Change] Completed. Preparing to return to global path.")
                return_idx = self.get_nearest_idx(st.x, st.y, self.global_path, 0)+7
                return_point = self.global_path[return_idx]
                spline_path = self.apply_cubic_spline([(st.x, st.y), return_point])
                self.lane_change_path=spline_path
                self.in_lane_change_mode = False
                self.global_idx = self.get_nearest_idx(st.x, st.y, self.global_path, self.global_idx)
                rospy.loginfo("Lane change finished => back to driving")
                return

        elif self.in_parking_mode:
            if len(self.parking_path) < 2:
                rospy.logwarn("[Parking] no valid path => back to driving")
                self.in_parking_mode = False
                return
            near_i = self.get_nearest_idx(st.x, st.y, self.parking_path, self.parking_idx)
            self.parking_idx = near_i
            xref = self.build_xref(self.parking_path, near_i, st, is_parking=True)
            self.visualize_xref(xref)
            self.mpc.set_scenario("parking")
            x0 = np.array([st.x, st.y, st.yaw])
            (v_traj, steer_traj), _ = self.mpc.solve_mpc(x0, xref)
            v_cmd = v_traj[0] if v_traj is not None else 0.0
            s_cmd = steer_traj[0] if steer_traj is not None else 0.0

            last_x, last_y = self.parking_path[-1]
            dist_end = math.hypot(st.x - last_x, st.y - last_y)
            yaw_error = abs(st.yaw)
            if dist_end < 0.15 and yaw_error < math.radians(10):
                rospy.loginfo("[Parking] => park complete. Switching to exit mode.")
                self.in_parking_mode = False
                self.in_exit_parking_mode = True
                for _ in range(100000):
                    cmd_dict_1 = {'action': '1', 'speed': 0}
                    self.cmd_pub.publish(json.dumps(cmd_dict_1))
                self.exit_parking_path = list(reversed(self.parking_path))
                self.exit_parking_idx = 0
                self.visualize_parking_path(self.exit_parking_path)

        elif self.in_exit_parking_mode:
            if len(self.exit_parking_path) < 2:
                rospy.logwarn("[Parking Exit] no valid exit path => back to driving")
                self.in_exit_parking_mode = False
                self.global_idx = self.get_nearest_idx(st.x, st.y, self.global_path, self.global_idx)
                return
            near_i = self.get_nearest_idx(st.x, st.y, self.exit_parking_path, self.exit_parking_idx)
            self.exit_parking_idx = near_i
            xref = self.build_xref(self.exit_parking_path, near_i, st, is_parking=True)
            self.visualize_xref(xref)
            self.mpc.set_scenario("parking")
            x0 = np.array([st.x, st.y, st.yaw])
            (v_traj, steer_traj), _ = self.mpc.solve_mpc(x0, xref)
            v_cmd = v_traj[0] if v_traj is not None else 0.0
            s_cmd = steer_traj[0] if steer_traj is not None else 0.0

            exit_target = self.exit_parking_path[-1]
            dist_exit = math.hypot(st.x - exit_target[0], st.y - exit_target[1])
            if dist_exit < 0.15:
                rospy.loginfo("[Parking Exit] => exit complete. Returning to global path.")
                self.in_exit_parking_mode = False
                self.global_idx = self.get_nearest_idx(st.x, st.y, self.global_path, self.global_idx)

        steer_deg = math.degrees(s_cmd)
        cmd_dict_1 = {'action': '1', 'speed': float(v_cmd)}
        self.cmd_pub.publish(json.dumps(cmd_dict_1))
        cmd_dict_2 = {'action': '2', 'steerAngle': float(-steer_deg)}
        self.cmd_pub.publish(json.dumps(cmd_dict_2))

    def get_nearest_idx(self, x, y, path, start_i):
        if start_i >= len(path):
            return len(path) - 1
        return min(range(start_i, len(path)), key=lambda i: (x - path[i][0])**2 + (y - path[i][1])**2)

    def visualize_xref(self, xref):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for i in range(xref.shape[1]):
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = xref[0, i]
            ps.pose.position.y = xref[1, i]
            path_msg.poses.append(ps)
        self.xref_pub.publish(path_msg)

    def visualize_parking_path(self, parking_path):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for (x, y) in parking_path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.parking_path_pub.publish(path_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = MPCNode()
    node.run()