#!/usr/bin/env python3
import rospy
import math
import numpy as np
import json
import casadi as ca
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Bool
from utils.msg import localisation

def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

# ------------------ MPC Controller ------------------
class NonlinearMPCController:
    def __init__(self, dt=0.25, horizon=8, wheelbase=0.26):
        self.dt = dt
        self.T = horizon
        self.wb = wheelbase
        self.scenario = 'driving'
        self.set_scenario(self.scenario)
        self.nx = 3  # (x, y, yaw)
        self.nu = 2  # (v, steer)

    def set_scenario(self, scenario):
        self.scenario = scenario
        if scenario == "parking":
            rospy.loginfo("[MPC] => parking param set")
            self.T = 12  # 예측 지평선 증가
            self.Qx = 40.0  # x 오차 가중치 증가
            self.Qy = 70.0  # y 오차 가중치 더 크게 설정
            self.Qyaw = 2  # yaw 가중치는 적당히
            self.Rv = 0.005
            self.Rsteer = 0.005
            self.Rdv = 0.001
            self.Rdsteer = 0.001
            self.v_min = -0.25
            self.v_max = 0.2
            self.max_steer = math.radians(25.0)
            self.min_steer = -self.max_steer
        else:
            rospy.loginfo("[MPC] => driving param set")
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
            dx = v*ca.cos(yaw)
            dy = v*ca.sin(yaw)
            dyaw = (v/wb)*steer
            return ca.vertcat(dx, dy, dyaw)

        for k in range(T):
            st_k = opt_x[sidx(k):sidx(k)+3]
            con_k = opt_x[cidx(k):cidx(k)+2]
            st_next = opt_x[sidx(k+1): sidx(k+1)+3]
            k1 = f(st_k, con_k)
            k2 = f(st_k+(dt/2)*k1, con_k)
            k3 = f(st_k+(dt/2)*k2, con_k)
            k4 = f(st_k+dt*k3, con_k)
            st_rk4 = st_k+(dt/6)*(k1+2*k2+2*k3+k4)
            g += [st_next - st_rk4]
            lbg += [0.0, 0.0, 0.0]
            ubg += [0.0, 0.0, 0.0]

        for k in range(T+1):
            xk, yk, yawk = opt_x[sidx(k)+0], opt_x[sidx(k)+1], opt_x[sidx(k)+2]
            xr, yr, yr_yaw = xref[0, k], xref[1, k], xref[2, k]
            obj += self.Qx*(xk - xr)**2 + self.Qy*(yk - yr)**2 + self.Qyaw*(yawk - yr_yaw)**2

        for k in range(T):
            vk, steer_k = opt_x[cidx(k)+0], opt_x[cidx(k)+1]
            obj += self.Rv*(vk**2) + self.Rsteer*(steer_k**2)
            if k < T-1:
                v_next, st_next = opt_x[cidx(k+1)+0], opt_x[cidx(k+1)+1]
                obj += self.Rdv*((v_next - vk)**2) + self.Rdsteer*((st_next - steer_k)**2)

        lbx = [-1e6]*((T+1)*self.nx) + [self.v_min, self.min_steer]*T
        ubx = [1e6]*((T+1)*self.nx) + [self.v_max, self.max_steer]*T

        nlp = {'f': obj, 'x': opt_x, 'g': ca.vertcat(*g)}
        solver = ca.nlpsol('solver', 'ipopt', nlp, {
            'ipopt': {'max_iter': 200, 'acceptable_tol': 1e-6, 'acceptable_obj_change_tol': 1e-6}
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
        all_controls = solx[self.nx*(T+1): ].reshape((T, self.nu)).T
        return (all_controls[0, :], all_controls[1, :]), None

# ------------------ ROS Node ------------------
class StateStruct:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

class MPCNode:
    def __init__(self):
        rospy.init_node("mpc_lateral_node", anonymous=True)
        self.mpc = NonlinearMPCController(dt=0.25, horizon=10, wheelbase=0.26)
        
        self.in_parking_mode = False
        self.parking_path = []
        self.parking_idx = 0
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.loc_ok = False
        self.yaw_ok = False
        
        self.global_path = []
        self.global_path_ok = False
        self.global_idx = 0
        
        self.graph_nodes = {
            "232": (10.82, 0.92),
            "901": (10.04, 0.54),
            "233": (10.5, 0.92),
        }
        self.graph_edges = {
            "232": ["901"],
            "901": ["232", "233"],
            "233": ["901"]
        }
        
        self.path_sub = rospy.Subscriber("/global_path", Path, self.path_callback, queue_size=1)
        self.loc_sub = rospy.Subscriber("/automobile/localisation", localisation, self.loc_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber("/automobile/IMU", Imu, self.imu_callback, queue_size=1)
        self.park_sub = rospy.Subscriber("/parking_signal", Bool, self.park_signal_cb, queue_size=1)
        self.cmd_pub = rospy.Publisher("/automobile/command", String, queue_size=10)
        self.parking_path_pub = rospy.Publisher("/parking_path", Path, queue_size=1)
        self.xref_pub = rospy.Publisher("/mpc_xref", Path, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def park_signal_cb(self, msg: Bool):
        self.parking_signal = msg.data
        rospy.loginfo(f"[Parking] => parking signal {'ON' if msg.data else 'OFF'}")

    def path_callback(self, msg: Path):
        self.global_path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.global_path_ok = len(self.global_path) > 2

    def loc_callback(self, msg: localisation):
        x_c, y_c = msg.posA, msg.posB
        theta = self.yaw
        wb = 0.26
        self.x = x_c - 0.5*wb*math.cos(theta)
        self.y = y_c - 0.5*wb*math.sin(theta)
        self.loc_ok = True

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.yaw_ok = True

    def build_parking_path(self, start_node, end_node):
        """주차 경로 생성 후 Bezier 곡선 적용"""
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
            if cur_node not in self.graph_edges:
                continue
            neighbors = self.graph_edges[cur_node]
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
        return self.apply_bezier(raw_path)  # Bezier 곡선으로 부드럽게 보정
    def apply_bezier(self, path_coords):
        """주차 경로를 Bezier 곡선으로 부드럽게 보정"""
        if len(path_coords) < 3:
            return path_coords
        new_path = []
        for i in range(0, len(path_coords) - 2, 1):  # 각 구간마다 제어점 활용
            p0 = path_coords[i]
            p1 = path_coords[i + 1]
            p2 = path_coords[i + 2] if i + 2 < len(path_coords) else path_coords[-1]
            for t in np.linspace(0, 1, 10):  # 각 구간에 10개의 보간 점 추가
                B_x = (1-t)**2 * p0[0] + 2*(1-t)*t * p1[0] + t**2 * p2[0]
                B_y = (1-t)**2 * p0[1] + 2*(1-t)*t * p1[1] + t**2 * p2[1]
                new_path.append((B_x, B_y))
        new_path.append(path_coords[-1])  # 마지막 점 유지
        return new_path
    def build_xref(self, path_xy, near_i, st, is_parking=False):
        T = self.mpc.T
        xref = np.zeros((3, T+1))
        n = len(path_xy)
        for i in range(T+1):
            idx = min(near_i + i, n-1)
            xref[0, i] = path_xy[idx][0]
            xref[1, i] = path_xy[idx][1]
            if i == 0:
                xref[2, i] = st.yaw
            else:
                if is_parking and idx == n-1:
                    xref[2, i] = 0.0  # 마지막 지점에서만 yaw=0
                else:
                    dx = path_xy[idx][0] - path_xy[idx-1][0]
                    dy = path_xy[idx][1] - path_xy[idx-1][1]
                    prev_yaw = xref[2, i-1]
                    new_yaw = math.atan2(dy, dx)
                    xref[2, i] = prev_yaw + math.atan2(math.sin(new_yaw - prev_yaw), math.cos(new_yaw - prev_yaw))
        return xref

    def control_loop(self, event):
        if not (self.loc_ok and self.yaw_ok) or not self.global_path_ok:
            return

        st = StateStruct(self.x, self.y, self.yaw)

        if not self.in_parking_mode:
            near_i = self.get_nearest_idx(st.x, st.y, self.global_path, self.global_idx)
            self.global_idx = near_i
            xref = self.build_xref(self.global_path, near_i, st, is_parking=False)
            self.visualize_xref(xref)

            self.mpc.set_scenario("driving")
            x0 = np.array([st.x, st.y, st.yaw])
            (v_traj, steer_traj), _ = self.mpc.solve_mpc(x0, xref)
            v_cmd = v_traj[0] if v_traj is not None else 0.0
            s_cmd = steer_traj[0] if steer_traj is not None else 0.0

            dist_232 = math.hypot(st.x - 10.84, st.y - 0.92)
            if dist_232 < 0.2 and getattr(self, 'parking_signal', False):
                rospy.loginfo("[Parking] => Switch to parking mode: 232->901")
                self.in_parking_mode = True
                self.parking_path = self.build_parking_path('232', '901')
                self.parking_idx = 0
                self.visualize_parking_path(self.parking_path)

        else:
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
            rospy.loginfo(f"[Parking] dist_end={dist_end:.3f}, yaw_error={math.degrees(yaw_error):.2f} deg")
            if dist_end < 0.15 and yaw_error < math.radians(15):
                rospy.loginfo("[Parking] => park complete => back to driving")
                self.in_parking_mode = False
                self.global_idx = self.get_nearest_idx(st.x, st.y, self.global_path, self.global_idx)

        steer_deg = math.degrees(s_cmd)
        rospy.loginfo(f"[MPC] parking={self.in_parking_mode}, v={v_cmd:.2f}, steer={s_cmd:.2f} rad => {steer_deg:.2f} deg")
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