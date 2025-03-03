#!/usr/bin/env python3
import rospy
import math
import numpy as np
import json
import casadi as ca

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from utils.msg import localisation

def quaternion_to_yaw(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)

class State:
    """
    Holds the vehicle state: x, y, yaw
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

class NonlinearMPCController:
    """
    Kinematic Bicycle Model with 2 inputs: [v, steer].
    - State = [x, y, yaw]
    - Control = [v, steer]
    - Solver: CasADi + IPOPT
    """
    def __init__(self, dt=0.2, horizon=5, wheelbase=0.26):
        self.dt = dt
        self.T = horizon
        self.wb = wheelbase

        # --- 비용 가중치(예시값) ---
        self.Qx = 700.0
        self.Qy = 700.0
        self.Qyaw = 500.0

        # 제어 입력 비용 가중치
        self.Rv = 0.01       # 속도 입력에 대한 비용
        self.Rsteer = 0.01   # 조향 입력에 대한 비용

        # 제어 변화율 비용 가중치
        self.Rdv = 0.005
        self.Rdsteer = 0.005

        # 최대/최소 속도
        self.v_min = -0.1
        self.v_max = 0.3

        # 최대 스티어링(±25 deg)
        self.max_steer = math.radians(25.0)
        self.min_steer = -self.max_steer

        # CasADi Symbolic Setup
        self.nx = 3  # (x, y, yaw)
        self.nu = 2  # (v, steer)

    def solve_mpc(self, x0, xref):
        """
        x0: 초기 상태 [x, y, yaw]
        xref: shape=(3, T+1), MPC horizon 동안 추종하고자 하는 레퍼런스 (x, y, yaw)
        return: (v_array, steer_array), state_array
        """

        T = self.T
        dt = self.dt
        wb = self.wb

        # 1) 최적화 변수: 상태(T+1) + 제어(T)
        #    상태 = 3 * (T+1), 제어 = 2 * T
        n_vars = (self.nx)*(T+1) + (self.nu)*T
        opt_x = ca.SX.sym('opt_x', n_vars)

        # Helper index
        def state_idx(k):
            return self.nx * k

        def control_idx(k):
            return self.nx*(T+1) + self.nu*k

        # 2) 비용함수 초기화
        obj = 0.0

        # 3) 제약조건 모음
        g = []
        lbg = []
        ubg = []

        # 3.1) 초기 상태 고정: (x(0), y(0), yaw(0)) = x0
        x_init = opt_x[state_idx(0) + 0]
        y_init = opt_x[state_idx(0) + 1]
        yaw_init = opt_x[state_idx(0) + 2]

        g += [x_init - x0[0], y_init - x0[1], yaw_init - x0[2]]
        lbg += [0.0, 0.0, 0.0]
        ubg += [0.0, 0.0, 0.0]

        # 동역학 함수 정의
        def f(st, con):
            """
            st = [x, y, yaw]
            con = [v, steer]
            """
            x, y, yaw = st[0], st[1], st[2]
            v, steer = con[0], con[1]
            dx = v * ca.cos(yaw)
            dy = v * ca.sin(yaw)
            dyaw = (v / wb) * steer  # tan(steer) 대신 steer
            return ca.vertcat(dx, dy, dyaw)

        # 3.2) RK4로 다음 상태 제약
        for k in range(T):
            # 현재 상태
            st_k = opt_x[state_idx(k): state_idx(k)+3]
            # 제어
            con_k = opt_x[control_idx(k): control_idx(k)+2]
            # 다음 상태
            st_next = opt_x[state_idx(k+1): state_idx(k+1)+3]

            # RK4
            k1 = f(st_k, con_k)
            k2 = f(st_k + (dt/2)*k1, con_k)
            k3 = f(st_k + (dt/2)*k2, con_k)
            k4 = f(st_k + dt*k3, con_k)
            st_next_RK4 = st_k + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

            # 제약: st_next = st_next_RK4
            g += [st_next - st_next_RK4]
            lbg += [0.0, 0.0, 0.0]
            ubg += [0.0, 0.0, 0.0]

        # 4) 비용함수 작성
        #    (xref - xk), (uref - uk) 등
        for k in range(T+1):
            xk = opt_x[state_idx(k) + 0]
            yk = opt_x[state_idx(k) + 1]
            yawk = opt_x[state_idx(k) + 2]

            xr = xref[0, k]
            yr = xref[1, k]
            yr_yaw = xref[2, k]

            # 위치/각도 오차 비용
            obj += self.Qx * (xk - xr)**2
            obj += self.Qy * (yk - yr)**2
            obj += self.Qyaw * (yawk - yr_yaw)**2

        # 제어 입력 비용 + 제어 변화율 비용
        for k in range(T):
            vk = opt_x[control_idx(k) + 0]
            steer_k = opt_x[control_idx(k) + 1]
            # 입력 자체 비용
            obj += self.Rv * (vk**2)
            obj += self.Rsteer * (steer_k**2)

            if k < T-1:
                v_next = opt_x[control_idx(k+1) + 0]
                steer_next = opt_x[control_idx(k+1) + 1]
                obj += self.Rdv * ((v_next - vk)**2)
                obj += self.Rdsteer * ((steer_next - steer_k)**2)

        # 5) bound 생성
        #   상태 (x, y, yaw)는 제한 없이 -inf ~ inf
        #   제어 (v, steer)는 하단/상단 제한
        lbx = []
        ubx = []
        # 상태 (T+1)
        for k in range(T+1):
            # x, y
            lbx += [-1.0e6, -1.0e6]
            ubx += [1.0e6, 1.0e6]
            # yaw
            lbx += [-1.0e6]
            ubx += [1.0e6]

        # 제어 (T)
        for k in range(T):
            # v
            lbx += [self.v_min]
            ubx += [self.v_max]
            # steer
            lbx += [self.min_steer]
            ubx += [self.max_steer]

        # 6) NLP 설정
        nlp_dict = {
            'f': obj,
            'x': opt_x,
            'g': ca.vertcat(*g)
        }

        solver_opts = {
            'ipopt': {
                'max_iter': 200,
                'acceptable_tol': 1e-6,
                'acceptable_obj_change_tol': 1e-6,
                # 'print_level': 0
            }
            # 'print_time': False
        }
        solver = ca.nlpsol('solver', 'ipopt', nlp_dict, solver_opts)

        # 7) 초기 추정값
        x_init_guess = []
        for k in range(T+1):
            alpha = k / float(T+1)
            xg = x0[0] * (1-alpha) + xref[0,-1] * alpha
            yg = x0[1] * (1-alpha) + xref[1,-1] * alpha
            ygaw = x0[2] * (1-alpha) + xref[2,-1] * alpha
            x_init_guess += [xg, yg, ygaw]

        for k in range(T):
            # v, steer 각각 0으로 시작
            x_init_guess += [0.0, 0.0]

        # 8) Solve
        sol = solver(
            x0=x_init_guess,
            lbx=lbx, ubx=ubx,
            lbg=lbg, ubg=ubg
        )

        if solver.stats()['return_status'] not in ['Solve_Succeeded', 'Optimal_Solution_Found']:
            rospy.logwarn("IPOPT failed. status=" + solver.stats()['return_status'])
            return None, None

        sol_x = sol['x'].full().flatten()

        # 9) 결과 파싱
        #    제어: v_array, steer_array
        #    상태: state_array
        all_states = sol_x[: self.nx*(T+1)].reshape((T+1, self.nx)).T  # (3, T+1)
        all_controls = sol_x[self.nx*(T+1): ].reshape((T, self.nu)).T  # (2, T)

        # [v1, v2, ..., vT], [steer1, steer2, ..., steerT]
        v_traj = all_controls[0, :]
        steer_traj = all_controls[1, :]

        return (v_traj, steer_traj), all_states


# ------------------------- ROS Node (예시) -------------------------
class MPCNode:
    def __init__(self):
        rospy.init_node("mpc_lateral_node", anonymous=True)

        # horizon, dt 등은 상황 맞춰 조정
        self.mpc = NonlinearMPCController(dt=0.25, horizon=4, wheelbase=0.26)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.loc_ok = False
        self.yaw_ok = False

        self.global_path = []
        self.path_ok = False
        self.target_i = 0

        self.path_sub = rospy.Subscriber("/global_path", Path, self.path_callback, queue_size=1)
        self.loc_sub = rospy.Subscriber("/automobile/localisation", localisation, self.loc_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber("/automobile/IMU", Imu, self.imu_callback, queue_size=1)

        self.cmd_pub = rospy.Publisher("/automobile/command", String, queue_size=10)
        self.xref_pub = rospy.Publisher("/mpc_xref", Path, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def path_callback(self, msg: Path):
        self.global_path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.path_ok = (len(self.global_path) > 2)

    def loc_callback(self, msg: localisation):
        x_c, y_c = msg.posA, msg.posB
        theta = self.yaw
        # 차량의 rear-axle 기준 좌표로 변환 (필요 시)
        x_rear = x_c - (0.26 * 0.5) * math.cos(theta)
        y_rear = y_c - (0.26 * 0.5) * math.sin(theta)
        self.x = x_rear
        self.y = y_rear
        self.loc_ok = True

    def imu_callback(self, imu_msg: Imu):
        q = imu_msg.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.yaw_ok = True

    def control_loop(self, event):
        if not (self.path_ok and self.loc_ok and self.yaw_ok):
            return
        if len(self.global_path) < 2:
            return

        st = State(self.x, self.y, self.yaw)
        near_i = self.get_nearest_idx(st.x, st.y, self.global_path, self.target_i)
        self.target_i = near_i

        xref = self.build_xref(near_i, st)
        self.visualize_xref(xref)

        x0 = np.array([st.x, st.y, st.yaw])
        (v_traj, steer_traj), state_traj = self.mpc.solve_mpc(x0, xref)
        if v_traj is None:
            rospy.logwarn("MPC failed, use safe command")
            v_cmd = 0.0
            steer_cmd = 0.0
        else:
            # 첫 시점의 해
            v_cmd = v_traj[0]
            steer_cmd = steer_traj[0]

        # speed_cmd, steer_deg
        steer_deg = math.degrees(steer_cmd)
        rospy.loginfo(f"[MPC] => v={v_cmd:.2f} m/s, steer={steer_cmd:.2f} rad => {steer_deg:.2f} deg")

        # 아래 예시는 command가 JSON 형태라는 가정 하에 작성
        cmd_dict_1 = {'action': '1', 'speed': float(v_cmd)}
        self.cmd_pub.publish(json.dumps(cmd_dict_1))
        cmd_dict_2 = {'action': '2', 'steerAngle': float(-steer_deg)}
        self.cmd_pub.publish(json.dumps(cmd_dict_2))

    def build_xref(self, near_i, st: State):
        T = self.mpc.T
        xref = np.zeros((3, T+1))
        n = len(self.global_path)
        for i in range(T+1):
            idx = min(near_i + i, n-1)
            xref[0, i] = self.global_path[idx][0]
            xref[1, i] = self.global_path[idx][1]
            if i == 0:
                xref[2, i] = st.yaw
            else:
                dx = self.global_path[idx][0] - self.global_path[idx-1][0]
                dy = self.global_path[idx][1] - self.global_path[idx-1][1]
                prev_yaw = xref[2, i-1]
                new_yaw = math.atan2(dy, dx)
                # 부드럽게 연결
                xref[2, i] = prev_yaw + math.atan2(math.sin(new_yaw - prev_yaw),
                                                   math.cos(new_yaw - prev_yaw))
        return xref

    def visualize_xref(self, xref):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()
        for i in range(xref.shape[1]):
            px = xref[0, i]
            py = xref[1, i]
            ps = PoseStamped()
            ps.header.frame_id = path_msg.header.frame_id
            ps.header.stamp = path_msg.header.stamp
            ps.pose.position.x = px
            ps.pose.position.y = py
            path_msg.poses.append(ps)
        self.xref_pub.publish(path_msg)

    def get_nearest_idx(self, x, y, path, start_i):
        if start_i == 0:
            return min(range(len(path)), key=lambda i: (x - path[i][0])**2 + (y - path[i][1])**2)
        search = 10
        return min(range(max(0, start_i-5), min(start_i+search, len(path))),
                   key=lambda i: (x - path[i][0])**2 + (y - path[i][1])**2)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = MPCNode()
    node.run()
    