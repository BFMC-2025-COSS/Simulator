#!/usr/bin/env python3
import rospy
import math
import numpy as np
import json
import casadi as ca

from nav_msgs.msg import Path as GlobalPath
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from utils.msg import localisation
from reeds_shepp_planner import ReedsSheppPathPlanner,Path


def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Convert quaternion to yaw angle (radians).
    """
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
    Kinematic Bicycle Model (lateral-only, constant speed) with RK4:
      x_{k+1}, y_{k+1}, yaw_{k+1} are updated using 4th-order Runge-Kutta method.
    - State = [x, y, yaw]
    - Control = [steer]
    - Solver: CasADi + IPOPT
    """
    def __init__(self, dt=0.2, horizon=5, wheelbase=0.26):
        self.dt = dt
        self.T = horizon
        self.wb = wheelbase

        # Cost weights
        self.Qx = 700.0
        self.Qy = 700.0
        self.Qyaw = 500.0
        self.Rsteer = 0.005
        self.Rd_steer = 0.002  # input change rate cost

        # Constraints
        self.max_steer = math.radians(25.0)  # ±25 deg
        self.max_dsteer = math.radians(30.0) # ±30 deg/s
        self.speed = 0.2  # m/s (상수)

        # CasADi Symbolic Setup
        self.nx = 3  # (x, y, yaw)
        self.nu = 1  # (steer)

    def solve_mpc(self, x0, xref):
        """
        x0: 초기 상태 [x, y, yaw]
        xref: shape=(3, T+1), MPC horizon 동안 추종하고자 하는 레퍼런스
        return: steer_array (길이 T), state_array (3 x (T+1)) 예측값
        """
        T = self.T
        dt = self.dt
        wb = self.wb
        v = self.speed

        # ---- 1) CasADi 변수 정의 ----
        n_vars = (self.nx)*(T+1) + (self.nu)*T
        opt_x = ca.SX.sym('opt_x', n_vars)

        # Helper 함수: 인덱스 접근
        def state_idx(k):
            return self.nx * k
        def control_idx(k):
            return self.nx*(T+1) + self.nu*k

        # ---- 2) 목적함수 (cost) 정의용 변수 및 식 초기화 ----
        obj = 0.0

        # ---- 3) 제약조건 g( opt_x ) = 0 (혹은 부등호) / bounds ----
        g = []
        lbg = []
        ubg = []

        # ---- 3.1) 초기 상태 고정: (x(0), y(0), yaw(0)) = x0
        x_init = opt_x[state_idx(0) + 0]
        y_init = opt_x[state_idx(0) + 1]
        yaw_init = opt_x[state_idx(0) + 2]
        g += [x_init - x0[0], y_init - x0[1], yaw_init - x0[2]]
        lbg += [0.0, 0.0, 0.0]
        ubg += [0.0, 0.0, 0.0]

        # ---- 3.2) 동역학 제약: RK4 적용 ----
        # 동역학 함수 정의
        def f(st, con):
            x, y, yaw = st[0], st[1], st[2]
            steer = con[0]
            dx = v * ca.cos(yaw)
            dy = v * ca.sin(yaw)
            dyaw = (v / wb) * steer  # 단순화된 모델 (tan(steer) 대신 steer 사용)
            return ca.vertcat(dx, dy, dyaw)

        for k in range(T):
            # 현재 상태
            st = ca.vertcat(opt_x[state_idx(k) + 0],
                            opt_x[state_idx(k) + 1],
                            opt_x[state_idx(k) + 2])
            # 제어 입력
            con = ca.vertcat(opt_x[control_idx(k) + 0])
            # 다음 상태
            st_next = ca.vertcat(opt_x[state_idx(k+1) + 0],
                                 opt_x[state_idx(k+1) + 1],
                                 opt_x[state_idx(k+1) + 2])

            # RK4 계산
            k1 = f(st, con)
            k2 = f(st + (dt/2) * k1, con)
            k3 = f(st + (dt/2) * k2, con)
            k4 = f(st + dt * k3, con)
            st_next_RK4 = st + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

            # 제약 조건: st_next = st_next_RK4
            g += [st_next - st_next_RK4]
            lbg += [0.0, 0.0, 0.0]
            ubg += [0.0, 0.0, 0.0]

        # ---- 4) 비용함수 obj에 (xref - xk), (uref - uk)등을 반영 ----
        for k in range(T+1):
            xk = opt_x[state_idx(k)+0]
            yk = opt_x[state_idx(k)+1]
            yawk = opt_x[state_idx(k)+2]
            xr = xref[0, k]
            yr = xref[1, k]
            yr_yaw = xref[2, k]
            obj += self.Qx * (xk - xr)**3
            obj += self.Qy * (yk - yr)**3
            obj += self.Qyaw * (yawk - yr_yaw)**2

        # 제어 입력 비용 + 제어 변화율 비용
        for k in range(T):
            steer_k = opt_x[control_idx(k)]
            obj += self.Rsteer * (steer_k**2)
            if k < T-1:
                steer_next = opt_x[control_idx(k+1)]
                dsteer = steer_next - steer_k
                obj += self.Rd_steer * (dsteer**2)

        # ---- 5) 스티어링 범위 제약 ----
        # (변수 bounds로 처리하므로 pass 유지)

        # ---- 6) 변수를 위한 lower bound / upper bound 만들기 ----
        lbx = []
        ubx = []
        for k in range(T+1):
            lbx += [-1.0e6, -1.0e6]
            ubx += [1.0e6, 1.0e6]
            lbx += [-1.0e6]
            ubx += [1.0e6]
        for k in range(T):
            lbx += [-self.max_steer]
            ubx += [self.max_steer]

        # ---- 7) NLP 문제 정의 (CasADi) ----
        nlp_dict = {
            'f': obj,
            'x': opt_x,
            'g': ca.vertcat(*g)
        }

        solver_opts = {
            'print_time': 0,
            'ipopt': {
                'max_iter': 200,
                'acceptable_tol': 1e-6,
                'acceptable_obj_change_tol': 1e-6,
                'print_level': 0,
                'print_info_string': 'no', 
                'sb': 'yes', 
                'print_timing_statistics': 'no' # 타이밍 통계 출력 끄기
            }
            
        }
        solver = ca.nlpsol('solver', 'ipopt', nlp_dict, solver_opts)

        # ---- 8) 초기 추정값(initial guess) ----
        x_init_guess = []
        for k in range(T+1):
            alpha = k / float(T+1)
            xg = x0[0] * (1-alpha) + xref[0,-1] * alpha
            yg = x0[1] * (1-alpha) + xref[1,-1] * alpha
            ygaw = x0[2] * (1-alpha) + xref[2,-1] * alpha
            x_init_guess += [xg, yg, ygaw]
        for k in range(T):
            x_init_guess += [0.0]

        # ---- 9) solve ----
        sol = solver(lbx=lbx, ubx=ubx, lbg=lbg, ubg=ubg, x0=x_init_guess)

        if solver.stats()['return_status'] not in ['Solve_Succeeded', 'Optimal_Solution_Found']:
            rospy.logwarn("IPOPT failed. status=" + solver.stats()['return_status'])
            return None, None

        sol_x = sol['x'].full().flatten()

        # ---- 10) 결과 파싱 ----
        steer_array = sol_x[self.nx*(T+1) : self.nx*(T+1) + T]
        state_array = sol_x[0 : self.nx*(T+1)].reshape((T+1, self.nx)).T

        return steer_array, state_array

# ------------------------- ROS Node -------------------------
class MPCNode:
    def __init__(self):
        rospy.init_node("mpc_lateral_node", anonymous=True)
        self.desired_speed = rospy.get_param("~desired_speed", 0.2)
        self.mpc = NonlinearMPCController(dt=0.25, horizon=5, wheelbase=0.26)
        self.mpc.speed = self.desired_speed
        self.original_speed=self.desired_speed
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.loc_ok = False
        self.yaw_ok = False

        self.global_path = []
        self.path_ok = False
        self.target_i = 0
        self.parking_idx = 0

        self.mode='follow_global'   #global mode
        self.parking_path = []  # list for parking
        self.parking_target = None # target goal
        self.parking_planner = ReedsSheppPathPlanner(max_curvature=2, step_size=0.05,show_ani=False) 
        self.path_sub = rospy.Subscriber("/global_path", GlobalPath, self.path_callback, queue_size=1)
        self.loc_sub = rospy.Subscriber("/automobile/localisation", localisation, self.loc_callback, queue_size=1)
        self.imu_sub = rospy.Subscriber("/automobile/IMU", Imu, self.imu_callback, queue_size=1)

        self.parking_trigger_sub = rospy.Subscriber("/parking_trigger", String, self.parking_trigger_callback, queue_size=1)
        # should be changed to yolo data

        self.cmd_pub = rospy.Publisher("/automobile/command", String, queue_size=10)
        self.xref_pub = rospy.Publisher("/mpc_xref", GlobalPath, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.05), self.control_loop)

    def path_callback(self, msg: GlobalPath):
        self.global_path = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        self.path_ok = (len(self.global_path) > 2)

    def loc_callback(self, msg: localisation):
        x_c, y_c = msg.posA, msg.posB
        theta = self.yaw
        x_rear = x_c - (0.26 * 0.5) * math.cos(theta)
        y_rear = y_c - (0.26 * 0.5) * math.sin(theta)
        self.x = x_rear
        self.y = y_rear
        self.loc_ok = True

    def imu_callback(self, imu_msg: Imu):
        q = imu_msg.orientation
        self.yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.yaw_ok = True

    def parking_trigger_callback(self, msg):
        if msg.data.startswith("start_parking"):
            self.stop_vehicle()  # 차량 정지
            try:
                _, x, y, yaw = msg.data.split(',')
                self.parking_target = (float(x), float(y), float(yaw))
            except:
                # 기본 주차 목표 (현재 위치에서 1m 전진, 0.5m 우측, 90도 회전)
                self.parking_target = (self.x + 1.0, self.y + 0.5, self.yaw + math.pi/2)
            self.parking_path = self.generate_parking_path()
            if not self.parking_path:
                rospy.logwarn("Failed to generate parking path.")
                self.mode = 'follow_global'
            else:
                self.mode = 'parking'
                self.mpc.speed = 0.1  # 주차 시 속도 감소
                rospy.loginfo("Parking mode activated with reduced speed.")

    def stop_vehicle(self):
        cmd_dict = {'action': '1', 'speed': 0.0}
        self.cmd_pub.publish(json.dumps(cmd_dict))
        rospy.loginfo("Vehicle stopped for parking path calculation.")

    def generate_parking_path(self):
        if not self.parking_target:
            return []
        target_x, target_y, target_yaw = self.parking_target
        xs, ys, yaws, dirs, _ = self.parking_planner.reeds_shepp_path_planning(
            self.x, self.y, self.yaw, target_x, target_y, target_yaw,
            maxc=0.1, step_size=0.05
        )
        if xs is None:
            return []
        return list(zip(xs, ys, yaws, dirs))


    def control_loop(self, event):
        if not (self.path_ok and self.loc_ok and self.yaw_ok):
            return
        if len(self.global_path) < 2:
            return
        st = State(self.x, self.y, self.yaw)
        if self.mode == 'follow_global':
            near_i = self.get_nearest_idx(st.x, st.y, self.global_path, self.target_i)
            self.target_i = near_i
            xref = self.build_xref(near_i, st)
        elif self.mode=='parking':
            # 주차 경로 인덱스가 범위 초과하면 그냥 완료로 처리
            if self.parking_idx >= len(self.parking_path):
                rospy.loginfo("Parking path index out of range -> done.")
                self.finish_parking(st)
                return

            # 1) 현재 인덱스의 (x, y, yaw, dir)
            cur_x, cur_y, cur_yaw, cur_dir= self.parking_path[self.parking_idx]

            # 2) 속도 결정
            #   +1 => 전진, -1 => 후진
            self.mpc.speed= 0.1 * cur_dir

            # 3) MPC Horizon 구성
            xref= self.build_parking_xref(st, self.parking_idx)

            # 4) 현재 waypoint와의 거리 확인
            dist= math.hypot(st.x - cur_x, st.y - cur_y)
            yaw_diff= abs(st.yaw - cur_yaw)
            # 어느 정도 근접하면 다음 인덱스로
            if dist < 0.1 and yaw_diff < 0.3:
                self.parking_idx += 1

            # 5) 혹은 마지막 인덱스 근처인지 확인
            if self.parking_idx >= len(self.parking_path)-1:
                # 더 정확한 조건을 넣어도 됨
                last_x, last_y, last_yaw, _ = self.parking_path[-1]
                dist_end= math.hypot(st.x - last_x, st.y - last_y)
                yaw_end= abs(st.yaw - last_yaw)
                if dist_end<0.1 and yaw_end<0.3:
                    rospy.loginfo("Parking completed => back to global path")
                    self.finish_parking(st)
        self.visualize_xref(xref)
        x0 = np.array([st.x, st.y, st.yaw])
        steer_traj, state_traj = self.mpc.solve_mpc(x0, xref)
        if steer_traj is None:
            rospy.logwarn("MPC failed, use safe command")
            steer_cmd = 0.0
        else:
            steer_cmd = steer_traj[0]
        speed_cmd = self.mpc.speed
        steer_deg = math.degrees(steer_cmd)
        # rospy.loginfo(f"[MPC] => speed={speed_cmd:.2f}, steer={steer_cmd:.2f} rad => {steer_deg:.2f} deg")
        cmd_dict = {'action': '1', 'speed': float(speed_cmd)}
        self.cmd_pub.publish(json.dumps(cmd_dict))
        cmd_dict = {'action': '2', 'steerAngle': float(-steer_deg)}
        self.cmd_pub.publish(json.dumps(cmd_dict))
    def finish_parking(self, st):
        """
        주차 완료 시 후처리
        """
        self.mode= 'follow_global'
        self.parking_path= []
        self.parking_target= None
        self.parking_idx= 0
        self.mpc.speed= self.original_speed
        # global path의 최근점 재설정
        self.target_i= self.get_nearest_idx(st.x, st.y, self.global_path, self.target_i)

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
                xref[2, i] = prev_yaw + math.atan2(math.sin(new_yaw - prev_yaw),
                                                   math.cos(new_yaw - prev_yaw))
        return xref
    
    def build_parking_xref(self, st: State, start_idx):
        """
        parking_path는 (x, y, yaw, dir)
        MPC horizon(T) 만큼, start_idx부터 T개를 참조
        """
        T= self.mpc.T
        xref= np.zeros((3, T+1))
        n= len(self.parking_path)

        for i in range(T+1):
            idx= min(start_idx + i, n-1)
            xref[0, i]= self.parking_path[idx][0]
            xref[1, i]= self.parking_path[idx][1]
            xref[2, i]= self.parking_path[idx][2]

        return xref

    # def build_parking_xref(self, near_i, st: State):
    #     """
    #     주차 경로(parking_path)는 (x, y, yaw, dir)로 구성.
    #     MPC Horizon(T) 만큼, near_i부터 T개를 끊어서 xref 구성
    #     """
    #     T= self.mpc.T
    #     xref= np.zeros((3, T+1))
    #     n= len(self.parking_path)
    #     for i in range(T+1):
    #         idx= min(near_i + i, n-1)
    #         xref[0, i] = self.parking_path[idx][0]
    #         xref[1, i] = self.parking_path[idx][1]
    #         xref[2, i] = self.parking_path[idx][2]
    #     return xref

    def visualize_xref(self, xref):
        path_msg = GlobalPath()
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

    def get_nearest_idx_parking(self, x, y, parking_path):
        # parking_path = [(x, y, yaw, dir), ...]
        # 단순히 전체 탐색
        return min(range(len(parking_path)), key=lambda i: (x - parking_path[i][0])**2 + (y - parking_path[i][1])**2)
   
    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = MPCNode()
    node.run()