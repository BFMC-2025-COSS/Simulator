import sys
import pathlib
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))


class Path:
    """
    경로 데이터를 저장하는 컨테이너 클래스
    """
    def __init__(self):
        self.lengths = []        # 구간 길이 (음수면 후진)
        self.ctypes = []         # 구간 타입 ("S": 직진, "L": 좌회전, "R": 우회전)
        self.L = 0.0             # 전체 경로 길이
        self.x = []              # x 좌표
        self.y = []              # y 좌표
        self.yaw = []            # yaw (rad)
        self.directions = []     # 이동 방향 (1: 전진, -1: 후진)


class ReedsSheppPathPlanner:
    """
    원본 Reeds-Shepp 경로 생성 코드를 모두 포함한 클래스.
    """

    def __init__(self, max_curvature=0.1, step_size=0.05, show_ani=True):
        self.max_curvature = max_curvature
        self.step_size = step_size
        self.show_animation = show_ani

    def rot_mat_2d(self, angle):
        """
        2D 회전 행렬 생성
        """
        return Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]

    def angle_mod(self, x, zero_2_2pi=False, degree=False):
        """
        각도를 [-π, π) 또는 [0, 2π) 범위로 변환
        """
        x = np.asarray(x).flatten()
        if degree:
            x = np.deg2rad(x)

        if zero_2_2pi:
            mod_angle = x % (2 * np.pi)
        else:
            mod_angle = (x + np.pi) % (2 * np.pi) - np.pi

        if degree:
            mod_angle = np.rad2deg(mod_angle)
        return mod_angle

    def pi_2_pi(self, x):
        """
        각도를 [-π, π) 범위로 변환
        """
        return self.angle_mod(x)

    def mod2pi(self, x):
        """
        fmod 비슷한 각도 변환
        """
        v = np.mod(x, np.copysign(2.0 * math.pi, x))
        if v < -math.pi:
            v += 2.0 * math.pi
        elif v > math.pi:
            v -= 2.0 * math.pi
        return v

    def plot_arrow(self, x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
        """
        화살표 형태로 위치/방향 표시
        """
        if isinstance(x, list):
            for (ix, iy, iyaw) in zip(x, y, yaw):
                self.plot_arrow(ix, iy, iyaw, length, width, fc, ec)
        else:
            plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                      fc=fc, ec=ec, head_width=width, head_length=width)
            plt.plot(x, y)

    def polar(self, x, y):
        """
        직교좌표 (x, y)를 극좌표 (r, theta)로 변환
        """
        r = math.hypot(x, y)
        theta = math.atan2(y, x)
        return r, theta

    def timeflip(self, travel_distances):
        """
        거리 리스트에 음수를 곱하여 뒤집기
        """
        return [-x for x in travel_distances]

    def reflect(self, steering_directions):
        """
        'L' <-> 'R' 반전
        """
        def switch_dir(dirn):
            if dirn == 'L':
                return 'R'
            elif dirn == 'R':
                return 'L'
            else:
                return 'S'
        return [switch_dir(d) for d in steering_directions]

    def set_path(self, paths, lengths, ctypes, step_size):
        """
        Path 객체 생성 후, paths 리스트에 추가
        """
        path = Path()
        path.ctypes = ctypes
        path.lengths = lengths
        path.L = sum(np.abs(lengths))

        # 기존에 동일한 path가 있는지 확인
        for i_path in paths:
            type_is_same = (i_path.ctypes == path.ctypes)
            length_is_close = (sum(np.abs(i_path.lengths)) - path.L) <= step_size
            if type_is_same and length_is_close:
                return paths  # 이미 존재하면 추가하지 않음

        # 길이가 너무 짧으면 추가하지 않음
        if path.L <= step_size:
            return paths

        paths.append(path)
        return paths

    # ----------------
    # 아래부터는 원본 코드의 각 경로 계산 함수
    # ----------------
    def left_straight_left(self, x, y, phi):
        u, t = self.polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
        if 0.0 <= t <= math.pi:
            v = self.mod2pi(phi - t)
            if 0.0 <= v <= math.pi:
                return True, [t, u, v], ['L', 'S', 'L']
        return False, [], []

    def left_straight_right(self, x, y, phi):
        u1, t1 = self.polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
        u1 = u1 ** 2
        if u1 >= 4.0:
            u = math.sqrt(u1 - 4.0)
            theta = math.atan2(2.0, u)
            t = self.mod2pi(t1 + theta)
            v = self.mod2pi(t - phi)
            if (t >= 0.0) and (v >= 0.0):
                return True, [t, u, v], ['L', 'S', 'R']
        return False, [], []

    def left_x_right_x_left(self, x, y, phi):
        zeta = x - math.sin(phi)
        eeta = y - 1 + math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 <= 4.0:
            A = math.acos(0.25 * u1)
            t = self.mod2pi(A + theta + math.pi/2)
            u = self.mod2pi(math.pi - 2 * A)
            v = self.mod2pi(phi - t - u)
            return True, [t, -u, v], ['L', 'R', 'L']
        return False, [], []

    def left_x_right_left(self, x, y, phi):
        zeta = x - math.sin(phi)
        eeta = y - 1 + math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 <= 4.0:
            A = math.acos(0.25 * u1)
            t = self.mod2pi(A + theta + math.pi/2)
            u = self.mod2pi(math.pi - 2*A)
            v = self.mod2pi(-phi + t + u)
            return True, [t, -u, -v], ['L', 'R', 'L']
        return False, [], []

    def left_right_x_left(self, x, y, phi):
        zeta = x - math.sin(phi)
        eeta = y - 1 + math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 <= 4.0:
            u = math.acos(1 - u1**2 * 0.125)
            A = math.asin(2 * math.sin(u) / u1)
            t = self.mod2pi(-A + theta + math.pi/2)
            v = self.mod2pi(t - u - phi)
            return True, [t, u, -v], ['L', 'R', 'L']
        return False, [], []

    def left_right_x_left_right(self, x, y, phi):
        zeta = x + math.sin(phi)
        eeta = y - 1 - math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 <= 2:
            A = math.acos((u1 + 2) * 0.25)
            t = self.mod2pi(theta + A + math.pi/2)
            u = self.mod2pi(A)
            v = self.mod2pi(phi - t + 2*u)
            if (t >= 0) and (u >= 0) and (v >= 0):
                return True, [t, u, -u, -v], ['L', 'R', 'L', 'R']
        return False, [], []

    def left_x_right_left_x_right(self, x, y, phi):
        zeta = x + math.sin(phi)
        eeta = y - 1 - math.cos(phi)
        u1, theta = self.polar(zeta, eeta)
        u2 = (20 - u1**2) / 16

        if 0 <= u2 <= 1:
            u = math.acos(u2)
            A = math.asin(2 * math.sin(u) / u1)
            t = self.mod2pi(theta + A + math.pi/2)
            v = self.mod2pi(t - phi)
            if (t >= 0) and (v >= 0):
                return True, [t, -u, -u, v], ['L', 'R', 'L', 'R']
        return False, [], []

    def left_x_right90_straight_left(self, x, y, phi):
        zeta = x - math.sin(phi)
        eeta = y - 1 + math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 >= 2.0:
            u = math.sqrt(u1**2 - 4) - 2
            A = math.atan2(2, math.sqrt(u1**2 - 4))
            t = self.mod2pi(theta + A + math.pi/2)
            v = self.mod2pi(t - phi + math.pi/2)
            if (t >= 0) and (v >= 0):
                return True, [t, -math.pi/2, -u, -v], ['L', 'R', 'S', 'L']
        return False, [], []

    def left_straight_right90_x_left(self, x, y, phi):
        zeta = x - math.sin(phi)
        eeta = y - 1 + math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 >= 2.0:
            u = math.sqrt(u1**2 - 4) - 2
            A = math.atan2(math.sqrt(u1**2 - 4), 2)
            t = self.mod2pi(theta - A + math.pi/2)
            v = self.mod2pi(t - phi - math.pi/2)
            if (t >= 0) and (v >= 0):
                return True, [t, u, math.pi/2, -v], ['L', 'S', 'R', 'L']
        return False, [], []

    def left_x_right90_straight_right(self, x, y, phi):
        zeta = x + math.sin(phi)
        eeta = y - 1 - math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 >= 2.0:
            t = self.mod2pi(theta + math.pi/2)
            u = u1 - 2
            v = self.mod2pi(phi - t - math.pi/2)
            if (t >= 0) and (v >= 0):
                return True, [t, -math.pi/2, -u, -v], ['L', 'R', 'S', 'R']
        return False, [], []

    def left_straight_left90_x_right(self, x, y, phi):
        zeta = x + math.sin(phi)
        eeta = y - 1 - math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 >= 2.0:
            t = self.mod2pi(theta)
            u = u1 - 2
            v = self.mod2pi(phi - t - math.pi/2)
            if (t >= 0) and (v >= 0):
                return True, [t, u, math.pi/2, -v], ['L', 'S', 'L', 'R']
        return False, [], []

    def left_x_right90_straight_left90_x_right(self, x, y, phi):
        zeta = x + math.sin(phi)
        eeta = y - 1 - math.cos(phi)
        u1, theta = self.polar(zeta, eeta)

        if u1 >= 4.0:
            u = math.sqrt(u1**2 - 4) - 4
            A = math.atan2(2, math.sqrt(u1**2 - 4))
            t = self.mod2pi(theta + A + math.pi/2)
            v = self.mod2pi(t - phi)
            if (t >= 0) and (v >= 0):
                return True, [t, -math.pi/2, -u, -math.pi/2, v], ['L', 'R', 'S', 'L', 'R']
        return False, [], []

    def interpolate(self, dist, length, mode, max_curvature, origin_x, origin_y, origin_yaw):
        """
        단일 거리에 대해, mode에 따른 위치/방향 보간
        (원본 코드 그대로)
        """
        if mode == "S":
            x = origin_x + dist / max_curvature * math.cos(origin_yaw)
            y = origin_y + dist / max_curvature * math.sin(origin_yaw)
            yaw = origin_yaw
        else:  # curve
            ldx = math.sin(dist) / max_curvature
            ldy = 0.0
            yaw = None
            if mode == "L":
                ldy = (1.0 - math.cos(dist)) / max_curvature
                yaw = origin_yaw + dist
            elif mode == "R":
                ldy = (1.0 - math.cos(dist)) / -max_curvature
                yaw = origin_yaw - dist
            gdx = math.cos(-origin_yaw) * ldx + math.sin(-origin_yaw) * ldy
            gdy = -math.sin(-origin_yaw) * ldx + math.cos(-origin_yaw) * ldy
            x = origin_x + gdx
            y = origin_y + gdy

        return x, y, yaw, 1 if length > 0.0 else -1

    def calc_interpolate_dists_list(self, lengths, step_size):
        """
        길이 배열을 step_size 단위로 보간
        """
        interpolate_dists_list = []
        for length in lengths:
            d_dist = step_size if length >= 0.0 else -step_size
            interp_dists = np.arange(0.0, length, d_dist)
            interp_dists = np.append(interp_dists, length)
            interpolate_dists_list.append(interp_dists)
        return interpolate_dists_list

    def generate_local_course(self, lengths, modes, max_curvature, step_size):
        """
        local course 생성
        """
        interpolate_dists_list = self.calc_interpolate_dists_list(lengths, step_size * max_curvature)
        origin_x, origin_y, origin_yaw = 0.0, 0.0, 0.0

        xs, ys, yaws, directions = [], [], [], []
        for (interp_dists, mode, length) in zip(interpolate_dists_list, modes, lengths):
            for dist in interp_dists:
                x, y, yaw, direction = self.interpolate(dist, length, mode,
                                                        max_curvature,
                                                        origin_x, origin_y,
                                                        origin_yaw)
                xs.append(x)
                ys.append(y)
                yaws.append(yaw)
                directions.append(direction)
            origin_x = xs[-1]
            origin_y = ys[-1]
            origin_yaw = yaws[-1]
        return xs, ys, yaws, directions

    def generate_path(self, q0, q1, max_curvature, step_size):
        """
        원본: generate_path(q0, q1, max_curvature, step_size)
        """
        dx = q1[0] - q0[0]
        dy = q1[1] - q0[1]
        dth = q1[2] - q0[2]
        c = math.cos(q0[2])
        s = math.sin(q0[2])
        x = (c * dx + s * dy) * max_curvature
        y = (-s * dx + c * dy) * max_curvature
        step_size *= max_curvature

        paths = []
        path_functions = [
            self.left_straight_left, self.left_straight_right,
            self.left_x_right_x_left, self.left_x_right_left, self.left_right_x_left,
            self.left_right_x_left_right, self.left_x_right_left_x_right,
            self.left_x_right90_straight_left, self.left_x_right90_straight_right,
            self.left_straight_right90_x_left, self.left_straight_left90_x_right,
            self.left_x_right90_straight_left90_x_right
        ]

        for path_func in path_functions:
            flag, travel_distances, steering_dirns = path_func(x, y, dth)
            if flag:
                # step_size가 너무 큰 경우 체크
                for distance in travel_distances:
                    if (0.1*sum([abs(d) for d in travel_distances])
                            < abs(distance) < step_size):
                        print("Step size too large for Reeds-Shepp paths.")
                        return []
                paths = self.set_path(paths, travel_distances,
                                      steering_dirns, step_size)

            # 다음 세 케이스도 동일하게 처리
            flag, travel_distances, steering_dirns = path_func(-x, y, -dth)
            if flag:
                for distance in travel_distances:
                    if (0.1*sum([abs(d) for d in travel_distances])
                            < abs(distance) < step_size):
                        print("Step size too large for Reeds-Shepp paths.")
                        return []
                travel_distances = self.timeflip(travel_distances)
                paths = self.set_path(paths, travel_distances,
                                      steering_dirns, step_size)

            flag, travel_distances, steering_dirns = path_func(x, -y, -dth)
            if flag:
                for distance in travel_distances:
                    if (0.1*sum([abs(d) for d in travel_distances])
                            < abs(distance) < step_size):
                        print("Step size too large for Reeds-Shepp paths.")
                        return []
                steering_dirns = self.reflect(steering_dirns)
                paths = self.set_path(paths, travel_distances,
                                      steering_dirns, step_size)

            flag, travel_distances, steering_dirns = path_func(-x, -y, dth)
            if flag:
                for distance in travel_distances:
                    if (0.1*sum([abs(d) for d in travel_distances])
                            < abs(distance) < step_size):
                        print("Step size too large for Reeds-Shepp paths.")
                        return []
                travel_distances = self.timeflip(travel_distances)
                steering_dirns = self.reflect(steering_dirns)
                paths = self.set_path(paths, travel_distances,
                                      steering_dirns, step_size)

        return paths

    def calc_paths(self, sx, sy, syaw, gx, gy, gyaw, maxc, step_size):
        """
        calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size)
        """
        q0 = [sx, sy, syaw]
        q1 = [gx, gy, gyaw]

        paths = self.generate_path(q0, q1, maxc, step_size)
        for path in paths:
            xs, ys, yaws, directions = self.generate_local_course(
                path.lengths, path.ctypes, maxc, step_size
            )

            # 전역 좌표 변환
            path.x = [
                math.cos(-q0[2]) * ix + math.sin(-q0[2]) * iy + q0[0]
                for (ix, iy) in zip(xs, ys)
            ]
            path.y = [
                -math.sin(-q0[2]) * ix + math.cos(-q0[2]) * iy + q0[1]
                for (ix, iy) in zip(xs, ys)
            ]
            path.yaw = [self.pi_2_pi(yaw + q0[2]) for yaw in yaws]
            path.directions = directions
            path.lengths = [length / maxc for length in path.lengths]
            path.L = path.L / maxc
        return paths

    def reeds_shepp_path_planning(self, sx, sy, syaw, gx, gy, gyaw, maxc, step_size=0.2):
        """
        Reeds-Shepp 경로 생성 후, 최적 경로를 반환
        """
        paths = self.calc_paths(sx, sy, syaw, gx, gy, gyaw, maxc, step_size)
        if not paths:
            return None, None, None, None, None  # could not generate any path

        # 최소 거리 경로 찾기
        best_path_index = paths.index(min(paths, key=lambda p: abs(p.L)))
        b_path = paths[best_path_index]

        return b_path.x, b_path.y, b_path.yaw, b_path.directions, b_path.lengths

    def main(self):
        """
        원본 main() 함수 로직 그대로
        """
        print("Reeds Shepp path planner sample start!!")

        start_x = 2.0
        start_y = -2.0
        start_yaw = np.deg2rad(0.0)

        end_x = 6.0
        end_y = 2.0
        end_yaw = np.deg2rad(0.0)

        curvature = 0.1
        step_size = 0.05

        xs, ys, yaws, modes, lengths = self.reeds_shepp_path_planning(
            start_x, start_y, start_yaw,
            end_x, end_y, end_yaw,
            curvature, step_size
        )

        if not xs:
            assert False, "No path"

        if self.show_animation:
            plt.cla()
            plt.plot(xs, ys, label="final course " + str(modes))
            print(f"{lengths=}")

            self.plot_arrow(start_x, start_y, start_yaw)
            self.plot_arrow(end_x, end_y, end_yaw)

            plt.legend()
            plt.grid(True)
            plt.axis("equal")
            plt.show()


if __name__ == '__main__':
    planner = ReedsSheppPathPlanner(max_curvature=0.1, step_size=0.05, show_ani=True)
    planner.main()
