import numpy as np
import time
import rosshow.termgraphics as termgraphics
from rosshow.viewers.generic.Space2DViewer import Space2DViewer


class NavigationViewer(Space2DViewer):
    """
    综合导航可视化器，支持同时显示：
    - 全局规划路径 (nav_msgs/Path)
    - 局部规划路径 (nav_msgs/Path)
    - 当前位置和姿态 (nav_msgs/Odometry)
    - 目标点 (geometry_msgs/PoseStamped)
    - 已走过的轨迹
    - 激光雷达扫描 (sensor_msgs/LaserScan)
    """

    COMMAND_TYPE_POLYLINE = 2

    def __init__(self, canvas, title="Navigation"):
        self.global_path = None
        self.local_path = None
        self.odometry = None
        self.goal = None
        self.laserscan = None
        self.trajectory = np.empty((512, 2), dtype=np.float64)
        self.trajectory[:] = np.nan
        self.trajectory_i = 0
        self.init_centered = False

        Space2DViewer.__init__(self, canvas, msg_decoder=self._msg_decoder, title=title)

    def update_global_path(self, msg):
        self.global_path = msg

    def update_local_path(self, msg):
        self.local_path = msg

    def update_odometry(self, msg):
        self.odometry = msg

        if self.odometry:
            x = self.odometry.pose.pose.position.x
            y = self.odometry.pose.pose.position.y
            self.trajectory[self.trajectory_i] = [x, y]
            self.trajectory_i = (self.trajectory_i + 1) & 0x1FF

            if not self.init_centered:
                self.offset_x = x
                self.offset_y = y
                self.target_offset_x = x
                self.target_offset_y = y
                self.init_centered = True

    def update_goal(self, msg):
        self.goal = msg

    def update_laserscan(self, msg):
        self.laserscan = msg

    def _msg_decoder(self, msg):
        draw_commands = [
            (Space2DViewer.COMMAND_TYPE_LINE, termgraphics.COLOR_RED, [(0., 0.), (1., 0.)]),
            (Space2DViewer.COMMAND_TYPE_LINE, termgraphics.COLOR_GREEN, [(0., 0.), (0., 1.)]),
        ]

        if self.global_path and len(self.global_path.poses) > 1:
            points = np.array(
                [[pose.pose.position.x, pose.pose.position.y] for pose in self.global_path.poses],
                dtype=np.float64
            )
            draw_commands.append((
                NavigationViewer.COMMAND_TYPE_POLYLINE,
                termgraphics.COLOR_YELLOW,
                points
            ))

        if self.local_path and len(self.local_path.poses) > 1:
            points = np.array(
                [[pose.pose.position.x, pose.pose.position.y] for pose in self.local_path.poses],
                dtype=np.float64
            )
            draw_commands.append((
                NavigationViewer.COMMAND_TYPE_POLYLINE,
                termgraphics.COLOR_GREEN,
                points
            ))

        draw_commands.append((
            Space2DViewer.COMMAND_TYPE_POINTS,
            termgraphics.COLOR_CYAN,
            self.trajectory
        ))

        if self.laserscan:
            angles = np.linspace(
                self.laserscan.angle_min,
                self.laserscan.angle_max,
                len(self.laserscan.ranges),
                dtype=np.float32
            )
            ranges = np.array(self.laserscan.ranges, dtype=np.float32)
            valid = np.isfinite(ranges) & (ranges > self.laserscan.range_min) & (ranges < self.laserscan.range_max)
            
            if self.odometry:
                robot_x = self.odometry.pose.pose.position.x
                robot_y = self.odometry.pose.pose.position.y
            else:
                robot_x, robot_y = 0, 0

            x_values = robot_x + ranges[valid] * np.cos(angles[valid])
            y_values = robot_y + ranges[valid] * np.sin(angles[valid])
            laser_points = np.vstack((x_values, y_values)).T
            draw_commands.append((
                Space2DViewer.COMMAND_TYPE_POINTS,
                termgraphics.COLOR_MAGENTA,
                laser_points
            ))

        if self.odometry:
            x = self.odometry.pose.pose.position.x
            y = self.odometry.pose.pose.position.y
            draw_commands.append((
                Space2DViewer.COMMAND_TYPE_POINTS,
                termgraphics.COLOR_WHITE,
                np.array([[x, y]])
            ))

            orient = self.odometry.pose.pose.orientation
            norm = np.sqrt(orient.x**2 + orient.y**2 + orient.z**2 + orient.w**2)
            if norm > 0:
                a, b, c, d = orient.x/norm, orient.y/norm, orient.z/norm, orient.w/norm
                yaw = np.arctan2(2*a*b + 2*c*d, 1 - 2*b*b - 2*c*c)
                draw_commands.append((
                    Space2DViewer.COMMAND_TYPE_LINE,
                    termgraphics.COLOR_WHITE,
                    [
                        (x, y),
                        (x + np.cos(yaw), y + np.sin(yaw))
                    ]
                ))

        if self.goal:
            goal_x = self.goal.pose.position.x
            goal_y = self.goal.pose.position.y
            size = 0.5
            goal_marker = np.array([
                [goal_x, goal_y + size],
                [goal_x + size, goal_y],
                [goal_x, goal_y - size],
                [goal_x - size, goal_y],
                [goal_x, goal_y + size],
            ])
            draw_commands.append((
                NavigationViewer.COMMAND_TYPE_POLYLINE,
                termgraphics.COLOR_RED,
                goal_marker
            ))

        return draw_commands

    def draw(self):
        if not self.odometry and not self.global_path:
            self.canvas.clear()
            self.canvas.set_color((127, 127, 127))
            self.canvas.text("Waiting for navigation data...", (0, self.canvas.shape[1] // 2))
            self.canvas.draw()
            return

        t = time.time()
        if t - self.last_update_shape_time > 0.25:
            self.canvas.update_shape()
            self.last_update_shape_time = t

        if self.scale != self.target_scale or \
           self.offset_x != self.target_offset_x or \
           self.offset_y != self.target_offset_y:
            animation_fraction = (time.time() - self.target_time) / 0.5
            if animation_fraction > 1.0:
                self.scale = self.target_scale
                self.offset_x = self.target_offset_x
                self.offset_y = self.target_offset_y
            else:
                self.scale = (1 - animation_fraction) * self.scale + animation_fraction * self.target_scale
                self.offset_x = (1 - animation_fraction) * self.offset_x + animation_fraction * self.target_offset_x
                self.offset_y = (1 - animation_fraction) * self.offset_y + animation_fraction * self.target_offset_y

        self.canvas.clear()
        self.canvas.set_color(termgraphics.COLOR_WHITE)

        w = self.canvas.shape[0]
        h = self.canvas.shape[1]
        xmax = self.scale
        ymax = self.scale * h / w

        commands = self._msg_decoder(None)
        for command_type, color, data in commands:
            self.canvas.set_color(color)
            if command_type == Space2DViewer.COMMAND_TYPE_POINTS:
                if len(data) == 0:
                    continue
                x = data[:, 0]
                y = data[:, 1]
                screen_is = (w * (x - self.offset_x + xmax) / (2 * xmax)).astype(np.uint16)
                screen_js = (h * (1 - (y - self.offset_y + ymax) / (2 * ymax))).astype(np.uint16)
                where_valid = ~np.isnan(screen_is) & ~np.isnan(screen_js) & \
                             (screen_is > 0) & (screen_js > 0) & \
                             (screen_is < w) & (screen_js < h)
                screen_is = screen_is[where_valid]
                screen_js = screen_js[where_valid]
                if len(screen_is) > 0:
                    screen_points = np.vstack((screen_is, screen_js)).T
                    self.canvas.points(screen_points)

            elif command_type == Space2DViewer.COMMAND_TYPE_LINE:
                screen_0_i = int(w * (data[0][0] - self.offset_x + xmax) / (2 * xmax))
                screen_0_j = int(h * (1 - (data[0][1] - self.offset_y + ymax) / (2 * ymax)))
                screen_1_i = int(w * (data[1][0] - self.offset_x + xmax) / (2 * xmax))
                screen_1_j = int(h * (1 - (data[1][1] - self.offset_y + ymax) / (2 * ymax)))
                self.canvas.line((screen_0_i, screen_0_j), (screen_1_i, screen_1_j))

            elif command_type == NavigationViewer.COMMAND_TYPE_POLYLINE:
                for i in range(len(data) - 1):
                    screen_0_i = int(w * (data[i][0] - self.offset_x + xmax) / (2 * xmax))
                    screen_0_j = int(h * (1 - (data[i][1] - self.offset_y + ymax) / (2 * ymax)))
                    screen_1_i = int(w * (data[i+1][0] - self.offset_x + xmax) / (2 * xmax))
                    screen_1_j = int(h * (1 - (data[i+1][1] - self.offset_y + ymax) / (2 * ymax)))
                    self.canvas.line((screen_0_i, screen_0_j), (screen_1_i, screen_1_j))

        self.canvas.set_color((0, 127, 255))
        self.canvas.text(self.title, (0, self.canvas.shape[1] - 4))

        self.canvas.set_color((127, 127, 127))
        status = []
        if self.global_path:
            status.append("G")
        if self.local_path:
            status.append("L")
        if self.odometry:
            status.append("O")
        if self.goal:
            status.append("G")
        if self.laserscan:
            status.append("S")
        status_text = " | ".join([
            "Global Path" if self.global_path else "",
            "Local Path" if self.local_path else "",
            "Odometry" if self.odometry else "",
            "Goal" if self.goal else "",
            "Scan" if self.laserscan else "",
        ]).strip(" | ")
        self.canvas.text("Status: {}".format(status_text), (int(self.canvas.shape[0] / 3), self.canvas.shape[1] - 4))

        self.canvas.draw()

    def update(self, msg):
        msg_type = type(msg).__module__ + "/" + type(msg).__name__
        if "nav_msgs.msg._Path.Path" in str(type(msg)) or msg_type.endswith("/Path"):
            if hasattr(self, '_last_path_time'):
                if self._last_path_time < getattr(msg.header.stamp, 'secs', 0):
                    self.global_path = msg
                else:
                    self.local_path = msg
            else:
                self.global_path = msg
            self._last_path_time = getattr(msg.header.stamp, 'secs', 0)
        elif "nav_msgs.msg._Odometry.Odometry" in str(type(msg)) or msg_type.endswith("/Odometry"):
            self.update_odometry(msg)
        elif "geometry_msgs.msg._PoseStamped.PoseStamped" in str(type(msg)) or msg_type.endswith("/PoseStamped"):
            self.update_goal(msg)
        elif "sensor_msgs.msg._LaserScan.LaserScan" in str(type(msg)) or msg_type.endswith("/LaserScan"):
            self.update_laserscan(msg)
        else:
            self.odometry = msg
            if msg is not None:
                self.trajectory[self.trajectory_i] = [0, 0]
                self.trajectory_i = (self.trajectory_i + 1) & 0x1FF
                if not self.init_centered:
                    self.init_centered = True
        self.msg = msg
