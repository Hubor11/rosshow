import math
import time
import numpy as np

import rosshow.termgraphics as termgraphics
from rosshow.viewers.generic.SubCanvas import SubCanvas


VIEWER_MAPPING = {
    "nav_msgs/Odometry": ("rosshow.viewers.nav_msgs.OdometryViewer", "OdometryViewer", {}),
    "nav_msgs/OccupancyGrid": ("rosshow.viewers.nav_msgs.OccupancyGridViewer", "OccupancyGridViewer", {}),
    "nav_msgs/Path": ("rosshow.viewers.nav_msgs.PathViewer", "PathViewer", {}),
    "std_msgs/Bool": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/Float32": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/Float64": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/Int8": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/Int16": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/Int32": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/Int64": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/UInt8": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/UInt16": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/UInt32": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "std_msgs/UInt64": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {}),
    "sensor_msgs/CompressedImage": ("rosshow.viewers.sensor_msgs.CompressedImageViewer", "CompressedImageViewer", {}),
    "sensor_msgs/FluidPressure": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "fluid_pressure"}),
    "sensor_msgs/RelativeHumidity": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "relative_humidity"}),
    "sensor_msgs/Illuminance": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "illuminance"}),
    "sensor_msgs/Image": ("rosshow.viewers.sensor_msgs.ImageViewer", "ImageViewer", {}),
    "sensor_msgs/Imu": ("rosshow.viewers.sensor_msgs.ImuViewer", "ImuViewer", {}),
    "sensor_msgs/LaserScan": ("rosshow.viewers.sensor_msgs.LaserScanViewer", "LaserScanViewer", {}),
    "sensor_msgs/NavSatFix": ("rosshow.viewers.sensor_msgs.NavSatFixViewer", "NavSatFixViewer", {}),
    "sensor_msgs/PointCloud2": ("rosshow.viewers.sensor_msgs.PointCloud2Viewer", "PointCloud2Viewer", {}),
    "sensor_msgs/Range": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "range"}),
    "sensor_msgs/Temperature": ("rosshow.viewers.generic.SinglePlotViewer", "SinglePlotViewer", {"data_field": "temperature"}),
    "geometry_msgs/Twist": ("rosshow.viewers.generic.MultiPlotViewer", "MultiPlotViewer", {"data_fields": ["linear.x", "linear.y", "linear.z", "angular.x", "angular.y", "angular.z"]}),
}


class MultiViewer(object):
    """
    Manages multiple viewers in a grid layout within a single terminal.
    Accepts a list of topic specs, each with 'topic' name and 'type' (ROS message type).
    Supports focus switching with Tab key.
    """

    def __init__(self, canvas, topic_specs, title="rosshow"):
        self.canvas = canvas
        self.topic_specs = topic_specs
        self.title = title
        self.viewers = []
        self.sub_canvases = []
        self.focus_index = 0
        self.last_update_shape_time = 0
        self._layout_dirty = True

        self._create_viewers()
        self._compute_layout()

    def _create_viewers(self):
        self.viewers = []
        for spec in self.topic_specs:
            topic_type = spec.get("type", spec.get("ros_type", ""))
            topic_name = spec.get("topic", "")
            display_name = spec.get("name", topic_name)

            if topic_type not in VIEWER_MAPPING:
                continue

            module_name, class_name, viewer_kwargs = VIEWER_MAPPING[topic_type]
            viewer_class = getattr(__import__(module_name, fromlist=(class_name)), class_name)

            self.viewers.append({
                "viewer": None,
                "viewer_class": viewer_class,
                "viewer_kwargs": viewer_kwargs,
                "topic": topic_name,
                "type": topic_type,
                "name": display_name,
                "sub_canvas": None,
            })

    def _compute_layout(self):
        n = len(self.viewers)
        if n == 0:
            return

        canvas_w, canvas_h = self.canvas.shape
        title_row_height = 4

        usable_h = canvas_h - title_row_height
        usable_w = canvas_w

        cols = max(1, int(math.ceil(math.sqrt(n))))
        rows = max(1, int(math.ceil(n / cols)))

        cell_w = usable_w // cols
        cell_h = usable_h // rows

        for idx, vinfo in enumerate(self.viewers):
            row = idx // cols
            col = idx % cols

            x_offset = col * cell_w
            y_offset = title_row_height + row * cell_h
            width = cell_w - 2
            height = cell_h - 2

            x_offset += 1
            y_offset += 1

            sub_canvas = SubCanvas(
                self.canvas,
                x_offset=x_offset,
                y_offset=y_offset,
                width=width,
                height=height,
            )

            vinfo["sub_canvas"] = sub_canvas
            vinfo["grid_pos"] = (row, col, x_offset, y_offset, width, height)

            if vinfo["viewer"] is None:
                vinfo["viewer"] = vinfo["viewer_class"](
                    sub_canvas,
                    title=vinfo["name"],
                    **vinfo["viewer_kwargs"]
                )
            else:
                vinfo["viewer"].g = sub_canvas
                if hasattr(vinfo["viewer"], "last_update_shape_time"):
                    vinfo["viewer"].last_update_shape_time = 0

        self._layout_dirty = False
        self._cols = cols
        self._rows = rows

    def update(self, msg, topic_index=0):
        if 0 <= topic_index < len(self.viewers):
            viewer = self.viewers[topic_index]["viewer"]
            if viewer and hasattr(viewer, "update"):
                viewer.update(msg)

    def make_update_callback(self, topic_index):
        def callback(msg):
            self.update(msg, topic_index)
        return callback

    def keypress(self, c):
        if c == "\t":
            self.focus_index = (self.focus_index + 1) % len(self.viewers)
            return
        if c == "~" or c == "}":
            self.focus_index = (self.focus_index - 1) % len(self.viewers)
            return

        if 0 <= self.focus_index < len(self.viewers):
            viewer = self.viewers[self.focus_index]["viewer"]
            if viewer and hasattr(viewer, "keypress"):
                viewer.keypress(c)

    def draw(self):
        t = time.time()

        if t - self.last_update_shape_time > 0.25:
            if self.canvas.update_shape():
                self._layout_dirty = True
            self.last_update_shape_time = t

        if self._layout_dirty:
            self._compute_layout()

        self.canvas.clear()

        self.canvas.set_color((0, 127, 255))
        title_text = "{}  [{} panels, Tab to switch focus]".format(self.title, len(self.viewers))
        self.canvas.text(title_text, (0, 0))

        for idx, vinfo in enumerate(self.viewers):
            row, col, x_off, y_off, w, h = vinfo["grid_pos"]

            is_focused = (idx == self.focus_index)
            border_color = (0, 255, 127) if is_focused else (63, 63, 63)
            self.canvas.set_color(border_color)

            self.canvas.rect((x_off - 1, y_off - 1), (x_off + w, y_off + h))

            label = "[{}] {}".format(idx + 1, vinfo["name"])
            if len(label) > w // 2 - 2:
                label = label[:max(1, w // 2 - 5)] + "..."
            self.canvas.set_color(border_color)
            self.canvas.text(label, (x_off, y_off - 3))

            viewer = vinfo["viewer"]
            if viewer and hasattr(viewer, "draw"):
                viewer.draw()

        self.canvas.set_color((127, 127, 127))
        help_text = "Tab: next panel  Shift+Tab: prev panel  1-9: jump to panel  q/Ctrl+C: quit"
        self.canvas.text(help_text, (0, self.canvas.shape[1] - 4))

        self.canvas.draw()

    def get_subscribers_config(self):
        configs = []
        for idx, vinfo in enumerate(self.viewers):
            configs.append({
                "topic": vinfo["topic"],
                "type": vinfo["type"],
                "callback": self.make_update_callback(idx),
                "index": idx,
            })
        return configs
