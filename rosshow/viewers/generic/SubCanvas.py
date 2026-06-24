import numpy as np
import rosshow.termgraphics as termgraphics


class SubCanvas(object):
    """
    A sub-region of a parent TermGraphics canvas. All coordinates are relative
    to the sub-canvas's top-left corner, and all drawing is clipped to the
    sub-canvas's bounds. Existing viewer classes can use a SubCanvas in place
    of a full TermGraphics without any code changes.
    """

    def __init__(self, parent, x_offset, y_offset, width, height):
        self.parent = parent
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.width = width
        self.height = height

        self.shape = (self.width, self.height)
        self.term_shape = (self.width // 2, self.height // 4)
        self.current_color = np.array([255, 255, 255], dtype=np.uint8)
        self.mode = parent.mode
        self.color_support = parent.color_support

    def _clip_points(self, points):
        if len(points) == 0:
            return points
        pts = np.array(points, dtype=np.int32) if not isinstance(points, np.ndarray) else points.astype(np.int32)
        pts[:, 0] += self.x_offset
        pts[:, 1] += self.y_offset
        valid = (pts[:, 0] >= self.x_offset) & (pts[:, 0] < self.x_offset + self.width) & \
                (pts[:, 1] >= self.y_offset) & (pts[:, 1] < self.y_offset + self.height)
        return pts[valid]

    def set_color(self, color):
        self.current_color = color
        self.parent.set_color(color)

    def clear(self):
        for j in range(self.y_offset // 4, (self.y_offset + self.height) // 4):
            for i in range(self.x_offset // 2, (self.x_offset + self.width) // 2):
                if 0 <= j < self.parent.buffer.shape[0] and 0 <= i < self.parent.buffer.shape[1]:
                    self.parent.buffer[j, i] = 0x2800
                    self.parent.colors[j, i, :] = 0

    def update_shape(self):
        return False

    def point(self, point, clear_block=False):
        px = point[0] + self.x_offset
        py = point[1] + self.y_offset
        if 0 <= px < self.x_offset + self.width and 0 <= py < self.y_offset + self.height:
            self.parent.point((px, py), clear_block=clear_block)

    def points(self, points, colors=None, clear_block=False):
        if len(points) == 0:
            return
        pts = np.array(points, dtype=np.int32) if not isinstance(points, np.ndarray) else points.astype(np.int32)
        pts[:, 0] += self.x_offset
        pts[:, 1] += self.y_offset

        valid = (pts[:, 0] >= self.x_offset) & (pts[:, 0] < self.x_offset + self.width) & \
                (pts[:, 1] >= self.y_offset) & (pts[:, 1] < self.y_offset + self.height)

        valid_pts = pts[valid]
        valid_colors = colors[valid] if colors is not None else None

        if len(valid_pts) > 0:
            self.parent.points(valid_pts, colors=valid_colors, clear_block=clear_block)

    def line(self, point0, point1):
        p0 = (point0[0] + self.x_offset, point0[1] + self.y_offset)
        p1 = (point1[0] + self.x_offset, point1[1] + self.y_offset)
        self.parent.line(p0, p1)

    def poly(self, points):
        translated = [(p[0] + self.x_offset, p[1] + self.y_offset) for p in points]
        self.parent.poly(translated)

    def rect(self, point0, point1):
        p0 = (point0[0] + self.x_offset, point0[1] + self.y_offset)
        p1 = (point1[0] + self.x_offset, point1[1] + self.y_offset)
        self.parent.rect(p0, p1)

    def text(self, text, point):
        tx = point[0] + self.x_offset
        ty = point[1] + self.y_offset
        if ty // 4 < self.parent.buffer.shape[0] and tx // 2 < self.parent.buffer.shape[1]:
            max_len = (self.x_offset + self.width - tx) // 2
            if max_len > 0:
                clipped_text = text[:max_len]
                self.parent.text(clipped_text, (tx, ty))

    def image(self, data, width, height, point, image_type=termgraphics.IMAGE_MONOCHROME, clear_block=False):
        px = point[0] + self.x_offset
        py = point[1] + self.y_offset
        self.parent.image(data, width, height, (px, py), image_type=image_type, clear_block=clear_block)

    def draw(self):
        pass
