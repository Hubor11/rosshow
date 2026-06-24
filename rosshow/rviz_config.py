import re
import os


RVIZ_CLASS_TO_ROS_TYPE = {
    "rviz/PointCloud2": "sensor_msgs/PointCloud2",
    "rviz/LaserScan": "sensor_msgs/LaserScan",
    "rviz/Image": "sensor_msgs/Image",
    "rviz/CompressedImage": "sensor_msgs/CompressedImage",
    "rviz/Map": "nav_msgs/OccupancyGrid",
    "rviz/Path": "nav_msgs/Path",
    "rviz/Odometry": "nav_msgs/Odometry",
    "rviz/Imu": "sensor_msgs/Imu",
    "rviz/NavSatFix": "sensor_msgs/NavSatFix",
    "rviz/Temperature": "sensor_msgs/Temperature",
    "rviz/FluidPressure": "sensor_msgs/FluidPressure",
    "rviz/RelativeHumidity": "sensor_msgs/RelativeHumidity",
    "rviz/Illuminance": "sensor_msgs/Illuminance",
    "rviz/Range": "sensor_msgs/Range",
    "rviz/Bool": "std_msgs/Bool",
    "rviz/Int32": "std_msgs/Int32",
    "rviz/Float32": "std_msgs/Float32",
    "rviz/Float64": "std_msgs/Float64",
    "rviz/Twist": "geometry_msgs/Twist",
}


def _parse_rviz_value(raw):
    raw = raw.strip()
    if raw.startswith('"') and raw.endswith('"'):
        return raw[1:-1]
    if raw.lower() == "true":
        return True
    if raw.lower() == "false":
        return False
    try:
        return int(raw)
    except ValueError:
        pass
    try:
        return float(raw)
    except ValueError:
        pass
    return raw


def _find_displays_list(node):
    """Recursively search for a 'Displays' list in the parsed config tree."""
    if isinstance(node, dict):
        if "Displays" in node and isinstance(node["Displays"], list):
            return node["Displays"]
        for key in node:
            result = _find_displays_list(node[key])
            if result is not None:
                return result
    elif isinstance(node, list):
        for item in node:
            result = _find_displays_list(item)
            if result is not None:
                return result
    return None


def parse_rviz_config(filepath):
    """
    Parse an rviz config file (.rviz) and extract display configurations.
    Returns a list of display dicts, each with at least 'class', 'name', 'topic' (if applicable).
    """
    if not os.path.isfile(filepath):
        raise FileNotFoundError("rviz config file not found: {}".format(filepath))

    with open(filepath, "r") as f:
        lines = f.readlines()

    root = _parse_rviz_yamlish(lines)
    raw_displays = _find_displays_list(root)

    display_list = []
    if isinstance(raw_displays, list):
        for item in raw_displays:
            if isinstance(item, dict) and "Class" in item:
                display_list.append(item)

    topic_key_candidates = [
        "Topic", "topic", "Topic Name", "Image Topic",
        "Topic Name:", "Topic Name "
    ]

    result = []
    for d in display_list:
        rviz_class = d.get("Class", "")
        ros_type = RVIZ_CLASS_TO_ROS_TYPE.get(rviz_class, None)

        topic = None
        for key in topic_key_candidates:
            if key in d and isinstance(d[key], str) and d[key]:
                topic = d[key]
                break
        if topic is None and "Topic" in d and isinstance(d["Topic"], dict):
            topic = d["Topic"].get("Value", None) or d["Topic"].get("topic", None)

        name = d.get("Name", rviz_class.split("/")[-1])
        enabled = d.get("Enabled", True)
        if isinstance(enabled, str):
            enabled = enabled.lower() != "false"

        result.append({
            "rviz_class": rviz_class,
            "ros_type": ros_type,
            "topic": topic,
            "name": name,
            "enabled": enabled,
            "raw": d,
        })

    return result


def _parse_rviz_yamlish(lines):
    """
    Minimal parser for rviz's OGRE-config-style format (similar to YAML but not strict).
    Handles nested dicts, lists of dicts, and simple key-value pairs.
    """
    lines = [ln.rstrip("\n") for ln in lines if ln.strip() and not ln.strip().startswith("#")]

    def get_indent(line):
        return len(line) - len(line.lstrip(" "))

    def parse_block(start_idx, base_indent):
        result = None
        i = start_idx
        list_items = []
        dict_items = {}
        is_list = None

        while i < len(lines):
            line = lines[i]
            indent = get_indent(line)

            if indent < base_indent:
                break
            if indent > base_indent:
                i += 1
                continue

            stripped = line.strip()

            if stripped.startswith("- "):
                if is_list is None:
                    is_list = True
                value_str = stripped[2:]
                if ":" in value_str:
                    sub_dict, i = _parse_list_item_dict(lines, i, indent)
                    list_items.append(sub_dict)
                else:
                    list_items.append(_parse_rviz_value(value_str))
                    i += 1
            elif ":" in stripped:
                if is_list is None:
                    is_list = False
                colon_idx = stripped.index(":")
                key = stripped[:colon_idx].strip()
                value_part = stripped[colon_idx + 1:].strip()

                if value_part:
                    dict_items[key] = _parse_rviz_value(value_part)
                    i += 1
                else:
                    next_i = i + 1
                    if next_i < len(lines):
                        next_indent = get_indent(lines[next_i])
                        if next_indent > indent:
                            sub_value, i = parse_block(next_i, next_indent)
                            dict_items[key] = sub_value
                        else:
                            dict_items[key] = None
                            i += 1
                    else:
                        dict_items[key] = None
                        i += 1
            else:
                i += 1

        if is_list is True:
            return list_items, i
        elif is_list is False:
            return dict_items, i
        else:
            return dict_items, i

    def _parse_list_item_dict(lines, start_idx, base_indent):
        """Parse a dict that starts with '- key: value' on the first line."""
        first_line = lines[start_idx].strip()
        first_value = first_line[2:]
        colon_idx = first_value.index(":")
        first_key = first_value[:colon_idx].strip()
        first_val = first_value[colon_idx + 1:].strip()

        result = {}
        if first_val:
            result[first_key] = _parse_rviz_value(first_val)
        else:
            result[first_key] = None

        i = start_idx + 1
        child_indent = base_indent + 2

        while i < len(lines):
            line = lines[i]
            indent = get_indent(line)
            if indent < child_indent:
                break
            if indent == child_indent and line.strip().startswith("- "):
                break
            if indent == child_indent and ":" in line.strip():
                stripped = line.strip()
                colon_idx2 = stripped.index(":")
                key = stripped[:colon_idx2].strip()
                val_part = stripped[colon_idx2 + 1:].strip()
                if val_part:
                    result[key] = _parse_rviz_value(val_part)
                    i += 1
                else:
                    next_i = i + 1
                    if next_i < len(lines) and get_indent(lines[next_i]) > indent:
                        sub_val, i = parse_block(next_i, get_indent(lines[next_i]))
                        result[key] = sub_val
                    else:
                        result[key] = None
                        i += 1
            else:
                i += 1

        return result, i

    if not lines:
        return {}

    base_indent = get_indent(lines[0])
    result, _ = parse_block(0, base_indent)
    return result


def get_supported_displays(displays):
    """Filter the parsed displays to only those rosshow can visualize."""
    supported = []
    for d in displays:
        if d["ros_type"] is not None and d["topic"] and d["enabled"]:
            supported.append(d)
    return supported


def generate_tmux_script(displays, output_path=None, rosshow_cmd="rosshow", extra_args=""):
    """
    Generate a tmux split-window script that runs rosshow for each display.
    Returns the script as a string; optionally writes to output_path.
    """
    supported = get_supported_displays(displays)
    if not supported:
        return "# No supported displays found in rviz config.\n"

    n = len(supported)
    lines = []
    lines.append("#!/bin/bash")
    lines.append("# Auto-generated from rviz config by rosshow")
    lines.append("# Run with: bash this_script.sh")
    lines.append("")
    lines.append("SESSION=rosshow_$$")
    lines.append("")
    lines.append("tmux new-session -d -s $SESSION")
    lines.append("")

    for i, d in enumerate(supported):
        window_idx = i
        if i == 0:
            lines.append("tmux rename-window -t $SESSION:0 '{}'".format(d["name"].replace("'", "")))
            lines.append("tmux send-keys -t $SESSION:0 '{} {} {} {}' C-m".format(
                rosshow_cmd, extra_args, d["topic"], ""
            ).strip())
        else:
            lines.append("tmux new-window -t $SESSION:{} -n '{}'".format(
                window_idx, d["name"].replace("'", "")))
            lines.append("tmux send-keys -t $SESSION:{} '{} {} {}' C-m".format(
                window_idx, rosshow_cmd, extra_args, d["topic"]
            ).strip())
        lines.append("")

    lines.append("tmux select-window -t $SESSION:0")
    lines.append("tmux attach-session -t $SESSION")
    lines.append("")

    script = "\n".join(lines)

    if output_path:
        with open(output_path, "w") as f:
            f.write(script)
        os.chmod(output_path, 0o755)

    return script
