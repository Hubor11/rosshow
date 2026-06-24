# This file should be written to be both python2 and python3 compatible

import os

try:
    import rospy # ROS1
except ImportError:
    import rosshow.rospy2 as rospy # ROS2
except ModuleNotFoundError as e:
    print(str(e))
    exit(1)

import sys
import time
import random
import threading
from rosshow.getch import Getch
import rosshow.termgraphics as termgraphics

getch = Getch()

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

def capture_key_loop(viewer):
    global getch
    while True:
        c = getch()
        if c == '\x03': # Ctrl+C
            rospy.signal_shutdown("Ctrl+C pressed")

        if "keypress" not in dir(viewer):
            continue

        if c == '\x1B': # ANSI escape
            c2 = getch()
            if c2 == '\x5B':
                c3 = getch()
                if c3 == '\x41':
                    viewer.keypress("up")
                if c3 == '\x42':
                    viewer.keypress("down")
                if c3 == '\x43':
                    viewer.keypress("left")
                if c3 == '\x44':
                    viewer.keypress("right")
                if c3 == '\x5A':
                    viewer.keypress("~")
            elif c2 == 'O':
                c3 = getch()
            else:
                viewer.keypress(c)
        elif c in "123456789" and hasattr(viewer, "focus_index"):
            idx = int(c) - 1
            if hasattr(viewer, "viewers") and idx < len(viewer.viewers):
                viewer.focus_index = idx
        else:
            viewer.keypress(c)


def _print_usage():
    print("Usage: rosshow [options] <topic>")
    print("       rosshow --from-rviz <config.rviz> [options]")
    print("       rosshow --nav [topics]")
    print("")
    print("Single-topic mode:")
    print("   rosshow <topic>")
    print("")
    print("Multi-topic from rviz config:")
    print("   rosshow --from-rviz <config.rviz>")
    print("   rosshow --from-rviz <config.rviz> --tmux-script out.sh")
    print("   rosshow --from-rviz <config.rviz> --list")
    print("")
    print("Navigation mode (visualize path planning):")
    print("   rosshow --nav")
    print("   rosshow --nav --global /move_base/GlobalPlanner/plan")
    print("   rosshow --nav --local /move_base/TebLocalPlannerROS/local_plan")
    print("   rosshow --nav --odom /odom --scan /scan --goal /move_base_simple/goal")
    print("")
    print("Options:")
    print("   -a, --ascii        Use ASCII only (no Unicode)")
    print("   -c1                Force monochrome")
    print("   -c4                Force 4-bit color (16 colors)")
    print("   -c24               Force 24-bit color")
    print("   --reliable         reliability QoS in ros2 (default: best_effort)")
    print("   --transient-local  durability QoS in ros2 (default: volatile)")
    print("   --from-rviz FILE   Load displays from an rviz config file")
    print("   --tmux-script FILE Generate a tmux script instead of inline display")
    print("   --list             List parsed displays and exit")
    print("   --nav              Navigation mode: visualize path planning")
    print("   --global TOPIC     Global path topic (default: /move_base/GlobalPlanner/plan)")
    print("   --local TOPIC      Local path topic (default: /move_base/local_plan)")
    print("   --odom TOPIC       Odometry topic (default: /odom)")
    print("   --scan TOPIC       Laser scan topic (default: /scan)")
    print("   --goal TOPIC       Goal pose topic (default: /move_base_simple/goal)")
    sys.exit(0)


def _parse_args():
    args = {
        "topic": None,
        "use_ascii": False,
        "color_support": None,
        "qos_reliable": False,
        "qos_transient_local": False,
        "from_rviz": None,
        "tmux_script": None,
        "list_only": False,
        "navigation_mode": False,
        "nav_global_topic": "/move_base/GlobalPlanner/plan",
        "nav_local_topic": "/move_base/local_plan",
        "nav_odom_topic": "/odom",
        "nav_scan_topic": "/scan",
        "nav_goal_topic": "/move_base_simple/goal",
    }

    if len(sys.argv) < 2:
        _print_usage()

    i = 1
    positional = []
    while i < len(sys.argv):
        arg = sys.argv[i]
        if arg == "-a" or arg == "--ascii":
            args["use_ascii"] = True
        elif arg == "-c1":
            args["color_support"] = termgraphics.COLOR_SUPPORT_1
        elif arg == "-c4":
            args["color_support"] = termgraphics.COLOR_SUPPORT_16
        elif arg == "-c24":
            args["color_support"] = termgraphics.COLOR_SUPPORT_24BIT
        elif arg == "--reliable":
            args["qos_reliable"] = True
        elif arg == "--transient-local":
            args["qos_transient_local"] = True
        elif arg == "--from-rviz":
            i += 1
            if i >= len(sys.argv):
                print("Error: --from-rviz requires a file path")
                sys.exit(1)
            args["from_rviz"] = sys.argv[i]
        elif arg == "--tmux-script":
            i += 1
            if i >= len(sys.argv):
                print("Error: --tmux-script requires a file path")
                sys.exit(1)
            args["tmux_script"] = sys.argv[i]
        elif arg == "--list":
            args["list_only"] = True
        elif arg == "--nav":
            args["navigation_mode"] = True
        elif arg == "--global":
            i += 1
            if i >= len(sys.argv):
                print("Error: --global requires a topic name")
                sys.exit(1)
            args["nav_global_topic"] = sys.argv[i]
        elif arg == "--local":
            i += 1
            if i >= len(sys.argv):
                print("Error: --local requires a topic name")
                sys.exit(1)
            args["nav_local_topic"] = sys.argv[i]
        elif arg == "--odom":
            i += 1
            if i >= len(sys.argv):
                print("Error: --odom requires a topic name")
                sys.exit(1)
            args["nav_odom_topic"] = sys.argv[i]
        elif arg == "--scan":
            i += 1
            if i >= len(sys.argv):
                print("Error: --scan requires a topic name")
                sys.exit(1)
            args["nav_scan_topic"] = sys.argv[i]
        elif arg == "--goal":
            i += 1
            if i >= len(sys.argv):
                print("Error: --goal requires a topic name")
                sys.exit(1)
            args["nav_goal_topic"] = sys.argv[i]
        elif arg == "-h" or arg == "--help":
            _print_usage()
        elif arg.startswith("-"):
            print("Warning: unknown option {}".format(arg))
        else:
            positional.append(arg)
        i += 1

    if args["from_rviz"] is None and not args["navigation_mode"] and len(positional) > 0:
        args["topic"] = positional[0]

    if args["from_rviz"] is None and not args["navigation_mode"] and args["topic"] is None:
        _print_usage()

    return args


def _normalize_topic_type(topic_type):
    if rospy.__name__ == "rospy2":
        return topic_type.replace("/msg/", "/")
    return topic_type


def _get_qos_kwargs(qos_reliable, qos_transient_local):
    kwargs = {}
    if rospy.__name__ == "rospy2":
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        kwargs = {"qos": QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE if qos_reliable else QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL if qos_transient_local else QoSDurabilityPolicy.VOLATILE,
        )}
    return kwargs


def _run_single_topic(topic, use_ascii, color_support, qos_reliable, qos_transient_local):
    rospy.init_node('rosshow', anonymous=True)

    time.sleep(1)
    topic_types = dict(rospy.get_published_topics())
    if topic not in topic_types:
        print("Topic {0} does not appear to be published yet.".format(topic))
        sys.exit(0)

    topic_type = _normalize_topic_type(topic_types[topic])

    if topic_type not in VIEWER_MAPPING:
        print("Unsupported message type: {}".format(topic_type))
        sys.exit(1)

    canvas = termgraphics.TermGraphics(
        mode=(termgraphics.MODE_EASCII if use_ascii else termgraphics.MODE_UNICODE),
        color_support=color_support)

    module_name, class_name, viewer_kwargs = VIEWER_MAPPING[topic_type]
    viewer_class = getattr(__import__(module_name, fromlist=(class_name)), class_name)
    viewer = viewer_class(canvas, title=topic, **viewer_kwargs)

    message_package, message_name = topic_type.split("/", 2)
    message_class = getattr(__import__(message_package + ".msg", fromlist=(message_name)), message_name)

    kwargs = _get_qos_kwargs(qos_reliable, qos_transient_local)
    rospy.Subscriber(topic, message_class, viewer.update, **kwargs)

    thread = threading.Thread(target=capture_key_loop, args=(viewer,))
    thread.daemon = True
    thread.start()

    _run_draw_loop(canvas, viewer)


def _run_multi_topic(topic_specs, use_ascii, color_support, qos_reliable, qos_transient_local, title="rosshow"):
    from rosshow.viewers.generic.MultiViewer import MultiViewer

    rospy.init_node('rosshow', anonymous=True)

    time.sleep(1)
    topic_types = dict(rospy.get_published_topics())

    valid_specs = []
    for spec in topic_specs:
        topic = spec["topic"]
        if topic not in topic_types:
            print("Warning: topic {} not published yet, skipping".format(topic))
            continue
        actual_type = _normalize_topic_type(topic_types[topic])
        spec_type = spec.get("type", spec.get("ros_type", ""))
        if spec_type and spec_type != actual_type:
            print("Warning: topic {} type mismatch: expected {}, got {}".format(
                topic, spec_type, actual_type))
            spec = dict(spec)
        spec["type"] = actual_type
        if actual_type not in VIEWER_MAPPING:
            print("Warning: unsupported message type {} for topic {}, skipping".format(actual_type, topic))
            continue
        valid_specs.append(spec)

    if not valid_specs:
        print("No valid topics to display.")
        sys.exit(1)

    canvas = termgraphics.TermGraphics(
        mode=(termgraphics.MODE_EASCII if use_ascii else termgraphics.MODE_UNICODE),
        color_support=color_support)

    viewer = MultiViewer(canvas, valid_specs, title=title)

    kwargs = _get_qos_kwargs(qos_reliable, qos_transient_local)

    for idx, sub_config in enumerate(viewer.get_subscribers_config()):
        topic = sub_config["topic"]
        topic_type = sub_config["type"]
        message_package, message_name = topic_type.split("/", 2)
        message_class = getattr(__import__(message_package + ".msg", fromlist=(message_name)), message_name)
        rospy.Subscriber(topic, message_class, sub_config["callback"], **kwargs)

    thread = threading.Thread(target=capture_key_loop, args=(viewer,))
    thread.daemon = True
    thread.start()

    _run_draw_loop(canvas, viewer)


def _run_draw_loop(canvas, viewer):
    frame_rate = 15.
    frame_duration = 1. / frame_rate
    try:
        while not rospy.is_shutdown():
            start_time = time.time()
            viewer.draw()
            stop_time = time.time()
            draw_time = stop_time - start_time
            delay_time = max(0, frame_duration - draw_time)
            time.sleep(delay_time)
    except rospy.exceptions.ROSInterruptException:
        sys.stdout.write("")
    except KeyboardInterrupt:
        pass
    finally:
        getch.reset()
        sys.stdout.write("\033[%d;0H\n" % canvas.term_shape[1])
        sys.stdout.flush()


def _run_navigation(args):
    from rosshow.viewers.nav_msgs.NavigationViewer import NavigationViewer

    rospy.init_node('rosshow_nav', anonymous=True)

    time.sleep(1)
    topic_types = dict(rospy.get_published_topics())

    canvas = termgraphics.TermGraphics(
        mode=(termgraphics.MODE_EASCII if args["use_ascii"] else termgraphics.MODE_UNICODE),
        color_support=args["color_support"])

    viewer = NavigationViewer(canvas, title="Navigation")

    kwargs = _get_qos_kwargs(args["qos_reliable"], args["qos_transient_local"])

    if args["nav_global_topic"] in topic_types:
        from nav_msgs.msg import Path
        rospy.Subscriber(args["nav_global_topic"], Path, viewer.update_global_path, **kwargs)
        print("Subscribed to global path: {}".format(args["nav_global_topic"]))
    else:
        print("Warning: global path topic {} not published".format(args["nav_global_topic"]))

    if args["nav_local_topic"] in topic_types:
        from nav_msgs.msg import Path
        rospy.Subscriber(args["nav_local_topic"], Path, viewer.update_local_path, **kwargs)
        print("Subscribed to local path: {}".format(args["nav_local_topic"]))
    else:
        print("Warning: local path topic {} not published".format(args["nav_local_topic"]))

    if args["nav_odom_topic"] in topic_types:
        from nav_msgs.msg import Odometry
        rospy.Subscriber(args["nav_odom_topic"], Odometry, viewer.update_odometry, **kwargs)
        print("Subscribed to odometry: {}".format(args["nav_odom_topic"]))
    else:
        print("Warning: odometry topic {} not published".format(args["nav_odom_topic"]))

    if args["nav_scan_topic"] in topic_types:
        from sensor_msgs.msg import LaserScan
        rospy.Subscriber(args["nav_scan_topic"], LaserScan, viewer.update_laserscan, **kwargs)
        print("Subscribed to laser scan: {}".format(args["nav_scan_topic"]))
    else:
        print("Warning: laser scan topic {} not published".format(args["nav_scan_topic"]))

    if args["nav_goal_topic"] in topic_types:
        from geometry_msgs.msg import PoseStamped
        rospy.Subscriber(args["nav_goal_topic"], PoseStamped, viewer.update_goal, **kwargs)
        print("Subscribed to goal: {}".format(args["nav_goal_topic"]))
    else:
        print("Warning: goal topic {} not published".format(args["nav_goal_topic"]))

    thread = threading.Thread(target=capture_key_loop, args=(viewer,))
    thread.daemon = True
    thread.start()

    _run_draw_loop(canvas, viewer)


def main():
    args = _parse_args()

    if args["navigation_mode"]:
        _run_navigation(args)
    elif args["from_rviz"]:
        from rosshow.rviz_config import parse_rviz_config, get_supported_displays, generate_tmux_script

        try:
            displays = parse_rviz_config(args["from_rviz"])
        except Exception as e:
            print("Error parsing rviz config: {}".format(str(e)))
            sys.exit(1)

        supported = get_supported_displays(displays)

        if args["list_only"]:
            print("Displays found in {}:".format(args["from_rviz"]))
            print("")
            print("  #  {:<30s} {:<30s} {:<25s} {}".format(
                "Name", "rviz Class", "ROS Type", "Topic"))
            print("  " + "-" * 100)
            for i, d in enumerate(displays):
                status = " " if d["enabled"] else " (disabled)"
                supported_mark = " " if d["ros_type"] else " [unsupported]"
                print("  {:>2d} {:<30s} {:<30s} {:<25s} {}{}{}".format(
                    i + 1,
                    d["name"][:28],
                    d["rviz_class"][:28],
                    (d["ros_type"] or "N/A")[:23],
                    d["topic"] or "N/A",
                    status,
                    supported_mark,
                ))
            print("")
            print("Supported: {} / {} displays".format(len(supported), len(displays)))
            return

        if not supported:
            print("No supported displays found in {}".format(args["from_rviz"]))
            sys.exit(1)

        if args["tmux_script"]:
            extra = ""
            if args["use_ascii"]:
                extra += "-a "
            if args["color_support"] == termgraphics.COLOR_SUPPORT_1:
                extra += "-c1 "
            elif args["color_support"] == termgraphics.COLOR_SUPPORT_16:
                extra += "-c4 "
            elif args["color_support"] == termgraphics.COLOR_SUPPORT_24BIT:
                extra += "-c24 "
            if args["qos_reliable"]:
                extra += "--reliable "
            if args["qos_transient_local"]:
                extra += "--transient-local "

            script = generate_tmux_script(
                displays,
                output_path=args["tmux_script"],
                extra_args=extra.strip(),
            )
            print("Generated tmux script: {}".format(args["tmux_script"]))
            print("Run with: bash {}".format(args["tmux_script"]))
            return

        topic_specs = []
        for d in supported:
            topic_specs.append({
                "topic": d["topic"],
                "type": d["ros_type"],
                "name": d["name"],
            })

        title = os.path.basename(args["from_rviz"])
        _run_multi_topic(
            topic_specs,
            args["use_ascii"],
            args["color_support"],
            args["qos_reliable"],
            args["qos_transient_local"],
            title=title,
        )
    else:
        _run_single_topic(
            args["topic"],
            args["use_ascii"],
            args["color_support"],
            args["qos_reliable"],
            args["qos_transient_local"],
        )


if __name__ == "__main__":
    main()
