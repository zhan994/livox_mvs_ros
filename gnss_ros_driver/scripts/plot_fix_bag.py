#!/usr/bin/env python3
"""Plot a GNSS NavSatFix bag as local ENU trajectory relative to the first fix."""

import argparse
import math
import os
import sys

import rosbag


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_enu(x, y, z, origin):
    lat0 = math.radians(origin["lat"])
    lon0 = math.radians(origin["lon"])
    sin_lat0 = math.sin(lat0)
    cos_lat0 = math.cos(lat0)
    sin_lon0 = math.sin(lon0)
    cos_lon0 = math.cos(lon0)

    dx = x - origin["x"]
    dy = y - origin["y"]
    dz = z - origin["z"]

    east = -sin_lon0 * dx + cos_lon0 * dy
    north = -sin_lat0 * cos_lon0 * dx - sin_lat0 * sin_lon0 * dy + cos_lat0 * dz
    up = cos_lat0 * cos_lon0 * dx + cos_lat0 * sin_lon0 * dy + sin_lat0 * dz
    return east, north, up


def valid_fix(msg, include_invalid):
    fields = (msg.latitude, msg.longitude, msg.altitude)
    if not all(math.isfinite(value) for value in fields):
        return False
    if not include_invalid and msg.status.status < 0:
        return False
    return True


def read_fix_trajectory(bag_path, topic, include_invalid):
    points = []
    origin = None

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            if not valid_fix(msg, include_invalid):
                continue

            alt = msg.altitude if math.isfinite(msg.altitude) else 0.0
            x, y, z = lla_to_ecef(msg.latitude, msg.longitude, alt)

            if origin is None:
                origin = {
                    "lat": msg.latitude,
                    "lon": msg.longitude,
                    "alt": alt,
                    "x": x,
                    "y": y,
                    "z": z,
                    "stamp": msg.header.stamp.to_sec()
                    if msg.header.stamp.to_sec() > 0.0
                    else t.to_sec(),
                }

            east, north, up = ecef_to_enu(x, y, z, origin)
            stamp = msg.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = t.to_sec()
            points.append((stamp, east, north, up, msg.latitude, msg.longitude, alt))

    return origin, points


def split_points(points):
    stamps = [point[0] for point in points]
    east = [point[1] for point in points]
    north = [point[2] for point in points]
    up = [point[3] for point in points]
    t0 = stamps[0]
    elapsed = [stamp - t0 for stamp in stamps]
    return stamps, elapsed, east, north, up


def derived_3d_output_path(output_path):
    if not output_path:
        return None
    root, ext = os.path.splitext(output_path)
    if not ext:
        ext = ".png"
    return "{}_3d{}".format(root, ext)


def set_axes_equal_3d(ax, east, north, up):
    x_min, x_max = min(east), max(east)
    y_min, y_max = min(north), max(north)
    z_min, z_max = min(up), max(up)
    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)
    z_mid = 0.5 * (z_min + z_max)
    radius = 0.5 * max(x_max - x_min, y_max - y_min, z_max - z_min, 1.0)

    ax.set_xlim(x_mid - radius, x_mid + radius)
    ax.set_ylim(y_mid - radius, y_mid + radius)
    ax.set_zlim(z_mid - radius, z_mid + radius)


def plot_2d_trajectory(origin, points, topic, output_path, show_plot):
    import matplotlib.pyplot as plt

    _, elapsed, east, north, up = split_points(points)

    fig, (ax_xy, ax_up) = plt.subplots(
        2,
        1,
        figsize=(10, 8),
        gridspec_kw={"height_ratios": [3, 1]},
        constrained_layout=True,
    )

    line = ax_xy.plot(east, north, linewidth=1.6, label="trajectory")[0]
    ax_xy.scatter(east[0], north[0], s=70, marker="o", color="green", label="start", zorder=3)
    ax_xy.scatter(east[-1], north[-1], s=70, marker="x", color="red", label="end", zorder=3)
    ax_xy.set_title(
        "GNSS /fix local ENU trajectory\n"
        "origin: lat={:.9f}, lon={:.9f}, alt={:.3f} m".format(
            origin["lat"], origin["lon"], origin["alt"]
        )
    )
    ax_xy.set_xlabel("East [m]")
    ax_xy.set_ylabel("North [m]")
    ax_xy.axis("equal")
    ax_xy.grid(True, linestyle="--", alpha=0.4)
    ax_xy.legend(loc="best")

    ax_up.plot(elapsed, up, color=line.get_color(), linewidth=1.4)
    ax_up.set_title("Up relative to origin")
    ax_up.set_xlabel("Time [s]")
    ax_up.set_ylabel("Up [m]")
    ax_up.grid(True, linestyle="--", alpha=0.4)

    fig.suptitle("{}: {} valid fixes".format(topic, len(points)))

    if output_path:
        fig.savefig(output_path, dpi=200)
        print("Saved plot: {}".format(output_path))
    if show_plot:
        plt.show()


def plot_3d_trajectory(origin, points, topic, output_path, show_plot):
    import matplotlib.pyplot as plt

    _, _, east, north, up = split_points(points)

    fig = plt.figure(figsize=(10, 8), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(east, north, up, linewidth=1.6, label="trajectory")
    ax.scatter(east[0], north[0], up[0], s=70, marker="o", color="green", label="start")
    ax.scatter(east[-1], north[-1], up[-1], s=70, marker="x", color="red", label="end")
    ax.set_title(
        "GNSS /fix local ENU 3D trajectory\n"
        "origin: lat={:.9f}, lon={:.9f}, alt={:.3f} m".format(
            origin["lat"], origin["lon"], origin["alt"]
        )
    )
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.set_zlabel("Up [m]")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best")
    set_axes_equal_3d(ax, east, north, up)
    fig.suptitle("{}: {} valid fixes".format(topic, len(points)))

    if output_path:
        fig.savefig(output_path, dpi=200)
        print("Saved 3D plot: {}".format(output_path))
    if show_plot:
        plt.show()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Visualize a sensor_msgs/NavSatFix bag topic in a local ENU frame."
    )
    parser.add_argument("bag", help="Input rosbag file recorded from gnss_ros_driver.")
    parser.add_argument(
        "--topic",
        default="/fix",
        help="NavSatFix topic name in the bag. Default: /fix",
    )
    parser.add_argument(
        "-o",
        "--output",
        help="Optional 2D output image path, for example trajectory.png.",
    )
    parser.add_argument(
        "--plot-3d",
        action="store_true",
        help="Also plot the local ENU trajectory in 3D.",
    )
    parser.add_argument(
        "--only-3d",
        action="store_true",
        help="Only plot the 3D trajectory.",
    )
    parser.add_argument(
        "--output-3d",
        help=(
            "Optional 3D output image path. If omitted with --plot-3d and -o, "
            "uses the 2D output name with _3d appended."
        ),
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save only; do not open the matplotlib window.",
    )
    parser.add_argument(
        "--include-invalid",
        action="store_true",
        help="Include NavSatFix messages whose status is STATUS_NO_FIX.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    try:
        origin, points = read_fix_trajectory(args.bag, args.topic, args.include_invalid)
    except rosbag.ROSBagException as exc:
        print("Failed to read bag: {}".format(exc), file=sys.stderr)
        return 1

    if not points:
        print(
            "No valid NavSatFix messages found on topic '{}'. "
            "Use --include-invalid if the bag only contains STATUS_NO_FIX messages.".format(
                args.topic
            ),
            file=sys.stderr,
        )
        return 1

    show_plot = not args.no_show
    plot_3d = args.plot_3d or args.only_3d

    if not args.only_3d:
        plot_2d_trajectory(origin, points, args.topic, args.output, show_plot)

    if plot_3d:
        output_3d = args.output_3d or derived_3d_output_path(args.output)
        plot_3d_trajectory(origin, points, args.topic, output_3d, show_plot)

    return 0


if __name__ == "__main__":
    sys.exit(main())
