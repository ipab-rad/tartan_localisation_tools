import os
import argparse
import re
import rosbag2_py
import folium
import numpy as np
import math
from dataclasses import dataclass
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from novatel_gps_msgs.msg import NovatelPosition


def extract_gps_traces(bag_path, topic_name):
    reader = rosbag2_py.SequentialReader()

    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )
    topic_types = reader.get_all_topics_and_types()

    topic_type_dict = {topic.name: topic.type for topic in topic_types}

    if topic_name not in topic_type_dict:
        print(f"Topic '{topic_name}' not found in the bag.")
        return []

    traces = []
    msg_type = get_message(topic_type_dict[topic_name])
    while reader.has_next():
        (topic, data, timestamp) = reader.read_next()
        if topic == topic_name:
            # Assuming the message type is a plain string
            traces.append(deserialize_message(data, msg_type))
    return traces


def generate_ellipse_points(center, width, height, angle, num_points=100):
    """Generate points along the perimeter of an ellipse."""
    cx, cy = center
    t = np.linspace(0, 2 * np.pi, num_points)
    ellipse_points = np.array([width * np.cos(t), height * np.sin(t)]).T

    # Rotate points by the given angle
    R = np.array(
        [[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]]
    )
    rotated_points = np.dot(ellipse_points, R)

    # Translate points to the center
    ellipse_points = rotated_points + np.array([cx, cy])
    return [[pt[1], pt[0]] for pt in ellipse_points]


def draw_uncertainty_ellipse(
    m,
    center,
    x_uncertainty,
    y_uncertainty,
    angle=0,
    color="grey",
    fill_color="grey",
    fill_opacity=0.2,
):
    """Draw an ellipse of uncertainty on a folium map."""
    # Convert uncertainty from meters to degrees (approximate)
    lat_per_meter = (
        1 / 111320
    )  # One degree of latitude is approximately 111.32 km
    lon_per_meter = 1 / (
        40075000 * np.cos(np.radians(center[0])) / 360
    )  # One degree of longitude varies based on latitude

    width = x_uncertainty * lon_per_meter
    height = y_uncertainty * lat_per_meter

    # Generate ellipse points
    ellipse_points = generate_ellipse_points(
        (center[1], center[0]), width, height, np.radians(angle)
    )

    # Convert ellipse points to list of tuples
    ellipse_points_list = list(map(tuple, ellipse_points))

    # Add ellipse to the map as a polygon
    folium.Polygon(
        locations=ellipse_points_list,
        color=color,
        fill=True,
        fill_color=fill_color,
        fill_opacity=fill_opacity,
    ).add_to(m)


# https://github.com/swri-robotics/novatel_gps_driver/blob/ros2-devel/novatel_gps_msgs/msg/NovatelPosition.msg
def convert_trace(gps_pos):
    return NovatelPosition(
        lat=gps_pos.lat,
        lon=gps_pos.lon,
        lon_sigma=gps_pos.lon_sigma,
        lat_sigma=gps_pos.lat_sigma,
    )


def haversine(lat1, lon1, lat2, lon2):
    # Radius of the Earth in meters
    R = 6371000
    # Convert degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # Haversine formula
    a = (
        math.sin(delta_phi / 2) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    )
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    return distance


def filter_points(points, min_distance=3):
    if not points:
        return []

    filtered_points = [points[0]]

    for point in points[1:]:
        last_point = filtered_points[-1]
        distance = haversine(
            last_point.lat, last_point.lon, point.lat, point.lon
        )
        if distance >= min_distance:
            filtered_points.append(point)

    return filtered_points


def sort_by_numeric_suffix(files):
    def extract_number(file):
        match = re.search(r"_(\d+)\.mcap$", file)
        return (
            int(match.group(1)) if match else float('inf')
        )  # Non-matching files go to the end

    return sorted(files, key=extract_number)


if __name__ == "__main__":
    # Assume all bags belong to the same drive
    parser = argparse.ArgumentParser(
        prog="uncertainty_mapper",
        description="Draws gps uncertainty onto a map to help indicate poor GPS locations/performance visually",
    )
    parser.add_argument("--input-bags-dir", "-I", required=True)
    parser.add_argument("--output-filename", "-O", default="map")

    args = parser.parse_args()

    input_path = args.input_bags_dir
    mcap_path_list = []
    if os.path.isdir(input_path):
        # Get all .mcap files in the directory
        for file_name in sorted(os.listdir(input_path)):
            if file_name.endswith('.mcap'):
                mcap_path = os.path.join(input_path, file_name)
                mcap_path_list.append(mcap_path)
    elif os.path.isfile(input_path) and input_path.endswith('.mcap'):
        mcap_path_list.append(input_path)
    else:
        print('The input path is not a valid .mcap file or directory')

    mcap_path_list = sort_by_numeric_suffix(mcap_path_list)

    total_rosbags = len(mcap_path_list)

    print(f'\nFound {total_rosbags} rosbags files\n')

    TOPIC_NAME = "/sensor/gps/bestpos"
    print(f"Script will search for GPS information in '{TOPIC_NAME}' topic\n")

    gps_traces = []
    idx = 0
    for mcap_path in mcap_path_list:
        print(f'\r{idx}/{total_rosbags} rosbags processed', end='', flush=True)
        gps_traces.extend(
            extract_gps_traces(bag_path=mcap_path, topic_name=TOPIC_NAME)
        )
        idx += 1

    assert len(gps_traces) > 1, "No GPS traces present in bag"

    # Downsample to every 10th point
    gps_traces = filter_points(gps_traces)

    # Add first point
    first_point = convert_trace(gps_traces[0])
    m = folium.Map(location=(first_point.lat, first_point.lon), zoom_start=10)

    # Add ellipses and markers for each coordinate
    for trace in gps_traces:
        pt = convert_trace(trace)
        folium.Circle(
            location=(pt.lat, pt.lon),
            radius=0.2,  # Radius in meters
            color="blue",
            fill=True,
            fill_color="blue",
            fill_opacity=1,
        ).add_to(m)
        draw_uncertainty_ellipse(
            m, (pt.lat, pt.lon), pt.lon_sigma, pt.lat_sigma
        )

    # Save the map to an HTML file
    m.save("output/" + args.output_filename + ".html")
