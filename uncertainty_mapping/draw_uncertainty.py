import argparse
import rosbag2_py
import folium
import numpy as np
import math
from dataclasses import dataclass
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

@dataclass
class GpsPoint:
    """Class for keeping track of an item in inventory."""
    lat: float
    lon: float
    x_uncertainty: float
    y_uncertainty: float

def extract_gps_traces(bag_path, topic_name):
    reader = rosbag2_py.SequentialReader()
    
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap'),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    )
    topic_types = reader.get_all_topics_and_types()
    
    topic_type_dict = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in topic_type_dict:
        print(f"Topic '{topic_name}' not found in the bag.")
        return []
    
    print(f"Reading messages from topic '{topic_name}':")
    
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
    R = np.array([[np.cos(angle), -np.sin(angle)], 
                  [np.sin(angle),  np.cos(angle)]])
    rotated_points = np.dot(ellipse_points, R)

    # Translate points to the center
    ellipse_points = rotated_points + np.array([cx, cy])
    return [[pt[1], pt[0]] for pt in ellipse_points]
   
def draw_uncertainty_ellipse(m, center, x_uncertainty, y_uncertainty, angle=0, color='grey', fill_color='grey', fill_opacity=0.2):
    """Draw an ellipse of uncertainty on a folium map."""
    # Convert uncertainty from meters to degrees (approximate)
    lat_per_meter = 1 / 111320  # One degree of latitude is approximately 111.32 km
    lon_per_meter = 1 / (40075000 * np.cos(np.radians(center[0])) / 360)  # One degree of longitude varies based on latitude

    width = x_uncertainty * lon_per_meter
    height = y_uncertainty * lat_per_meter

    # Generate ellipse points
    ellipse_points = generate_ellipse_points((center[1], center[0]), width, height, np.radians(angle))

    # Convert ellipse points to list of tuples
    ellipse_points_list = list(map(tuple, ellipse_points))
    
    # Add ellipse to the map as a polygon
    folium.Polygon(
        locations=ellipse_points_list,
        color=color,
        fill=True,
        fill_color=fill_color,
        fill_opacity=fill_opacity
    ).add_to(m)

# Assumed to be Novatel BESTPOS.msg    https://github.com/novatel/novatel_oem7_driver/blob/master/src/novatel_oem7_msgs/msg/BESTPOS.msg
def convert_trace(gps_pos):
    return GpsPoint(lat=gps_pos.lat, lon=gps_pos.lon, x_uncertainty=gps_pos.lon_sigma, y_uncertainty=gps_pos.lat_sigma)

def haversine(lat1, lon1, lat2, lon2):
    # Radius of the Earth in meters
    R = 6371000
    # Convert degrees to radians
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    # Haversine formula
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c
    
    return distance

def filter_points(points, min_distance=3):
    if not points:
        return []
    
    filtered_points = [points[0]]
    
    for point in points[1:]:
        last_point = filtered_points[-1]
        distance = haversine(last_point.lat, last_point.lon, point.lat, point.lon)
        if distance >= min_distance:
            filtered_points.append(point)
    
    return filtered_points

if __name__ == "__main__":
    # Assume all bags belong to the same drive
    parser = argparse.ArgumentParser(
                    prog='uncertainty_mapper',
                    description='Draws gps uncertainty onto a map to help indicate poor GPS locations/performance visually')                
    parser.add_argument('--input-bags', "-I", nargs='+')
    parser.add_argument("--output-filename", "-O", default="map.html")
    
    args = parser.parse_args()

    gps_traces=[]
    for path in args.input_bags:
        gps_traces.extend(extract_gps_traces(bag_path=path, topic_name="/sensor/gps/bestpos"))
        
        
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
            fill_opacity=1
        ).add_to(m)
        draw_uncertainty_ellipse(m, (pt.lat, pt.lon), pt.x_uncertainty, pt.y_uncertainty)

    # Save the map to an HTML file
    m.save('map.html')

