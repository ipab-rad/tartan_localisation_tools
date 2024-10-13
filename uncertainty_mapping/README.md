## Docker commands to build/run the script (should easily adapt to other ros distro that provide the messages)

# TODO: make this into a Dockerfile to build/run?
docker run -it --network host -v `pwd`/:/data ros:iron
sudo apt-get update && sudo apt-get install ros-humble-novatel-gps-msgs ros-iron-novatel-gps-msgs python3-pip
pip install folium

# Example usage
python3 ./draw_uncertainty --input-bags data/2024_07_30-16_30_10_sensor_recording_0.mcap data/2024_07_30-16_30_10_sensor_recording_19.mcap --output-filename map.html

