#!/bin/bash
# ---------------------------------------------------------------------------
# Build docker image and run ROS code for runtime or interactively with bash
# ---------------------------------------------------------------------------

BASH_CMD=""

# Function to print usage
usage() {
    echo "
Usage: runtime.sh [-b|bash] [--path | -p ] [-h|--help]

Where:
    -b | bash       Open bash in docker container (Default in dev.sh)
    -p | --path   ROSBAGS_DIR_PATH
                    Specify path to store recorded rosbags
    -h | --help     Show this help message
    "
    exit 1
}

# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -b|bash)
            BASH_CMD=bash
            ;;
        -p|--path)
            if [[ -n "$2" && "$2" != -* ]]; then
                ROSBAGS_DIR="$2"
                shift
            else
                echo "Error: Argument for $1 is missing."
                usage
            fi
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
    shift
done

# Verify ROSBAGS_DIR exists
if [ ! -d "$ROSBAGS_DIR" ]; then
    echo "$ROSBAGS_DIR does not exist! Please provide a valid path to store rosbags"
    exit 1
fi

# Build docker image only up to base stage
DOCKER_BUILDKIT=1 docker build \
    -t tartan_localisation_tools:latest \
    -f Dockerfile --target runtime .

# Run docker image without volumes
docker run -it --rm --net host \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v /etc/localtime:/etc/localtime:ro \
    -v ./output:/opt/ros_ws/output \
    -v $ROSBAGS_DIR:/opt/ros_ws/rosbags \
    tartan_localisation_tools:latest $CMD
