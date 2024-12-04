#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

BASH_CMD=""

# Function to print usage
usage() {
    echo "
Usage: dev.sh [-b|bash] [--path | -p ] [-h|--help]

Where:
    -b | bash       Open bash in docker container (Default in dev.sh)
    -p | --path   ROSBAGS_DIR_PATH
                    Specify path for script to access mcap rosbags
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
    echo "$ROSBAGS_DIR does not exist! Please provide a valid path with mcap files"
    exit 1
fi

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t tartan_localisation_tools:latest-dev \
    -f Dockerfile --target dev .

# Create directory to save generated maps
mkdir -p ./output

# Run docker image with local code volumes for development
docker run -it --rm --net host \
    -v /dev:/dev \
    -v /tmp:/tmp \
    -v /etc/localtime:/etc/localtime:ro \
    -v ./uncertainty_mapping:/opt/ros_ws/src/uncertainty_mapping \
    -v ./output:/opt/ros_ws/output \
    -v $ROSBAGS_DIR:/opt/ros_ws/rosbags \
    tartan_localisation_tools:latest-dev
