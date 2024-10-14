# Uncertainty mapping visualisation

This component processes input mcap (ROSbags) and produces a map.html file.
Opening the map.html file in a browser shows the AV GPS trace and draws ellipses
representing the covariance (uncertainty) of the AV location.

## Usage
Run script to build docker image and access interactive container:

```
./dev.sh -p /path/to/mcap
```

Then inside the container:

```
python3 ./src/uncertainty_mapping/draw_uncertainty.py \
--input-bags rosbags/example.mcap \
--output-filename output/example.html
```
