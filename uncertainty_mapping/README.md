# Uncertainty mapping visualisation

This component processes input mcap (ROSbags) and produces a map.html file.
Opening the map.html file in a browser shows the AV GPS trace and draws ellipses
representing the covariance (uncertainty) of the AV location.

## Usage
Run script to build docker image and access interactive container:

```bash
./dev.sh -p /path/to/your/rosbags
```

Then inside the container:

``` bash
python3 ./src/uncertainty_mapping/draw_uncertainty.py \
--input-bags rosbags/my_rosbags_dir \
--output-filename my_map  # .html ext will be added automatically
```

Once the script completes, the generated `.html` file will be located in the `output` directory. You can open it in your browser to view the map. For more information, refer to the [map.html](../data/map.html) example.
