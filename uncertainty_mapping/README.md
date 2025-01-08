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
draw_uncertainty.py \
--input-bags-dir rosbags/my_rosbags_dir \
--output-filename output/<my_map>.html
```

#### Important notes

- The `--output-filename` parameter should start with `output/` when running the script in the Docker container to ensure the `.html` file is saved correctly. This is not required when running the script locally.

- Replace `<my_map>` with your desired filename (without brackets).

Once the script completes, the generated `.html` file will be located in your local `output` directory. You can open it in your browser to view the map. For more information, refer to the [map.html](../data/map.html) example.
