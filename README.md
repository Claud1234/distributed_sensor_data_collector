# Rosbag Unpacker

Unpacks the rosbag containing camera images and radar pointclouds file into separate frames, serializes radar data into JSON format, and (optionally) stores them into a database.

## Usage

| Command       | Short Command | Desrciption                                                                                                                                                                                      |
| ------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `--output`    | `-o`          | Output directory to store the extracted frames                                                                                                                                                   |
| `--fps`       | `-f`          | Save data using this FPS value. Defaults to 10. Will generate additional frames, if this value is larger than actual video FPS. Will skip frames if this value is smaller than actual video FPS. |
| `--overwrite` | `-ow`         | Overwrite the output directory. If this value is not specified, the output directory will not be overwritten.                                                                                    |
| `--progress`  | `-p`          | Display progress bar                                                                                                                                                                             |
