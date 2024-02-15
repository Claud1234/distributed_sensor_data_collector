# End-to-End Multimodal Sensor Dataset Collection Framework for Autonomous Vehicles 
This is the source code for paper: https://doi.org/10.3390/s23156783

Cite as:
```
@Article{s23156783,
AUTHOR = {Gu, Junyi and Lind, Artjom and Chhetri, Tek Raj and Bellone, Mauro and Sell, Raivo},
TITLE = {End-to-End Multimodal Sensor Dataset Collection Framework for Autonomous Vehicles},
JOURNAL = {Sensors},
VOLUME = {23},
YEAR = {2023},
NUMBER = {15},
ARTICLE-NUMBER = {6783},
URL = {https://www.mdpi.com/1424-8220/23/15/6783},
PubMedID = {37571566},
ISSN = {1424-8220},
DOI = {10.3390/s23156783}
}
```

TO BE CONTINUE.....



## Database 
The database was dumped in docker. Please make sure you have the docker compose installed in your system. 
It is recommended here to install the docker compose through the repository instead of downloading the binary. 
Check docker compose with 

```
docker compose version
```

Go to the 'database' folder, make sure the 'docker-compose.yml' is available, and create the database.

```
cd finest_mobility/database
docker compose up -d
docker compose run mysql bash -c "mysql -h host.docker.internal -P 3306 -u db_user -ptransport123 transport_ecosystem_management_db < /tmp/sql/database.sql"
```

Then the docker will pull the corresponding images and running in background. If you want to close the database, run command

```
docker compose down
```


## Data Unpack
Unpacks the rosbag containing camera images and radar point clouds file into separate 
frames, serializes radar data into JSON format, and (optionally) stores them into a database.

### Dependencies 
It is recommended to use the python3 for executing the scripts. Install the python modules

```
pip3 install mysql mysql-connector Pillow argparse progressbar2 opencv-python pyyaml numpy dataclasses pycryptodomex gnupg rospkg 
```

### Usage
If it is the new-deployed dataset, the first step is define the 'sensor_type' and 
'sensor' fields in database. Please noted that these things will be saved in 
database permanently until deleting the whole docker volume.

The 'manage_models_sensors.py' is the script to input the sensors and models into 
database. It contains the flags
              
| Command              | Short Command | Desrciption                                                                                                   |
|----------------------|---------------|---------------------------------------------------------------------------------------------------------------
| `--insert`           | `-i`          | Insert objects into database.                                                                                 |
| `--remove`           | `-r`          | Remove the objects from database                                                                              
| `--list`             | `-l`          | List the objects in database                                                                                  |
| `--machine-learning` | `-ml`         |                                                             |
| `--dataset`          | `-ow`         |  |
| `--detection-model`  | `-p`          |                                                                                           |
| `--sensor-type`      | `-st`         |  Input the sensor types in database
| `--sensor`           | `-s`          |  Input the sensors in dataset

The fist step is adding the types of the sensors into the database.
In our case, there are three types 'lidar' 'radar', and 'camera'.
The example to add the sensor type

```
python3 manage_models_sensors.py -i -st
```

A unique ID will be assigned to each 'sensor_type', then add the 'sensor' with
corresponding 'sensor_type'. We have one Velodyne VLP-32C LiDAD, one FLIR 
Grasshopper3 camera, one raspi camera, and two TI mmwave awr1843 radars.
The example to add the sensor
```
python3 manage_models_sensors.py -i -s
```
After input the 'name' field, there is a need to specify the sensor type ID 
and the topic names.  

Please note that database will only store the data from the topics specified 
here, other ROS topics will be ignored. 
 







| Command       | Short Command | Desrciption                                                                                                                                                                                      |
| ------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `--rosbag`    | `-r`          | Rosbag to process                                                                                                                                                                                |
| `--output`    | `-o`          | Output directory to store the extracted frames. The program will create a folder with the name of the rosbag in the folder specified by the `--output` flag                                      |
| `--fps`       | `-f`          | Save data using this FPS value. Defaults to 10. Will generate additional frames, if this value is larger than actual video FPS. Will skip frames if this value is smaller than actual video FPS. |
| `--2d`        |               | Add this flag if radar is using a 2D configuration                                                                                                                                               |
| `--overwrite` | `-ow`         | Overwrite the output directory. If this value is not specified, the output directory will not be overwritten.                                                                                    |
| `--progress`  | `-p`          | Display progress bar                                                                                                                                                                             |

## Object Detector
Reads images and radar files, performs machine learning on the images and stores 
data in the database

### Dependencies

```
pip3 install "tensorflow>=2.5" "cryptography>=2.5" "greenlet>=0.3" "python-dateutil<3.0.0" "cffi>=1.1" "SQLAlchemy>=1.3.0" keras natsort tensorflow-hub matplotli imutils pandas tqdm scikit-learn seaborn psycopg2
```

