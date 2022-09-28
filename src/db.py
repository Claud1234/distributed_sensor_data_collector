import datetime
from time import time
import mysql.connector


class DBHandler():
    def __init__(self, db_cfg: dict) -> None:
        
        # Initialialize DB connection
        self.database = mysql.connector.connect(
            host=db_cfg['db_host'],
            user=db_cfg['db_user'],
            passwd=db_cfg['db_pass'],
            port=db_cfg['db_port'],
            database=db_cfg['db_name']
        )

        # DB cursor
        self.cursor = self.database.cursor()

        # Data structure containing information
        # about all known sensors
        self.sensors = dict()
        self.sensors_by_topic = dict()

        self._read_sensor_data()

    def _read_sensor_data(self) -> None:
        """
        Reads sensor metadata from the DB
        and saves it into self.sensors dict
        """
        sensor_types = dict()

        # Build an SQL query
        query = f"SELECT * FROM sensor_type"

        # Execute query
        self.cursor.execute(query)

        # Fetch the results
        results = self.cursor.fetchall()

        for res in results:
            sensor_types[res[0]] = res[1]

        # Read the list of known sensors from the database
        # Build an SQL query
        query = f"SELECT * FROM sensor"

        # Execute query
        self.cursor.execute(query)

        # Fetch the results
        results = self.cursor.fetchall()

        # Build the sensor structure
        for res in results:
            sensor = {
                'id': res[0],
                'name': res[1],
                'json': res[2],
                'comment': res[4],
                'topic': res[5]
            }

            sensor_type = sensor_types.get(res[3], 'other')
            if sensor_type not in self.sensors:
                self.sensors[sensor_type] = []

            self.sensors[sensor_type].append(sensor)
            self.sensors_by_topic[res[5]] = res[0]

    def get_sensors(self) -> dict:
        """
        Returns the datas structure containing
        sensor data

        Returns:
            dict: self.sensors
        """
        return self.sensors

    def get_sensor_topics(self) -> dict:
        """
        Returns a data structure containing
        sensor topics, sorted by sensor type

        Returns:
            dict: Sensor topics, sorted by sensor type
        """
        sensor_topics = dict()

        for k, v in self.sensors.items():
            if k not in sensor_topics:
                sensor_topics[k] = []

            for sensor in v:
                sensor_topics[k].append(sensor.get('topic', None))

        return sensor_topics

    def save_frame(self, timestamp: float, sensor_data: list):

        dt = datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')
        
        # Store frame entry
        sql = "INSERT INTO frame (timestamp) VALUES (%s)"
        val = (dt,)

        try:
            self.cursor.execute(sql, val)
            self.database.commit()
        except:
            print("Error inserting frame to database")
            self.database.rollback()

        frame_id = self.cursor.getlastrowid()

        # Store frame_sensor table entries
        for sensor in sensor_data:
            topic = sensor.get('topic', None)
            if topic is None:
                print("Warning: sensor without topic detected!")
                continue

            s_file = sensor.get('file', None)
            s_data = sensor.get('data', None)

            if s_file is None and s_data is None:
                print(f"Warning: sensor {topic} has no data or file assigned to it!")
                continue

            sensor_id = self.sensors_by_topic.get(topic, None)
            if sensor_id is None:
                print("Warning: unknown sensor:", topic)
                continue
            
            sql = "INSERT INTO frame_sensor (frame_id, sensor_id, file_name, sensor_data) VALUES (%s, %s, %s, %s)"
            val = (frame_id, sensor_id, s_file, s_data)

            # try:
            self.cursor.execute(sql, val)
            self.database.commit()
            # except:
                # print("Error inserting frame_sensor to database")
                # self.database.rollback()

