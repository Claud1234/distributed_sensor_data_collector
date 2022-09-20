import mysql.connector


class DBHandler():
    def __init__(self, db_name: str, db_host: str, db_user: str,
                 db_passwd: str, db_port: int) -> None:

        # Initialialize DB connection
        self.database = mysql.connector.connect(
            host=db_host,
            user=db_user,
            passwd=db_passwd,
            port=db_port,
            database=db_name
        )

        # DB cursor
        self.cursor = self.database.cursor()

        # Data structure containing information
        # about all known sensors
        self.sensors = dict()

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
