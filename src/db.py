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

        if not self.database:
            print('Cannot connect to DB!')
            exit(1)

        # DB cursor
        self.cursor = self.database.cursor()

        self.det_models = dict()

    def close(self) -> None:
        self.database.close()

    def fetch_models(self) -> None:

        # SQL query to fetch all ML models
        query = "SELECT detection_model.id, detection_model.name, detection_model.comment, " \
                "ml_model.id, ml_model.name, ml_model.comment, " \
                "dataset.id, dataset.name, dataset.comment " \
                "FROM detection_model " \
                "INNER JOIN dataset " \
                "ON detection_model.dataset_id = dataset.id " \
                "INNER JOIN ml_model " \
                "ON detection_model.ml_model_id = ml_model.id"

        # Execute the query
        self.cursor.execute(query)

        # Fetch the results
        results = self.cursor.fetchall()

        # Process the results
        for result in results:
            self.det_models[result[1]] = {
                'id':              result[0],
                'name':            result[1],
                'comment':         result[2],
                'model_id':        result[3],
                'model_name':      result[4],
                'model_comment':   result[5],
                'dataset_id':      result[6],
                'dataset_name':    result[7],
                'dataset_comment': result[8],
            }

    def get_models(self) -> list:
        return self.det_models

    def get_unprocessed_frames(self, detection_model_id) -> list:
        
        frame_ids = []

        # Gets all frames that have not been processed using the specific model
        query = f"SELECT frame.id FROM frame WHERE frame.id NOT IN " \
                f"(SELECT frame_id FROM frame_detection_table " \
                f"WHERE frame_detection_table.detection_model_id = {detection_model_id})"
        
        # Execute the query
        self.cursor.execute(query)

        # Fetch the results
        results = self.cursor.fetchall()

        # Process the results
        for result in results:
            frame_ids.append(result[0])

        return frame_ids

    def fetch_sensor_data(self, frame_id: int) -> dict:

        files = dict()

        query = f"SELECT frame_sensor.file_name, sensor_type.type_name FROM frame_sensor " \
                f"INNER JOIN sensor " \
                f"ON frame_sensor.sensor_id = sensor.id " \
                f"INNER JOIN sensor_type " \
                f"ON sensor.sensor_type_id = sensor_type.id " \
                f"WHERE frame_sensor.frame_id = {frame_id}"

        # Execute the query
        self.cursor.execute(query)

        # Fetch the results
        results = self.cursor.fetchall()

        # Process the results
        for result in results:
            file, sens_type = result

            if sens_type not in files.keys():
                files[sens_type] = []

            files[sens_type].append(file)

        return files