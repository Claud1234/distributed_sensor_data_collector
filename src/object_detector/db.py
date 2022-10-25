import mysql.connector

class DBHandler():
    def __init__(self, db_cfg: dict) -> None:
        """
        Class for handling DB connecrions

        Args:
            db_cfg (dict): Details needed for connecting
                           to the DB (host, user, etc)
        """
        
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
        """
        Closes the currently open database
        """
        self.database.close()

    def fetch_models(self) -> None:
        """
        Fetches all known detection models from the DBs
        """

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
        """
        Returs the list of models. It is important
        to call the fetch_models method before calling
        this method.

        Returns:
            list: List of models in the database
        """
        return self.det_models

    def get_unprocessed_frames(self, detection_model_id) -> list:
        """
        Fetches a list of frames from the database that have
        not been processed using the specified detection model

        Args:
            detection_model_id (_type_): Unique ID of the detection models
                                         Frames that have not been processed
                                         with this model are returned.

        Returns:
            list: List of the ID fields of all frames that have not been
                  processed with detection specidied in the detection_model_id
                  parameter
        """


        frame_ids = []

        # Gets all frames that have not been processed using the specific model
        query = f"SELECT frame.id FROM frame WHERE frame.id NOT IN " \
                f"(SELECT frame_id FROM frame_detection_model " \
                f"WHERE frame_detection_model.detection_model_id = {detection_model_id})"
        
        # Execute the query
        self.cursor.execute(query)

        # Fetch the results
        results = self.cursor.fetchall()

        # Process the results
        for result in results:
            frame_ids.append(result[0])

        return frame_ids

    def fetch_sensor_data(self, frame_id: int) -> dict:
        """
        Fetches sensor all sensor data for frame with ID 
        frame_id from the database

        Args:
            frame_id (int): Unique ID of the frame to fetch data for

        Returns:
            dict: Dictionary containing all sensor data in the requested frame
                  
        """

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

    def fetch_classes(self, dataset_id: int) -> list:
        """
        Fetches a list of all known class names for the
        dataset specified in dataset_id

        Args:
            dataset_id (int): Unique ID of the dataset to 
            check return the known class names for

        Returns:
            list: List of class names (strings) for the 
            dataset with unique ID dataset_id
        """

        query = f"SELECT object_class.id, object_class.name FROM object_class " \
                f"INNER JOIN object_class_dataset " \
                f"ON object_class_dataset.object_class_id = object_class.id " \
                f"WHERE object_class_dataset.dataset_id = {dataset_id}"

        # Execute the query
        self.cursor.execute(query)

        # Fetch the results
        results = {res[1]:res[0] for res in self.cursor.fetchall()}

        return results

    def insert_class(self, dataset_id: int, class_num: int, class_name: str) -> None:
        """
        Inserts a new object class into the database

        Args:
            dataset_id (int): Unique ID of the dataset this object belongs to
            class_num (int): Numeric ID of the object in the dataset
            class_name (str): Name of the object
        """

        # Object class entry
        query = f"INSERT INTO object_class (name) VALUES (%s)"
        val = (class_name,)
        self.cursor.execute(query, val)

        self.database.commit()

        class_id = int(self.cursor.lastrowid)

        # object_class_dataset entry
        query = "INSERT INTO object_class_dataset (object_class_id, dataset_id, class_number) " \
                "VALUES (%s, %s, %s)"
        val = (int(class_id), int(dataset_id), int(class_num))
        self.cursor.execute(query, val)

        self.database.commit()

    def save_objs(self, frame_id: int, model_data: dict, threshold: float,
                  ids: list, labels: list, bboxes: list, 
                  velocities: list, scores: list) -> None:
        """
        Saves objects on a single frame into the database

        Args:
            frame_id (int): Unique ID of the frame table entry, representing the frame
                            where the objects was detected
            model_data (dict): Dictionary object describing the detection model used for
                               detecting the object
            threshold (float): Save only objects, which have higher confidence score than
                               this value
            ids (list): Object class IDs, as reported by the ML model (different than class id in the DB)
            labels (list): Labels (class names) of the object, as reported by the ML model
            bboxes (list): Bounding boxes of the objects
            velocities (list): Velocities of the objects
            scores (list): Confidence scores of the object
        """
        

        # Read a list of known object classes from the DB    
        known_classes = self.fetch_classes(model_data.get('dataset_id'))

        # Process all detected object
        for id, label, bbox, score, vel in zip(ids, labels, bboxes, scores, velocities):

            # We only want object that have large enough confidence score
            if score >= threshold:

                # If we don't know the class yet, add it to the DB
                if label not in known_classes.keys():
                    self.insert_class(model_data.get('dataset_id'), id, label)
                    known_classes = self.fetch_classes(model_data.get('dataset_id'))

                # TODO: Add value for distance
                # Insert the object into the database
                query = "INSERT INTO object " \
                        "(detection_model_id, class_id, frame_id, bounding_box, speed, distance, confidence) " \
                        f"VALUES ({model_data.get('id')}, " \
                        f"{known_classes.get(label)}, " \
                        f"{frame_id}, " \
                        f"ST_GeomFromText('MULTIPOINT({bbox[0]} {bbox[1]}, {bbox[2]} {bbox[3]})'), " \
                        f"{vel}, " \
                        f"NULL, " \
                        f"{score})"
              
                
                self.cursor.execute(query)
                self.database.commit()

    def mark_frame_processed(self, frame_id: int, det_model_id: int) -> None:
        """
        Marks frame with ID frame_id to be processed with detection model with
        ID det_model_id in the database 

        Args:
            frame_id (int): Unique ID of the frame to mark processed
            det_model_id (int): Unique ID of the detection model it 
            was processed with
        """


    # Mark that the frame has been processed using the current model
        query = "INSERT INTO frame_detection_model " \
                "(frame_id, detection_model_id) " \
                f"VALUES ({frame_id}, " \
                f"{det_model_id})"
        
        self.cursor.execute(query)
        self.database.commit()
