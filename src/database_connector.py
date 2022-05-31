from email.mime import image
import time
import psycopg2
from src.ml_algorithms.ml_base import DetectorBase

def add_model_to_db(db_conn: psycopg2.extensions.connection, detector: DetectorBase, depth=0):

    # Detect if current model exists in the DB
    query = f''' SELECT "ID" 
                 FROM "Model" 
                 WHERE "Model"."ModelName" = '{detector.get_name()}'
                 AND "Model"."DatasetName" = '{detector.get_dataset()}'
                 AND "Model"."InputSize" = '{{{detector.get_dim()}, {detector.get_dim()}}}';
             '''

    insert_str = f''' INSERT INTO "Model"("ModelName", "DatasetName" , "InputSize") 
                      VALUES('{detector.get_name()}' , 
                             '{detector.get_dataset()}',  
                             '{{{detector.get_dim()}, {detector.get_dim()}}}');
                  '''

    cur = db_conn.cursor()

    cur.execute(query)
    rows = cur.fetchall()

    model_id = None

    # Infinite recursion is not good
    if depth > 2:
        return None

    if len(rows) == 0:
        print("Model not in DB, adding")
        cur.execute(insert_str)
        db_conn.commit()
        model_id = add_model_to_db(db_conn, detector, depth+1)
        return model_id

    elif len(rows) > 1:
        print("Error! Model Query returned more than one row! This should never happen")
        exit(1)
    else:
        model_id = rows[0][0]
        return model_id

def add_to_db(db_conn: psycopg2.extensions.connection, detector: DetectorBase, 
              image_file: str, radar_files: list, 
              labels: list, bboxes: list, threshold: float, 
              velocities: list, scores: list) -> None:

    # TODO: Replace with actual values
    gps_pos = (0.0, 0.0)

    timestamp = time.time_ns()
    is_processed = True
    distance = 0

    cur = db_conn.cursor()

    model_id = add_model_to_db(db_conn, detector)
  
    if model_id is None:
        print("ERROR! Adding model to the DB failed!")

    # Frame data
    cur.execute(f'''INSERT INTO "Frame"("SelfSpeed", "GpsCoords" , "Timestamp", "Processed" ) 
                   VALUES(1.0, 
                          '{{{gps_pos[0]}, {gps_pos[1]}}}',  
                          '{timestamp}', 
                          true);''')

    db_conn.commit()

    # # cur.execute(f'''INSERT INTO "Frame"("SelfSpeed", "GpsCoords" , "Timestamp", "Processed" ) 
    # #                VALUES((SELECT "Id" FROM "Sensor_Data" WHERE "Sensor_Data"."image_file" = '{image_file}' ), 
    # #                        1.0, 
    # #                        '{{{gps_pos[0]}, {gps_pos[1]}}}',  
    # #                        '{timestamp}', 
    # #                        true);''')

    # SensorData table image file entry
    cur.execute(f'''INSERT INTO "SensorData" ("SensorType", "DataFile", "FrameID")
                    VALUES ('image', '{image_file}', 
                    (SELECT "ID" FROM "Frame" WHERE "Frame"."Timestamp" = '{timestamp}' ));''')
    db_conn.commit()

    # SensorData table radar file entries
    for radar_file in radar_files:
        # TODO: Add information to the DB to identify the radar
        cur.execute(f'''INSERT INTO "SensorData" ("SensorType", "DataFile", "FrameID")
                      VALUES ('radar', '{radar_file}', 
                      (SELECT "ID" FROM "Frame" WHERE "Frame"."Timestamp" = '{timestamp}' ));''')

    db_conn.commit()

    # Detected Objects
    for i, bbox in enumerate(bboxes):
        if scores[i] > threshold:

            cur.execute(f'''INSERT INTO "DetectedObject"( "Class", "Speed", "Distance" , "BoundingBox", "FrameID", "ModelID" ) 
                            VALUES('{labels[i]}', 
                                    {velocities[i]}, 
                                    {distance}, 
                                    '(({bbox[0]}, {bbox[1]}), ({bbox[2]}, {bbox[3]}))', 
                                    (SELECT "ID" FROM "Frame" WHERE "Frame"."Timestamp" = '{timestamp}'),
                                    {model_id});''')
            db_conn.commit()
