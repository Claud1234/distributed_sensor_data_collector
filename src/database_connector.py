import time


def add_to_db(db_conn, image_file, radar_file, labels, bboxes, threshold, velocities, scores):

    # TODO: Replace with actual values
    gps_pos = (0.0, 0.0)

    timestamp = time.time_ns()
    is_processed = True
    distance = 0

    # Sensor_Data table
    cur = db_conn.cursor()
    cur.execute(f'''INSERT INTO "Sensor_Data" (image_file, radar_point_cloud)
                VALUES ('{image_file}', '{radar_file}');''')

    db_conn.commit()

    # Frame data
    cur = db_conn.cursor()
    cur.execute(f'''INSERT INTO "Frame"( "Sensor_Data", "Self_Speed", "GPS_Coords" , "Timestamp", "Processed" ) 
                   VALUES((SELECT "Id" FROM "Sensor_Data" WHERE "Sensor_Data"."image_file" = '{image_file}' ), 
                           1.0, 
                           '{{{gps_pos[0]}, {gps_pos[1]}}}',  
                           '{timestamp}', 
                           true);''')

    db_conn.commit()

    # Detected Objects
    for i, bbox in enumerate(bboxes):
        if scores[i] > threshold:

            cur = db_conn.cursor()
            cur.execute(f'''INSERT INTO "Detected_Objects"( "Class", "Speed", "Distance" , "Bounding_Box", "frame_id" ) 
                            VALUES('{labels[i]}', 
                                    {velocities[i]}, 
                                    {distance}, 
                                    '(({bbox[0]}, {bbox[1]}), ({bbox[2]}, {bbox[3]}))', 
                                    (SELECT "Id" FROM "Frame" WHERE "Frame"."Timestamp" = '{timestamp}' ));''')
            db_conn.commit()
