import cv2
import numpy as np
import datetime
from utils.radar import read_radar_points

first_frame = True


def show_image(image: np.array, window_name: str, width: int = None, height: int = None,
               full_screen: bool = False) -> None:
    """
    Shows an image in a resizable window
    Parameters
    ----------
    image: OpenCV image to show
    window_name: Name of the window
    width: Window's width at creation time (only used if full_screen is False)
    height: Window's height at creation time (only used if full_screen is False)

    full_screen: (bool) If set to True (default), the window will be shown in full screen,
                        otherwise a normal window will be displayed

    Returns
    -------
    None
    """

    global first_frame

    # Show image
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    if full_screen:
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    elif width is None or height is None:
        raise Exception('Windows dimensions not specified!')

    cv2.imshow(window_name, image)

    if first_frame and not full_screen:
        cv2.resizeWindow(window_name, width, height)


def visualize_model_output(frame: np.array, boxes: np.array, scores: np.array,
                           labels: np.array, velocities: np.array, frame_metadata: str) -> np.array:
    """
    Draws objects detected by the model on the frame
    Parameters
    ----------
    frame: OpenCV frame to draw on
    boxes: Bounding boxes from the model
    scores: Confidence scores of the objects
    labels: Object labels
    velocities: Velocity values of objects measured by radar
    threshold: Do not draw objects that have confidence value under this threshold
    Returns
    -------
    Frame with annotations
    """

    y_coord = 21
    for metadata in frame_metadata:
        cv2.putText(frame, metadata, (10, y_coord), 0, 5e-3 * 150, (0, 255, 255), 2)
        y_coord += 30

    for i, bbox in enumerate(boxes):

        cv2.rectangle(frame, (int(bbox[0][0]), int(bbox[0][1])), (int(
            bbox[1][0]), int(bbox[1][1])), (255, 255, 255), 2)

        cv2.putText(frame, str(f'{int(velocities[i]*18/5)}km/h'), (int(bbox[1][0]), int(
            bbox[0][1]) + 22), 0, 5e-3 * 150, (0, 255, 255), 2)

        cv2.putText(frame, str(labels[i]), (int(bbox[1][0]), int(
            bbox[1][1] - 5)), 0, 5e-3 * 150, (0, 255, 255), 2)

    return frame

def visualize(db_objects):
    global first_frame

    img_file = db_objects[0].image_file
    radar_files = db_objects[0].radar_files

    frame_id = db_objects[0].frame_id

    time_str = db_objects[0].timestamp

    print(f"Image file: {img_file}")
    print(f"Radar files:")
    for radar_file in radar_files:
        print(f"\t{radar_file}")

    boxes = []
    scores = []
    labels = []
    velocities = []

    frame_metadata = [f'Frame ID: {frame_id}',
                     f'Timestamp: {time_str}']

    for db_object in db_objects:
        boxes.append(db_object.bounding_box)
        scores.append(db_object.confidence)
        labels.append(db_object.class_name)
        velocities.append(db_object.speed)

    image = cv2.imread(img_file)

    # Draw model detections
    image = visualize_model_output(image, boxes, scores, labels, velocities, frame_metadata)

    radar_colors = [(255, 0, 255), (255, 255, 0)]

    for i, radar_data in enumerate(radar_files):
        radar_points = read_radar_points(radar_data)

        # Draw radar points
        for radar_point in radar_points:
            image = cv2.circle(image, radar_point.screen_coords,
                            radius=7, color=radar_colors[i], thickness=-1)

    show_image(image, 'preview', 1280, 720)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    first_frame = False


