import cv2
import numpy as np

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
                           labels: np.array, velocities: np.array, threshold: float) -> np.array:
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

    for i, bbox in enumerate(boxes):

        if scores[i] > threshold:

            cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(
                bbox[2]), int(bbox[3])), (255, 255, 255), 2)

            cv2.putText(frame, str(f'{int(scores[i]*100)}%'), (int(bbox[0]), int(
                bbox[1])), 0, 5e-3 * 200, (0, 255, 0), 2)

            cv2.putText(frame, str(f'{int(velocities[i]*18/5)}km/h'), (int(bbox[0]), int(
                bbox[1]) + 22), 0, 5e-3 * 150, (0, 255, 0), 2)

            cv2.putText(frame, str(labels[i]), (int(bbox[0]), int(
                bbox[3] + 20)), 0, 5e-3 * 150, (0, 255, 0), 2)

    return frame


def visualize(image, labels, scores, boxes, threshold, radar_points, velocities):
    global first_frame

    radar_colors = [(255, 0, 255), (255, 255, 0)]

    # Draw model detections
    image = visualize_model_output(image, boxes, scores, labels, velocities, threshold)

    # Draw radar points
    for radar_point in radar_points:
        image = cv2.circle(image, radar_point.screen_coords,
                        radius=7, color=radar_colors[radar_point.radar_id], thickness=-1)

    show_image(image, 'preview', 1280, 720)

    first_frame = False


