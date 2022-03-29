import cv2
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub


class Detector:
    def __init__(self, model_url: str, model_dim: int, threshold: float, all_labels: dict) -> None:
        self.model_dim = model_dim
        self.threshold = float(threshold)
        self.all_labels = all_labels

        self.obj_detector = hub.load(model_url)

    def detect(self, image: np.array):
        model_input = cv2.resize(image, (self.model_dim, self.model_dim))
        input_tensor = tf.convert_to_tensor(model_input)
        input_tensor = input_tensor[tf.newaxis, ...]

        results = self.obj_detector(input_tensor)

        scores = results["detection_scores"].numpy()[0]
        ids = results["detection_classes"].numpy()[0].astype('int')
        boxes = results["detection_boxes"].numpy()[0]
        labels = []

        for class_id in ids:
            labels.append(self.all_labels[class_id - 1])

        return ids, boxes, scores, labels

    def get_threshold(self):
        return self.threshold
