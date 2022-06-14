import cv2
import numpy as np
import tensorflow as tf
import tensorflow_hub as hub

class DetectorBase:
    def __init__(self, name: str, dataset: str, label_file: str, model_dim: int, threshold: float) -> None:
        self.model_dim = model_dim
        self.threshold = float(threshold)
        self.name = name
        self.dataset = dataset

        # Read the class labels
        self.labels = dict()

        # Read class file
        with open(label_file, 'r') as lf:
            label_contents = lf.readlines()

            for i, label in enumerate(label_contents):
                self.labels[i] = label.strip()

    def detect(self, image: np.array):
        raise NotImplementedError("No machine learning model selected")

    def get_name(self):
        return self.name

    def get_dataset(self):
        return self.dataset

    def get_dim(self):
        return self.model_dim

    def get_threshold(self):
        return self.threshold

class HubDetector(DetectorBase):
    def __init__(self, hub_url: str, name: str, dataset: str, label_file: str, model_dim: int, threshold: float) -> None:
        super().__init__(name, dataset, label_file, model_dim, threshold)

        self.obj_detector = hub.load(hub_url)

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
            labels.append(self.labels[class_id - 1])

        return ids, boxes, scores, labels

# class KerasJSONDetector(DetectorBase):
#     def __init__(self, model_cfg: str, model_weights: str, name: str, dataset: str, label_file: str, model_dim: int, threshold: float) -> None:
#         super().__init__(name, dataset, label_file, model_dim, threshold)

#         # TODO: The parameters should not be hardcoded
#         model = {"config_path": model_cfg,
#                  "model_weights_path": model_weights,
#                  "coco_names": label_file,
#                  "confidence_threshold": 0.7,
#                  "threshold":0.3
#                 }

#         self.obj_detector = cv2.dnn.readNetFromDarknet(model["config_path"], model["model_weights_path"])


#     # def detect(self, image: np.array):
#     #     model_input = cv2.resize(image, (self.model_dim, self.model_dim))
#     #     input_tensor = tf.convert_to_tensor(model_input)
#     #     input_tensor = input_tensor[tf.newaxis, ...]

#     #     results = self.obj_detector(input_tensor)

#     #     scores = results["detection_scores"].numpy()[0]
#     #     ids = results["detection_classes"].numpy()[0].astype('int')
#     #     boxes = results["detection_boxes"].numpy()[0]
#     #     labels = []

#     #     for class_id in ids:
#     #         labels.append(self.labels[class_id - 1])

#     #     return ids, boxes, scores, labels
