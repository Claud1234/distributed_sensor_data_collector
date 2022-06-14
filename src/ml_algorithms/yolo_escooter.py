import numpy as np

import tensorflow as tf
import os
import cv2
import time
import glob
import math
import re
from PIL import Image
from matplotlib import pyplot as plt

import imutils as imutils
import numpy as np  # linear algebra
import pandas as pd  # data processing, CSV file I/O (e.g. pd.read_csv)
import tensorflow as tf  # machine learning
from tqdm import tqdm  # make your loops show a smart progress meter
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
import seaborn as sn
from sklearn.metrics import roc_curve, roc_auc_score

from keras.models import model_from_json

# from src.ml_algorithms.ml_base import KerasJSONDetector
from src.ml_algorithms.ml_base import DetectorBase

# class YOLOeScooterDetect(DetectorBase):
#     def __init__(self, threshold: float) -> None:

#         name = 'YOLOeScooterDetect'
#         dataset = 'COCO_17'
#         label_file = 'assets/coco_labels.txt'
#         model_dim = 640 # TODO: Not sure if this is correct

#         super().__init__(model_cfg, model_weights, name, dataset, label_file, model_dim, threshold)


class YOLOeScooterDetect(DetectorBase):
    def __init__(self, threshold: float) -> None:

        name = 'YOLOeScooterDetect'
        dataset = 'COCO_17'
        label_file = 'assets/coco_labels.txt'
        model_dim = 416

        super().__init__(name, dataset, label_file, model_dim, threshold)

        # load json and create model
        json_file = open("assets/escooter_detector_model.json", 'r')
        loaded_model_json = json_file.read()
        json_file.close()
        self.obj_detector = model_from_json(loaded_model_json)
        # load weights into new model
        self.obj_detector.load_weights("assets/escooter_detector_model.h5")
        print("Loaded model from disk")

        # evaluate loaded model on test data
        # loaded_model.compile(loss='binary_crossentropy', optimizer='rmsprop', metrics=['accuracy'])
        self.obj_detector.compile(
            optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

        # filepath for the dependent files; this will change based on the original file path
        self.yolomodel = {"config_path": "assets/yolo3.cfg",
                          "model_weights_path": "assets/yolov3.weights",
                          "coco_names": "assets/coco_labels.txt",
                          "confidence_threshold": 0.7,
                          "threshold": 0.3
                          }

        self.net = cv2.dnn.readNetFromDarknet(
            self.yolomodel["config_path"], self.yolomodel["model_weights_path"])
        self.labels = open(
            self.yolomodel["coco_names"]).read().strip().split("\n")

        np.random.seed(12345)
        self.layer_names = self.net.getLayerNames()
        self.layer_names = [self.layer_names[i-1]
                            for i in self.net.getUnconnectedOutLayers()]

    def enlarge_bounding_boxes(self, orig_bboxes):
        modf_bboxes = []
        for bbox in orig_bboxes:
            [x1, y1, w1, h1] = bbox
            [x2, y2, w2, h2] = [x1-w1, y1, 3*w1, h1+h1//4]
            if x2 < 0:
                x2 = 0
            if y2 < 0:
                y2 = 0

            modf_bboxes.append([x2, y2, w2, h2])
        return modf_bboxes

    def detect(self, frame: np.array):
        (H, W) = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(
            frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        # detect objects using object detection model
        detections_layer = self.net.forward(self.layer_names)

        boxes, confidences, classIDs, classNames = [], [], [], []

        for out in detections_layer:
            for detection in out:
                scores = detection[5:]
                classID = np.argmax(scores)

                if classID != 0:  # person
                    continue

                confidence = scores[classID]

                if confidence > self.yolomodel['confidence_threshold']:
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        idxs = cv2.dnn.NMSBoxes(
            boxes, confidences, self.yolomodel["confidence_threshold"], self.yolomodel["threshold"])

        nms_boxes = []
        nms_confidences = []
        nms_classIDs = []
        nms_classNames = []

        if len(idxs) > 0:
            for i in idxs.flatten():
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])

                # clr = [int(c) for c in bbox_colors[classIDs[i]]]
                # cv2.rectangle(frame, (x, y), (x+w, y+h), clr, 2)
                nms_boxes.append([x, y, w, h])
                nms_confidences.append(confidences[i])
                nms_classIDs.append(classIDs[i])
                nms_classNames.append('scooter')

        # print(nms_classIDs, nms_boxes, nms_confidences, nms_classNames)

        # Scooter detection
        enl_bboxes = self.enlarge_bounding_boxes(nms_boxes)
        x_frame = []
        predictions = []

        for i in range(len(enl_bboxes)):
            bbox = enl_bboxes[i]
            [x, y, w, h] = bbox
            print(x, y, x+w, y+h)
            extracted_frame = frame[y:y+h, x:x+w, :]
            img_arr = cv2.resize(extracted_frame, (160, 160))
            x_frame.append(img_arr/255.0)

        if x_frame != []:
            X = np.asarray(x_frame)
            # calculate confidence here
            predictions = np.rint(self.obj_detector.predict(X))

        scoot_IDs, scoot_boxes, scoot_confs, scoot_names = [], [], [], []
        print(predictions)
        for i in range(len(nms_boxes)):
            if predictions[i] != 0:
                scoot_boxes.append(nms_boxes[i])
                scoot_confs.append(predictions[i])
                scoot_IDs.append(nms_classIDs[i])
                scoot_names.append(nms_classNames[i])

        # return nms_classIDs, nms_boxes, nms_confidences, nms_classNames
        return scoot_IDs, scoot_boxes, scoot_confs, scoot_names
