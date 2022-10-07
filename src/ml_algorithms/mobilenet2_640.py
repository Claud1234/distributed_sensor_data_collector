from src.ml_algorithms.ml_base import HubDetector

class Mobilenet_640_detector(HubDetector):
    def __init__(self, threshold: float) -> None:
        hub_url = 'https://tfhub.dev/tensorflow/ssd_mobilenet_v2/fpnlite_640x640/1'
        name = 'SSDMobilenetv2'
        dataset = 'COCO_17'
        label_file = 'assets/coco_labels.txt'
        model_dim = 640
        default_threshold = 0.6

        if threshold is not None and float(threshold) >= 0:
            upd_threshold = float(threshold)
        else:
            upd_threshold = default_threshold

        super().__init__(hub_url, name, dataset, label_file, model_dim, upd_threshold)
