#! /bin/bash

conda create --name object_detector python=3.8
conda activate object_detector
pip install --upgrade pip
pip install -U "tensorflow>=2.5" "cryptography>=2.5" "greenlet>=0.3" "python-dateutil<3.0.0" "cffi>=1.1" "SQLAlchemy>=1.3.0" keras opencv-python argparse natsort tensorflow-hub mysql-connector
