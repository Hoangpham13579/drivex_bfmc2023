#!/usr/bin/env python3

# Imports
from functools import partial
from typing import Any

import cv2
import numpy as np
import rospy
import yaml
from sensor_msgs.msg._Image import Image
from std_msgs.msg import Float32, String
from cv_bridge.core import CvBridge
import torch
from torchvision import transforms
import pathlib
import os

# Custom imports
from models.cnn_nvidia import Nvidia_Model
from models.cnn_rota import Rota_Model
from models.mobilenetv2 import MobileNetV2
from models.inceptionV3 import InceptionV3
from models.vgg import MyVGG
from models.resnet import ResNet
from models.lstm import LSTM
from models.resnet_imported import ResNetV1
from models.transformer import MyViT
from src.utils import LoadModel

os.environ["TF_CPP_MIN_LOG_LEVEL"] = "3"  # or any {'0', '1', '2'}


def preProcess(img):
    # Define Region of interest
    # img = img[60:135, :, :]
    img = cv2.cvtColor(img, cv2.COLOR_RGB2YUV)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    # img = img[40:, :]  #cut the 40 first lines
    img = cv2.resize(img, (256, 256))
    img = img / 255
    return img


def imgRgbCallback(message, config):
    config["img_rgb"] = config["bridge"].imgmsg_to_cv2(message, "passthrough")

    config["begin_img"] = True


def modelSteeringCallback(message, config):
    model_name = message.data
    drivexdriving_path = os.environ.get("DRIVEX_DRIVING")
    path = f"{drivexdriving_path}/models/{model_name}/{model_name}.pkl"
    # Retrieving info from yaml
    with open(f"{drivexdriving_path}/models/{model_name}/{model_name}.yaml") as file:
        info_loaded = yaml.load(file, Loader=yaml.FullLoader)

    rospy.loginfo("Using model: %s", path)
    device = f"cuda:0" if torch.cuda.is_available() else "cpu"  # cuda: 0 index of gpu
    model = eval(info_loaded["model"]["ml_arch"]["name"])
    model = LoadModel(path, model, device)
    model.eval()
    config["model"] = model

def main():
    config: dict[str, Any] = dict(vel=None, img_rgb=None, bridge=None, begin_img=None)
    # Defining starting values
    config["begin_img"] = False
    config["vel"] = 0
    config["bridge"] = CvBridge()

    # Init Node
    rospy.init_node("ml_driving", anonymous=False)

    # Getting parameters
    image_raw_topic = rospy.get_param(
        "~image_raw_topic", "/bottom_front_camera/image_raw"
    )
    model_steering_topic = rospy.get_param("~model_steering_topic", "/model_steering")
    model_name = rospy.get_param("/model_name", "")

    # Defining path to model
    drivexdriving_path = os.environ.get("DRIVEX_DRIVING")
    path = f"{drivexdriving_path}/models/{model_name}/{model_name}.pkl"

    # Retrieving info from yaml
    with open(f"{drivexdriving_path}/models/{model_name}/{model_name}.yaml") as file:
        info_loaded = yaml.load(file, Loader=yaml.FullLoader)

    rospy.loginfo("Using model: %s", path)
    device = f"cuda:0" if torch.cuda.is_available() else "cpu"  # cuda: 0 index of gpu
    config["model"] = eval(info_loaded["model"]["ml_arch"]["name"])
    config["model"] = LoadModel(path, config["model"], device)
    config["model"].eval()

    PIL_to_Tensor = transforms.Compose([transforms.ToTensor()])

    # Partials
    imgRgbCallback_part = partial(imgRgbCallback, config=config)

    changeModelCallback = partial(modelSteeringCallback, config=config)

    # Subscribe and publish topics
    rospy.Subscriber(image_raw_topic, Image, imgRgbCallback_part)
    rospy.Subscriber("/set_model", String, changeModelCallback)
    model_steering_pub = rospy.Publisher(model_steering_topic, Float32, queue_size=10)

    # Frames per second
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        if config["begin_img"] is False:
            continue

        resized_img = preProcess(config["img_rgb"])

        # Predict angle
        image = np.array([resized_img])
        image = image[0, :, :, :]
        image = PIL_to_Tensor(image)
        image = image.unsqueeze(0)
        image = image.to(device, dtype=torch.float)
        label_t_predicted = config["model"].forward(image)
        steering = float(label_t_predicted)
        # Publish angle
        model_steering_pub.publish(steering)
        rate.sleep()


if __name__ == "__main__":
    main()