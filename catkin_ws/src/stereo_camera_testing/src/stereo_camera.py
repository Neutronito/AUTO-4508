#!/usr/bin/env python3
import rospy
import shutil
from stereo_camera_testing.srv import *
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

import os
import time
import datetime
import math

import time

from pathlib import Path
import depthai as dai
import cv2
import argparse
import numpy as np
import blobconverter
import json

DEFAULT_PATH = str((Path(__file__).parent / Path('../models/garbo.blob')).resolve().absolute())
CONFIG_PATH = str((Path(__file__).parent / Path('../config/garbo.json')).resolve().absolute())

# X_CENTRE_MAX = 100
# X_CENTRE_MIN = -100
CONE_CONFIDENCE_THRESHOLD   = 0.7
BUCKET_CONFIDENCE_THRESHOLD = 0.6
MIN_DIST = 0.01

# parser = argparse.ArgumentParser()
# parser.add_argument("-m", "--model", help="Provide model name or model path for inference",
#                     default='coneslayer_openvino_2021.4_6shave.blob', type=str)
# parser.add_argument("-c", "--config", help="Provide config path for inference",
#                     default='json/coneslayer.json', type=str)
# args = parser.parse_args()

# parse config
configPath = Path(CONFIG_PATH)
if not configPath.exists():
    raise ValueError("Path {} does not exist!".format(configPath))

with configPath.open() as f:
    config = json.load(f)
nnConfig = config.get("nn_config", {})

# parse input shape
if "input_size" in nnConfig:
    W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

# extract metadata
metadata = nnConfig.get("NN_specific_metadata", {})
classes = metadata.get("classes", {})
coordinates = metadata.get("coordinates", {})
anchors = metadata.get("anchors", {})
anchorMasks = metadata.get("anchor_masks", {})
iouThreshold = metadata.get("iou_threshold", {})
confidenceThreshold = metadata.get("confidence_threshold", {})

# print(metadata)

# parse labels
nnMappings = config.get("mappings", {})
labels = nnMappings.get("labels", {})


#log_file = open(f"log-{datetime.datetime.now().strftime('%H:%M:%S')}.csv", "a+")
#log_file.write("x, y, z, timestamp\n")


# Create pipeline
pipeline = dai.Pipeline()

nnPath = DEFAULT_PATH
if not Path(nnPath).exists():
    print("No blob found at {}. Looking into DepthAI model zoo.".format(nnPath))
    nnPath = str(blobconverter.from_zoo(DEFAULT_PATH, shaves = 6, zoo_type = "depthai", use_cache=True))

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
nnNetworkOut = pipeline.create(dai.node.XLinkOut)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutNN = pipeline.create(dai.node.XLinkOut)
xoutBoundingBoxDepthMapping = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutNN.setStreamName("detections")
xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
xoutDepth.setStreamName("depth")
nnNetworkOut.setStreamName("nnNetwork")

# Properties
camRgb.setPreviewSize(416, 416)
camRgb.setFps(8)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P) # cant go lower
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# setting node configs
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
# Align depth map to the perspective of RGB camera, on which inference is done
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

rospy.loginfo("Stereo cam has following resolutions")
rospy.loginfo(monoLeft.getResolutionWidth())
rospy.loginfo(monoLeft.getResolutionHeight())

CONFIDENCE = 0.6

spatialDetectionNetwork.setBlobPath(nnPath)
spatialDetectionNetwork.setConfidenceThreshold(CONFIDENCE)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(11000)
spatialDetectionNetwork.setSpatialCalculationAlgorithm(dai.SpatialLocationCalculatorAlgorithm(1))

# Yolo specific parameters
spatialDetectionNetwork.setNumClasses(classes)
spatialDetectionNetwork.setCoordinateSize(coordinates)
spatialDetectionNetwork.setAnchors(anchors)
spatialDetectionNetwork.setAnchorMasks(anchorMasks)
spatialDetectionNetwork.setIouThreshold(iouThreshold)
spatialDetectionNetwork.setNumInferenceThreads(2)

# Linking
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)

camRgb.preview.link(spatialDetectionNetwork.input)
spatialDetectionNetwork.passthrough.link(xoutRgb.input)


spatialDetectionNetwork.out.link(xoutNN.input)
spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

stereo.depth.link(spatialDetectionNetwork.inputDepth)
spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

device = dai.Device(pipeline, usb2Mode=True)

qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)

previewQueue = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
detectionNNQueue = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=1, blocking=False)
depthQueue = device.getOutputQueue(name="depth", maxSize=1, blocking=False)
networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=1, blocking=False)

def depth_convert_multiplier(input):
    return (0.9524 - 0.04*0.001*input)


def detect_objects(*args):
    if detectionNNQueue.isClosed():
        rospy.logerr("DETECTION QUEUE IS CLOSED ON THE STEREO CAMERA THIS IS BAD NEWS")

    # Get data from the queues
    inPreview = previewQueue.tryGet()
    inDet = detectionNNQueue.tryGet()
    depth = depthQueue.get()
    inNN = networkQueue.get()
    
    # Initiate findings to be false
    found_cone = False
    found_bucket = False
    cone_info = [-1, -1, -1]
    bucket_info = [-1, -1, -1]

    cone_conf   = 0
    bucket_conf = 0

    # Set cone and bucket distances to large numbers
    cone_dist = 2**17
    bucket_dist = 2**17

    # # debuggy
    # frame = inPreview.getCvFrame()
    # height = frame.shape[0]
    # width  = frame.shape[1]

    # detections = inDet.detections
    # for detection in detections:
    #     # Denormalize bounding box
    #     x1 = int(detection.xmin * width)
    #     x2 = int(detection.xmax * width)
    #     y1 = int(detection.ymin * height)
    #     y2 = int(detection.ymax * height)

    #     label = labels[detection.label]
        
    #     iter+=1
    #     cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #     cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #     cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #     cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #     cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

    # cv2.imshow("rgb", frame)

    # Fetch object labels from the detection queue
    if inDet != None:
        detections = inDet.detections

        # Count number of objects found
        counts = [0] * len(labels)
        for detection in detections:
            index = detection.label
            counts[index] += 1

        # Log the number of objects detected
        rospy.loginfo("Detected " + str(len(detections)) + " objects of interest!")
        if len(detections) > 0:
            rospy.loginfo(f"Objects are: {labels[0]} x{counts[0]}, {labels[1]} x{counts[1]}, {labels[2]} x{counts[2]}, {labels[3]} x{counts[3]},")



        for detection in detections:
            coords = detection.spatialCoordinates
            x, y, z = coords.x, coords.y, coords.z
            if (x < MIN_DIST and y < MIN_DIST and z < MIN_DIST):
                continue

            z = depth_convert_multiplier(z) * z
            dist = math.sqrt(x**2 + y**2 + z**2)

            width = 416

            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            x = (x1 + x2) / 2 - (width/2)

            if detection.label == 3 and detection.confidence > CONE_CONFIDENCE_THRESHOLD and dist < cone_dist:
                found_cone = True
                cone_info = [x, y, z]
                cone_dist = dist
                cone_conf = detection.confidence
            elif detection.confidence > BUCKET_CONFIDENCE_THRESHOLD and dist < bucket_dist:
                found_bucket = True
                bucket_info = [x, y, z]
                bucket_dist = dist
                bucket_conf = detection.confidence

        if (cone_conf > 0):
            rospy.loginfo(f"Cone confidence is {cone_conf}")
        if (bucket_conf > 0):
            rospy.loginfo(f"Bucket confidence is {bucket_conf}")
        
    else:
        rospy.loginfo("Polling the stereo camera failed, there is no image to analyse in the queue!")
    
    # Create a service response
    service_response = {}
    service_response["found_cone"] = found_cone
    service_response["found_bucket"] = found_bucket

    service_response["cone_x"] = int(cone_info[0])
    service_response["cone_y"] = int(cone_info[1])
    service_response["cone_z"] = int(cone_info[2])

    service_response["bucket_x"] = int(bucket_info[0])
    service_response["bucket_y"] = int(bucket_info[1])
    service_response["bucket_z"] = int(bucket_info[2])
    return service_response

# Make sure the destination path is present before starting to store the examples
# dirName = "./images"

dirName = "../catkin_dependencies/photos"
shutil.rmtree(dirName)
os.mkdir(dirName)
print(dirName)

def take_image_with_joystick(joy):
    if joy.buttons[2] == 1:
        rospy.loginfo("BUtton Pressed")
        take_image(True)
    return

def take_image(data):
    # if (joy.buttons[2] == 1):
    #     rospy.loginfo("Button Pressed")
    inPreview = qRgb.tryGet()
    inDet = detectionNNQueue.tryGet()

    if inPreview != None and inDet != None:
        frame = inPreview.getCvFrame()
        detections = inDet.detections

        for detection in detections:
        # Denormalize bounding box
            width = 416
            height = 416
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)

            label = labels[detection.label]
            
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        
        # print(image_data)
        fName = f"{dirName}/{int(time.time() * 1000)}.jpg"

        cv2.imwrite(fName, frame)
        # with open(fName, "wb") as f:
        #     f.write(image_data)
        rospy.loginfo("Image has been saved to:")
        rospy.loginfo(fName)
    else:
        rospy.logerr("There was no image in the queue! This is indicative of an ERROR!")
            # rospy.loginfo('Image saved to', str(fName))



# Initiate the ros node
rospy.init_node("stereo_camera_node", anonymous=False)
rospy.Subscriber("joy", Joy, take_image_with_joystick)
object_detection = rospy.Service("stereo_camera_testing/object_locations", object_locations, detect_objects)
# take_image = rospy.Service("stereo_camera_testing/take_image", object_locations, detect_objects)
rospy.Subscriber("stereo_camera_node/image_request", Bool, take_image)
rospy.loginfo("Stereo Camera has finished init sequence")
rospy.spin()

## ORIGINAL CODE


# # Connect to device and start pipeline
# try:
#     device = dai.Device(pipeline, usb2Mode=True)

#     # Output queues will be used to get the rgb frames and nn data from the outputs defined above
#     # previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
#     detectionNNQueue = device.getOutputQueue(name="detections", maxSize=1, blocking=False)
#     # xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
#     # depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
#     # networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=1, blocking=False)
# except Exception as e:
#     rospy.logerr(e)
#     rospy.signal_shutdown("No good reason but we are likely getting issues with starting the camera")


# def process_camera_for_objects():
#     # inPreview = previewQueue.get()
#     inDet = detectionNNQueue.tryGet()
#     # depth = depthQueue.get()
#     # inNN = networkQueue.get()

#     # if printOutputLayersOnce:
#     #     toPrint = 'Output layer names:'
#     #     for ten in inNN.getAllLayerNames():
#     #         toPrint = f'{toPrint} {ten},'
#     #     print(toPrint)
#     #     printOutputLayersOnce = False

#     # frame = inPreview.getCvFrame()
#     # depthFrame = depth.getFrame() # depthFrame values are in millimeters

#     # depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
#     # depthFrameColor = cv2.equalizeHist(depthFrameColor)
#     # depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
#     # If the frame is available, draw bounding boxes on it and show the frame
#     previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
#     inPreview = previewQueue.get()
#     frame = inPreview.getCvFrame()
#     height = frame.shape[0]
#     width  = frame.shape[1]
#     iter = 0
#     points = []

#     detections = inDet.detections
#     for detection in detections:
#         # Denormalize bounding box
#         x1 = int(detection.xmin * width)
#         x2 = int(detection.xmax * width)
#         y1 = int(detection.ymin * height)
#         y2 = int(detection.ymax * height)

#         label = labels[detection.label]
        
#         iter+=1
#         cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
#         cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
#         cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
#         cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
#         cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

#     cv2.imshow("rgb", frame)

#     found_cone = False
#     found_bucket = False
#     cone_info = [-1, -1, -1]
#     bucket_info = [-1, -1, -1]

#     if inDet != None:
#         rospy.loginfo("Found " + str(len(detections)) + " objects of interest!")
#         for detection in detections:
#             coords = detection.spatialCoordinates
#             x, y, z = coords.x, coords.y, coords.z

#             if detection.label == 1:
#                 found_cone = True
#                 cone_info = [x, y, z]
#             elif detection.label == 0:
#                 found_bucket = True
#                 bucket_info = [x, y, z]
#     else:
#         rospy.loginfo("Tryget from queue failed!")

#     return found_cone, found_bucket, cone_info, bucket_info

# def detect_objects(*args):
#     found_cone, found_bucket, cone_info, bucket_info = process_camera_for_objects()
#     service_response = {}
#     service_response["found_cone"] = found_cone
#     service_response["found_bucket"] = found_bucket

#     service_response["cone_x"] = int(cone_info[0])
#     service_response["cone_y"] = int(cone_info[1])
#     service_response["cone_z"] = int(cone_info[2])

#     service_response["bucket_x"] = int(bucket_info[0])
#     service_response["bucket_y"] = int(bucket_info[1])
#     service_response["bucket_z"] = int(bucket_info[2])
#     return service_response


# def init_node():
#     rospy.init_node("stereo_camera", anonymous=False)
#     object_detection = rospy.Service("stereo_camera_testing/object_locations", object_locations, detect_objects)
#     rospy.loginfo("Stereo Camera has finished init sequence")
#     rospy.spin()


# if __name__ == "__main__":
#     init_node()


############
#THIS WORKS#
############

# #!/usr/bin/env python3
# import rospy
# from stereo_camera_testing.msg import *

# import os
# import time
# import datetime

# from pathlib import Path
# import depthai as dai
# import cv2
# import argparse
# import numpy as np
# import blobconverter
# import json

# DEFAULT_PATH = str((Path(__file__).parent / Path('../models/best_openvino_2022.1_6shave.blob')).resolve().absolute())
# CONFIG_PATH = str((Path(__file__).parent / Path('../config/best.json')).resolve().absolute())

# # parser = argparse.ArgumentParser()
# # parser.add_argument("-m", "--model", help="Provide model name or model path for inference",
# #                     default='coneslayer_openvino_2021.4_6shave.blob', type=str)
# # parser.add_argument("-c", "--config", help="Provide config path for inference",
# #                     default='json/coneslayer.json', type=str)
# # args = parser.parse_args()

# # parse config
# configPath = Path(CONFIG_PATH)
# if not configPath.exists():
#     raise ValueError("Path {} does not exist!".format(configPath))

# with configPath.open() as f:
#     config = json.load(f)
# nnConfig = config.get("nn_config", {})

# # parse input shape
# if "input_size" in nnConfig:
#     W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

# # extract metadata
# metadata = nnConfig.get("NN_specific_metadata", {})
# classes = metadata.get("classes", {})
# coordinates = metadata.get("coordinates", {})
# anchors = metadata.get("anchors", {})
# anchorMasks = metadata.get("anchor_masks", {})
# iouThreshold = metadata.get("iou_threshold", {})
# confidenceThreshold = metadata.get("confidence_threshold", {})

# print(metadata)

# # parse labels
# nnMappings = config.get("mappings", {})
# labels = nnMappings.get("labels", {})

# rospy.logerr(labels)


# #log_file = open(f"log-{datetime.datetime.now().strftime('%H:%M:%S')}.csv", "a+")
# #log_file.write("x, y, z, timestamp\n")


# # Create pipeline
# pipeline = dai.Pipeline()

# nnPath = DEFAULT_PATH
# if not Path(nnPath).exists():
#     print("No blob found at {}. Looking into DepthAI model zoo.".format(nnPath))
#     nnPath = str(blobconverter.from_zoo(DEFAULT_PATH, shaves = 6, zoo_type = "depthai", use_cache=True))

# # Define sources and outputs
# camRgb = pipeline.create(dai.node.ColorCamera)
# spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
# monoLeft = pipeline.create(dai.node.MonoCamera)
# monoRight = pipeline.create(dai.node.MonoCamera)
# stereo = pipeline.create(dai.node.StereoDepth)
# nnNetworkOut = pipeline.create(dai.node.XLinkOut)

# xoutRgb = pipeline.create(dai.node.XLinkOut)
# xoutNN = pipeline.create(dai.node.XLinkOut)
# xoutBoundingBoxDepthMapping = pipeline.create(dai.node.XLinkOut)
# xoutDepth = pipeline.create(dai.node.XLinkOut)

# xoutRgb.setStreamName("rgb")
# xoutNN.setStreamName("detections")
# xoutBoundingBoxDepthMapping.setStreamName("boundingBoxDepthMapping")
# xoutDepth.setStreamName("depth")
# nnNetworkOut.setStreamName("nnNetwork")

# # Properties
# camRgb.setPreviewSize(640, 416)
# camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
# camRgb.setInterleaved(False)
# camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

# monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
# monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
# monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
# monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# # setting node configs
# stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
# # Align depth map to the perspective of RGB camera, on which inference is done
# stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
# stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

# spatialDetectionNetwork.setBlobPath(nnPath)
# spatialDetectionNetwork.setConfidenceThreshold(confidenceThreshold)
# spatialDetectionNetwork.input.setBlocking(False)
# spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
# spatialDetectionNetwork.setDepthLowerThreshold(100)
# spatialDetectionNetwork.setDepthUpperThreshold(11000)
# spatialDetectionNetwork.setSpatialCalculationAlgorithm(dai.SpatialLocationCalculatorAlgorithm(1))

# # Yolo specific parameters
# spatialDetectionNetwork.setNumClasses(classes)
# spatialDetectionNetwork.setCoordinateSize(coordinates)
# spatialDetectionNetwork.setAnchors(anchors)
# spatialDetectionNetwork.setAnchorMasks(anchorMasks)
# spatialDetectionNetwork.setIouThreshold(iouThreshold)
# spatialDetectionNetwork.setNumInferenceThreads(2)

# # Linking
# monoLeft.out.link(stereo.left)
# monoRight.out.link(stereo.right)

# camRgb.preview.link(spatialDetectionNetwork.input)
# spatialDetectionNetwork.passthrough.link(xoutRgb.input)


# spatialDetectionNetwork.out.link(xoutNN.input)
# spatialDetectionNetwork.boundingBoxMapping.link(xoutBoundingBoxDepthMapping.input)

# stereo.depth.link(spatialDetectionNetwork.inputDepth)
# spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)
# spatialDetectionNetwork.outNetwork.link(nnNetworkOut.input)

# rospy.init_node("stereo_camera", anonymous=False)
# pub = rospy.Publisher('stereo_camera_testing/objects', objects, queue_size=1)

# # Connect to device and start pipeline
# with dai.Device(pipeline, usb2Mode=True) as device:

#     # Output queues will be used to get the rgb frames and nn data from the outputs defined above
#     previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
#     detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
#     xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
#     depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
#     networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False)

#     color = (255, 255, 255)
#     printOutputLayersOnce = True

#     while True:
#         inDet = detectionNNQueue.get()
#         depth = depthQueue.get()
#         inNN = networkQueue.get()

#         found_cone = False
#         found_bucket = False
#         cone_info = [-1, -1, -1]
#         bucket_info = [-1, -1, -1]

#         detections = inDet.detections
#         if len(detections) != 0:
#             for detection in detections:
#                 rospy.loginfo("TROLLOLOLOL")
#                 coords = detection.spatialCoordinates
#                 x, y, z = coords.x, coords.y, coords.z
#                 if detection.label == 1:
#                     found_cone = True
#                     cone_info = [x, y, z]
#                 elif detection.label == 0:
#                     found_bucket = True
#                     bucket_info = [x, y, z]

#         msg = objects()

#         msg.found_bucket = found_bucket
#         msg.found_cone = found_cone

#         msg.cone_x = int(cone_info[0])
#         msg.cone_y = int(cone_info[1])
#         msg.cone_z = int(cone_info[2])

#         msg.bucket_x = int(bucket_info[0])
#         msg.bucket_y= int(bucket_info[1])
#         msg.bucket_z = int(bucket_info[2])

#         pub.publish(msg)