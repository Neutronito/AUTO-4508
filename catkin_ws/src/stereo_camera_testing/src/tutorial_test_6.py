#!/usr/bin/env python3
import rospy
from stereo_camera_testing.msg import *

import os
import time
import datetime

from pathlib import Path
import depthai as dai
import cv2
import argparse
import numpy as np
import blobconverter
import json

DEFAULT_PATH = str((Path(__file__).parent / Path('../models/sigma.blob')).resolve().absolute())
CONFIG_PATH = str((Path(__file__).parent / Path('../config/sigma.json')).resolve().absolute())

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

print(metadata)

# parse labels
nnMappings = config.get("mappings", {})
labels = nnMappings.get("labels", {})

rospy.logerr(labels)


#log_file = open(f"log-{datetime.datetime.now().strftime('%H:%M:%S')}.csv", "a+")
#log_file.write("x, y, z, timestamp\n")


# Create pipeline
pipeline = dai.Pipeline()

nnPath = DEFAULT_PATH
if not Path(nnPath).exists():
    print("No blob found at {}. Looking into DepthAI model zoo.".format(nnPath))
    nnPath = str(blobconverter.from_zoo(DEFAULT_PATH, shaves = 13, zoo_type = "depthai", use_cache=True))

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
camRgb.setPreviewSize(320, 320)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
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

spatialDetectionNetwork.setBlobPath(nnPath)
spatialDetectionNetwork.setConfidenceThreshold(confidenceThreshold)
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
# spatialDetectionNetwork.setNumInferenceThreads(1)
# spatialDetectionNetwork.setNumNCEPerInferenceThread(2)

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

rospy.init_node("stereo_camera", anonymous=False)
pub = rospy.Publisher('stereo_camera_testing/objects', objects, queue_size=1)

# Connect to device and start pipeline
with dai.Device(pipeline, usb2Mode=True) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    previewQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    detectionNNQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)
    xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(name="boundingBoxDepthMapping", maxSize=4, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    networkQueue = device.getOutputQueue(name="nnNetwork", maxSize=4, blocking=False)

    startTime = time.monotonic()
    counter = 0
    fps = 0
    color = (255, 255, 255)
    printOutputLayersOnce = True

    while True:
        inPreview = previewQueue.get()

        mytime = dai.Clock.now()
        
        inDet = detectionNNQueue.get()
        depth = depthQueue.get()
        inNN = networkQueue.get()

        if printOutputLayersOnce:
            toPrint = 'Output layer names:'
            for ten in inNN.getAllLayerNames():
                toPrint = f'{toPrint} {ten},'
            print(toPrint)
            printOutputLayersOnce = False

        frame = inPreview.getCvFrame()
        depthFrame = depth.getFrame() # depthFrame values are in millimeters

        depthFrameColor = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthFrameColor = cv2.equalizeHist(depthFrameColor)
        depthFrameColor = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)

        counter+=1
        current_time = time.monotonic()
        if (current_time - startTime) > 1 :
            fps = counter / (current_time - startTime)
            counter = 0
            startTime = current_time

        detections = inDet.detections
        if len(detections) != 0:
            boundingBoxMapping = xoutBoundingBoxDepthMappingQueue.get()
            roiDatas = boundingBoxMapping.getConfigData()

            for roiData in roiDatas:
                roi = roiData.roi
                roi = roi.denormalize(depthFrameColor.shape[1], depthFrameColor.shape[0])
                topLeft = roi.topLeft()
                bottomRight = roi.bottomRight()
                xmin = int(topLeft.x)
                ymin = int(topLeft.y)
                xmax = int(bottomRight.x)
                ymax = int(bottomRight.y)

                cv2.rectangle(depthFrameColor, (xmin, ymin), (xmax, ymax), color, cv2.FONT_HERSHEY_SCRIPT_SIMPLEX)

        # found_cone = False
        # found_bucket = False
        # cone_info = [-1, -1, -1]
        # bucket_info = [-1, -1, -1]

        # detections = inDet.detections
        # if len(detections) != 0:
        #     for detection in detections:
        #         rospy.loginfo("TROLLOLOLOL")
        #         coords = detection.spatialCoordinates
        #         x, y, z = coords.x, coords.y, coords.z
        #         if detection.label == 1:
        #             found_cone = True
        #             cone_info = [x, y, z]
        #         elif detection.label == 0:
        #             found_bucket = True
        #             bucket_info = [x, y, z]

        # msg = objects()

        # msg.found_bucket = found_bucket
        # msg.found_cone = found_cone

        # msg.cone_x = int(cone_info[0])
        # msg.cone_y = int(cone_info[1])
        # msg.cone_z = int(cone_info[2])

        # msg.bucket_x = int(bucket_info[0])
        # msg.bucket_y= int(bucket_info[1])
        # msg.bucket_z = int(bucket_info[2])

        # pub.publish(msg)
        # rospy.Rate(0.5).sleep()


        # If the frame is available, draw bounding boxes on it and show the frame
        height = frame.shape[0]
        width  = frame.shape[1]
        iter = 0
        points = []
        for detection in detections:
            if detection.confidence < 0.5:
                continue

            # Denormalize bounding box
            x1 = int(detection.xmin * width)
            x2 = int(detection.xmax * width)
            y1 = int(detection.ymin * height)
            y2 = int(detection.ymax * height)

            label = labels[detection.label]
            
            iter+=1
            cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, "{:.2f}".format(detection.confidence*100), (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"X: {int(detection.spatialCoordinates.x)} mm", (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Y: {int(detection.spatialCoordinates.y)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
            cv2.putText(frame, f"Z: {int(detection.spatialCoordinates.z)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

            # print("Cone #" + str(iter) + " | " + "{:.2f}".format(detection.confidence*100))
            # print('x:' + str(int(detection.spatialCoordinates.x)) + 'mm')
            # print('y:' + str(int(detection.spatialCoordinates.y)) + 'mm')
            # print('z:' + str(int(detection.spatialCoordinates.z)) + 'mm')
            #log_file.write(f"{int(detection.spatialCoordinates.x)}, {int(detection.spatialCoordinates.y)}, {int(detection.spatialCoordinates.z)}, {int(time.time_ns() / 1000000)}\n")
            
            points.append([(detection.spatialCoordinates.x / 1000), (detection.spatialCoordinates.z / 1000), (detection.spatialCoordinates.y / 1000)])
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)
        
        latency = (mytime - inPreview.getTimestamp()).total_seconds() * 1000
        cv2.putText(frame, "NN fps: {:.2f}".format(latency), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
        cv2.imshow("depth", depthFrameColor)
        cv2.imshow("rgb", frame)

        if cv2.waitKey(1) == ord('q'):
            # log_file.close()
            break
