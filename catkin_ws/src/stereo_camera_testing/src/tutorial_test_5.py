#!/usr/bin/env python3
from depthai_sdk import Previews
from depthai_sdk.managers import PipelineManager, PreviewManager, BlobManager, NNetManager
import depthai as dai
import cv2
# import argparse
from pathlib import Path

# # parse arguments
# parser = argparse.ArgumentParser()
# parser.add_argument("-m", "--model", help="Provide model path for inference",
#                     default='yolov4_tiny_coco_416x416', type=str)
# parser.add_argument("-c", "--config", help="Provide config path for inference",
#                     default='json/yolov4-tiny.json', type=str)
# args = parser.parse_args()
# CONFIG_PATH = args.config

DEFAULT_PATH = str((Path(__file__).parent / Path('../models/coneslayer_openvino_2022.1_6shave.blob')).resolve().absolute())
CONFIG_PATH = str((Path(__file__).parent / Path('../config/coneslayer.json')).resolve().absolute())

# create blob, NN, and preview managers
if Path(DEFAULT_PATH).exists():
    # initialize blob manager with path to the blob
    bm = BlobManager(blobPath=DEFAULT_PATH)
else:
    # initialize blob manager with the name of the model otherwise
    bm = BlobManager(zooName=DEFAULT_PATH)

nm = NNetManager(nnFamily="YOLO", inputSize=4)
nm.readConfig(CONFIG_PATH)  # this will also parse the correct input size

pm = PipelineManager()
pm.createColorCam(previewSize=nm.inputSize, xout=True)

# create preview manager
# fpsHandler = FPSHandler()
pv = PreviewManager(display=[Previews.color.name])

# create NN with managers
nn = nm.createNN(pipeline=pm.pipeline, nodes=pm.nodes, source=Previews.color.name,
                 blobPath=bm.getBlob(shaves=6, openvinoVersion=pm.pipeline.getOpenVINOVersion(), zooType="depthai"))
pm.addNn(nn)

# initialize pipeline
with dai.Device(pm.pipeline) as device:
    # create outputs
    pv.createQueues(device)
    nm.createQueues(device)

    nnData = []

    while True:

        # parse outputs
        pv.prepareFrames()
        inNn = nm.outputQueue.tryGet()

        if inNn is not None:
            nnData = nm.decode(inNn)
            # count FPS
            # fpsHandler.tick("color")

        nm.draw(pv, nnData)
        pv.showFrames()

        if cv2.waitKey(1) == ord('q'):
            break
