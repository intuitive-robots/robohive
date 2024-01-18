from threading import Thread
import threading
import numpy as np
# from hardware_base import hardwareBase
from robohive.robot.hardware_base import hardwareBase

import argparse
import a0
import logging
import copy
import time
import datetime
import depthai as dai

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DAIThread(Thread):
    def __init__(self, device_MxId, **kwargs):
        Thread.__init__(self, daemon=True)

        pipeline = dai.Pipeline()

        camRgb = pipeline.create(dai.node.ColorCamera)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutRgb.setStreamName("rgb")
        camRgb.video.link(xoutRgb.input)

        self.device_info = dai.DeviceInfo(device_MxId)
        self.pipeline = pipeline

        self.frame = None
        self.timestamp = None
        self.should_stop = threading.Event()

    def run(self):
        with dai.Device(self.pipeline, self.device_info) as device:
            # Output queue will be used to get the rgb frames from the output defined above
            qRgb = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
            while not self.should_stop.is_set():
                inRgb = qRgb.get()
                if inRgb is not None:
                    self.frame = inRgb.getFrame()
                    self.timestamp = time.time()

class DepthAI(hardwareBase):
    def __init__(self, name, device_MxId, **kwargs):
        self.device_MxId = device_MxId

        self.timeout = 1 # in seconds

    def connect(self):
        self.thread = DAIThread(self.device_MxId)
        self.thread.start()

    def get_sensors(self):
        # get all data from all topics
        last_img = copy.deepcopy(self.thread.frame)
        while last_img is None:
            last_img = copy.deepcopy(self.thread.frame)
            time.sleep(0.1)
        return {'time':self.thread.timestamp, 'rgb': last_img}

    def apply_commands(self):
        return 0

    def close(self):
        self.thread.should_stop.set()
        self.thread.join()
        return True

    def okay(self):
        # if self.rgb_topic and (self.rgb_sub is None):
        #     print("WARNING: No subscriber found for topic: ", self.rgb_topic)
        #     return False

        # if self.d_topic and (self.d_sub is None):
        #     print("WARNING: No subscriber found for topic: ", self.d_topic)
        #     return False

        # if self.most_recent_pkt_ts is None:
        #     print("WARNING: No packets received yet from the realsense subscibers: ", self.rgb_topic, self.d_topic)
        #     return False
        # else:
        #     now = datetime.datetime.now(datetime.timezone.utc)
        #     okay_age_threshold = datetime.timedelta(seconds=self.timeout)
        #     time_delay = now - self.most_recent_pkt_ts
        #     if time_delay>okay_age_threshold:
        #         print("Significant signal delay: ", time_delay)
        #         return False

        return True

    def reset(self):
        return 0


# # Get inputs from user
# def get_args():
#     parser = argparse.ArgumentParser(description="DepthAI Client: Connects to realsense pub.\n"
#     "\nExample: python robot/hardware_realsense.py -r realsense_815412070341/color/image_raw -d realsense_815412070341/depth_uncolored/image_raw")

#     parser.add_argument("-r", "--rgb_topic",
#                         type=str,
#                         help="rgb_topic name of the camera",
#                         default="")
#     parser.add_argument("-d", "--d_topic",
#                         type=str,
#                         help="rgb_topic name of the camera",
#                         default="")
#     parser.add_argument("-v", "--view",
#                         type=None,
#                         help="Choice: CV2",
#                         )
#     return parser.parse_args()


if __name__ == "__main__":
    import cv2

    # args = get_args()
    # print(args)
    MXID = '1844301071E7AB1200'
    rs = DepthAI(name="test cam", device_MxId=MXID)
    rs.connect()

    for i in range(50):
        img = rs.get_sensors()
        if img['rgb'] is not None:
            print("Received image{} of size:".format(i), img['rgb'].shape, flush=True)
            cv2.imshow("rgb", img['rgb'])
            cv2.waitKey(1)

        if img['rgb'] is None:
            print(img)

        time.sleep(0.1)

    rs.close()
