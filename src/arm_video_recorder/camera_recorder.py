import rospy
from arm_video_recorder.srv import TriggerVideoRecording, TriggerVideoRecordingRequest
from datetime import datetime
from pathlib import Path


class CameraRecorder:

    def __init__(self, filename=None, timestamp_format='%Y_%m_%d_%H_%M_%S', timeout_seconds=600.0):
        self.save_file = filename
        self.timestamp_format = timestamp_format
        self.video_srv = rospy.ServiceProxy("/video_recorder", TriggerVideoRecording)
        self.timeout = timeout_seconds

    def __enter__(self):
        req = TriggerVideoRecordingRequest()
        f = Path(self.save_file)
        if self.timestamp_format is not None:
            f = f.parent / (f.stem + '_' + datetime.now().strftime(self.timestamp_format) + f.suffix)
        req.filename = f.as_posix()
        req.timeout_in_sec = self.timeout
        req.record = True
        resp = self.video_srv(req)
        if not resp.success:
            raise RuntimeError("Unable to start live camera recording")

    def __exit__(self, *args):
        req = TriggerVideoRecordingRequest()
        req.record = False
        resp = self.video_srv(req)
