#! /usr/bin/env python
import cv2
import rospy
from std_msgs.msg import String
from arm_video_recorder.srv import TriggerVideoRecording, TriggerVideoRecordingResponse
from threading import Lock
import time


class VideoRecorder:
    def __init__(self, cap):
        self.out_filename = None
        self.lock = Lock()
        self.is_recording = False
        self.s = rospy.Service('video_recorder', TriggerVideoRecording, self.srv_trigger_recording_callback)

        self.out = None
        self.cap = cap
        self.record_start_time = time.time()
        self.record_time_limit = 0

    def stop_current_recording(self):
        if self.out is not None:
            self.out.release()
        if self.is_recording:
            rospy.loginfo("Stopping current recording")
        self.is_recording = False

    def start_new_recording(self, filename):
        self.is_recording = True
        self.record_start_time = time.time()
        rospy.loginfo("Starting recording for " + filename)
        frame_dims = (int(self.cap.get(3)), int(self.cap.get(4)))
        self.out = cv2.VideoWriter(filename,
                                   0x00000021,
                                   30,
                                   frame_dims)

    def srv_trigger_recording_callback(self, req):
        resp = TriggerVideoRecordingResponse()
        resp.success = True
        with self.lock:
            self.stop_current_recording()
            self.record_time_limit = req.timeout_in_sec
            if(req.record):
                self.start_new_recording(req.filename)
            
        if(self.record_time_limit <= 0):
            resp.success = False
            rospy.logerr("Cannot record with a timeout of 0s")

        return resp

    def save_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logerr("Camera capture failed")
            self.stop_current_recording()
            return
        self.out.write(frame)

    def check_timeout(self):
        if time.time() - self.record_start_time > self.record_time_limit:
            rospy.loginfo("Record time limit of " + str(self.record_time_limit) + " has been reached")
            self.stop_current_recording()
            return False
        return True
        

    def work_in_loop(self):
        with self.lock:
            if self.is_recording and self.check_timeout():
                self.save_frame()
                return


            


if __name__== "__main__":
    rospy.init_node("video_recorder")

    cap = cv2.VideoCapture(0)  #0 captures from webcam
    # cap = cv2.VideoCapture("sample_video.mp4")

    if(not cap.isOpened()):
        rospy.logerr("Error opening video stream or file")
        exit()

        
    rospy.loginfo("Video Recorder ready")

    vr = VideoRecorder(cap)

    while not rospy.is_shutdown():
        vr.work_in_loop()
        rospy.sleep(0.01) # Don't eat up 100%CPU in the while loop


    cap.release()



