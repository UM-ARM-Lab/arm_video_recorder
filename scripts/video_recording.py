#! /usr/bin/env python
import cv2
import rospy
from std_msgs.msg import String
from arm_video_recorder.srv import TriggerVideoRecording, TriggerVideoRecordingResponse
from threading import Lock
import time
import sys


class VideoRecorder:
    def __init__(self, cap, path_prefix):
        self.out_filename = None
        self.lock = Lock()
        self.is_recording = False
        self.s = rospy.Service('video_recorder', TriggerVideoRecording, self.srv_trigger_recording_callback)

        self.out = None
        self.cap = cap
        self.record_start_time = time.time()
        self.record_time_limit = 0
        self.path_prefix = path_prefix

    def stop_current_recording(self):
        if self.out is not None:
            self.out.release()
        if self.is_recording:
            rospy.loginfo("Stopping current recording")
        self.is_recording = False

    def start_new_recording(self, filename):
        fourcc_code = None
        if filename[-4:] == ".mp4":
            fourcc_code = 0x00000021
        elif filename[-4:] == ".avi":
            fourcc_code = int(cap.get(cv2.CAP_PROP_FOURCC))
        else:
            rospy.logerr("Invalid file type " + filename[-4:])
            return False
            
        self.is_recording = True
        self.record_start_time = time.time()
        rospy.loginfo("Starting recording for " + filename)
        
        frame_dims = (int(self.cap.get(3)), int(self.cap.get(4)))

        self.out = cv2.VideoWriter(self.path_prefix + filename,
                                   fourcc_code,
                                   30,
                                   frame_dims)
        return True

    def srv_trigger_recording_callback(self, req):    
        resp = TriggerVideoRecordingResponse()
        resp.success = True
        with self.lock:
            self.stop_current_recording()
            self.record_time_limit = req.timeout_in_sec
            if(req.record):
                if not self.start_new_recording(req.filename):
                    resp.success = False
            
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



def live_view(cap):
    print "Frame dims: ", cap.get(3), ", ", cap.get(4)
    print "FourCC code: ", cap.get(cv2.CAP_PROP_FOURCC)
    while(True):
        ret, frame = cap.read()
        cv2.imshow('frame',frame)
 
        # Press Q on keyboard to stop recording
        if cv2.waitKey(1) & 0xFF == ord('q'):
            exit()


if __name__== "__main__":
    rospy.init_node("video_recorder")

    cap = cv2.VideoCapture(0)  #0 captures from webcam
    # cap = cv2.VideoCapture("sample_video.mp4")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 5000)  #Sets the camera to the maximal resolution
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 5000)  # up to 5000 x 5000

    if(not cap.isOpened()):
        rospy.logerr("Error opening video stream or file")
        exit()

        
    rospy.loginfo("Video Recorder ready")
    prefix = ""
    if len(sys.argv) < 4:
        rospy.loginfo("No absolution path provided. Saving files to relative path")
    else:
        prefix = sys.argv[1]
        rospy.loginfo("Recording to " + prefix)


    vr = VideoRecorder(cap, prefix)

    while not rospy.is_shutdown():
        vr.work_in_loop()
        rospy.sleep(0.01) # Don't eat up 100%CPU in the while loop


    cap.release()



