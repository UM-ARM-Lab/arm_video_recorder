#! /usr/bin/env python
import cv2
import rospy
from std_msgs.msg import String
from arm_video_recorder.srv import TriggerVideoRecording, TriggerVideoRecordingResponse
from threading import Lock

record_srv_cmd = False
start_new_recording = False
stop_current_recording = False
out_filename = None

lock = Lock()
        
def video_service_callback(req):
    global start_new_recording
    global stop_current_recording
    global out_filename
    global lock

    with lock:
        out_filename = req.filename
        
        start_new_recording = req.record
        stop_current_recording = True
    
    resp = TriggerVideoRecordingResponse()

    resp.success = True
    return resp



if __name__== "__main__":
    rospy.init_node("video_recorder")
    s = rospy.Service('video_recorder', TriggerVideoRecording, video_service_callback)

    cap = cv2.VideoCapture("sample_video.mp4")

    if(not cap.isOpened()):
        print("Error opening video stream or file")

    frame_dims = (int(cap.get(3)), int(cap.get(4)))

    rospy.loginfo("Video Recorder ready")

    recording = False
    out = None
    while not rospy.is_shutdown():
        with lock:
            if stop_current_recording:

                stop_current_recording = False
                if recording:
                    rospy.loginfo("Stopping current recording")
                recording = False
                if out is not None:
                    out.release()
            
            if start_new_recording:
                recording = True
                start_new_recording = False
                rospy.loginfo("Starting recording for " + out_filename)
                out = cv2.VideoWriter(out_filename,
                                      0x00000021,
                                      30,
                                      frame_dims)

        if not recording:
            rospy.sleep(0.1)
            continue


        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Camera capture failed")
            break
        
        out.write(frame)

    out.release()
    cap.release()

