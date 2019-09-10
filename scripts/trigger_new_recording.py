#! /usr/bin/env python
"""
This node can be used to start a new recording by launching this node from the command line.
Note: The ros service call can (and probably should) be placed inside the code to run experiments.
Running this node is more manual, and more error prone. But it is likely faster to get running


Call this node as
rosrun arm_video_recorder trigger_new_recording.py _name:="video_name _a"
"""

import rospy
import sys
import datetime
from arm_video_recorder.srv import TriggerVideoRecording, TriggerVideoRecordingResponse

def stop_recording():
    rospy.loginfo("Stopping recording")
    try:
        stop = rospy.ServiceProxy("video_recorder", TriggerVideoRecording)
        stop(name, False, 1)
    except rospy.ServiceException, e:
        print("Failed to stop recording: %s"%e)

def record(name):

    rospy.wait_for_service("video_recorder")
    try:
        start = rospy.ServiceProxy("video_recorder", TriggerVideoRecording)
        start(name, True, 60*60)
    except rospy.ServiceException, e:
        print("Serivce call failed: %s"%e)
        
        
    while not rospy.is_shutdown():
        print("Recording...")
        rospy.sleep(60)

    




if __name__== "__main__":
    rospy.init_node("trigger_new_video")

    name = rospy.get_param('~name', "Recording")
    add_timestamp = rospy.get_param('~add_timestamp', True)

    if(add_timestamp):
        name += "_" + datetime.datetime.now().isoformat()
    name += ".mp4"


    rospy.loginfo("Starting new recording: " + name)
    rospy.on_shutdown(stop_recording)
    record(name)
