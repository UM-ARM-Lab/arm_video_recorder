#! /usr/bin/env python
import cv2
import rospy
from std_msgs.msg import String
from arc_video_recorder.srv import TriggerVideoRecording, TriggerVideoRecordingResponse



out = None
recording = False
frame_dims = None

# class VideoRecorder:
#     def __init__(self, outfile_name):
#         self.sub = rospy.Subscriber("camera", MotionStatus,
#                                     self.left_arm_motion_status_callback)
#         self.out = cv2.VideoWriter(outfile_name,
#                                    cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
#                                    30,
#                                    (frame_width, frame_height))

#     def subscriber_callback(self, name)

#     def record_frame(self):
#         ret, frame = cap.read()
#         if not ret:
#             rospy.logerr("Camera capture failed")
#         out.write(file)
#         cv.imshow('Frame', frame)
#         if cv2.waitKey(25) & 0xFF == ord('q'):
#             break
        
        
def video_service_callback(req):
    global out
    
    recording = req.record
    resp = TriggerVideoRecordingResponse()
    if not recording:
        resp.success = False
        return resp
    
    out = cv2.VideoWriter(req.filename,
                          cv2.VideoWriter_fourcc(*'H264'),
                          30,
                          frame_dims)
    resp.success = True
    return resp




if __name__== "__main__":
    rospy.init_node("video_recorder")
    s = rospy.Service('video_recorder', TriggerVideoRecording, video_service_callback)
    

    cap = cv2.VideoCapture("sample_video.mp4")

    if(not cap.isOpened()):
        print("Error opening video stream or file")

    frame_dims = (int(cap.get(3)), int(cap.get(4)))


    out = cv2.VideoWriter('output.mp4',
                          cv2.VideoWriter_fourcc('M','P','E','G'),
                          30,
                          frame_dims)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # out.write(frame)

        
    # while not rospy.is_shutdown():
    #     if not recording:
    #         rospy.sleep(0.1)
    #         continue
        
    #     ret, frame = cap.read()
    #     if not ret:
    #         rospy.logerr("Camera capture failed")
    #         break
        
    #     out.write(file)
        

    cap.release()

