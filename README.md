# arm_video_recorder
ROS node to record from a connected video device (webcam) and save to a file

## Basic Usage

1. Clone repo, rebuild (`catkin_make`), re-source (`source ~/catkin_ws/devel/setup.bash`)
2. Plug in camera to USB port
2. `roslaunch arm_video_recorder arm_video_recorder.launch`
3. Call the service to start recording video. Call the service again to stop recording


## Service Arguments
- `bool record`: If `False` request recording stop. If `True` request recording start (stops previous recording). 
- `string filename`: Filepath to save recorded video. Can be absolute or relative. Must end with `.mp4` (recommended) or `.avi`.
- `float timeout_in_sec`: Maximum time to record. Must be set `> 0`. Recommend setting to 1hr to avoid large video recordings if you forget to stop recordings

## High Quality Video
Any camera that can interface with opencv's VideoCapture can be recorded. For low quality video plug a webcam into the computer running this ros node.

For high quality video, connect the lab camera to the capture card via HDMI. Connect the capture card to the computer via USB. 
If necessary, change settings on the Camera to prevent showing status icons (e.g. battery level) on the HDMI output.

![Capture Card:](https://github.com/UM-ARM-Lab/arm_video_recorder/blob/master/CaptureCard.JPG)
![MiniHDMIConnection:](https://github.com/UM-ARM-Lab/arm_video_recorder/blob/master/MiniHDMI.JPG)
