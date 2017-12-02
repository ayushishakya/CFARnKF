# CFARnKF

It is a ROS based implementation of Constant False Alarm Rate algorithm and Kalman Filter for object detection.

ROS WORKFLOW :-

(input_image) ---(input)--->> (object_detect) ---(state)--->> (print)

USAGE :-

window 1:
rosrun </directory name/> pubs -vid </location of video/>

**takes location of video as input and sends it to the node- object_detect via topic- input.

window 2:
rosrun </directory name/> detect

**subscribes to the topic- input, performs algorithms on the image and publishes it on the topic- state

. window 3:
rosrun </directory name/> subs

 **node print subcribes to the topic- state and prints the location of buoys in the video.
