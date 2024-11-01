# Running the demo live
Install [ROVIO](https://github.com/ethz-asl/rovio) for bootstrapping using grayscale frames and IMU.
Then copy the [rovio_davis_cvpr_live.launch]() file into your local ROVIO `launch` directory. 

First, calibrate you stereo DAVIS cameras using [kalibr](https://github.com/ethz-asl/kalibr) and save it in your local calib paths for the mapper, tracker and ROVIO. 
For reference, check [calib_mcemvs.yaml](/mapper_emvs_stereo/calib/calib_mcemvs.yaml) in the mapper and [davis_0889.yaml](dvs_tracking/parameters/calib/davis_0889.yaml) in the tracker packages.
Make sure the calibration paths are updated in the respective launch files for all 3 packages (mapper, tracker and ROVIO).

Run roscore in a terminal and keep it running.

    roscore

In another terminal, run the cameras:

    roslaunch davis_ros_driver stereo.launch

In a separate terminal, launch the camera tracking node:

    roslaunch dvs_tracking live_tracker_demo.launch

In a separate terminal, launch the mapping node:

    roslaunch mapper_emvs_stereo live_mapper_demo.launch

In a separate terminal run the following: 

    python3 rqt_evo/src/rqt_evo/reset_rovio.py

This will allow to reset the rovio tracker in case it fails due to independent motion in the scene.

In a separate terminal: 

    sudo python3 rqt_evo/src/rqt_evo/keypresses.py

This script will listen globally for keypresses, the following mappings key/actions are in place:

| Key | Action                                           |
|-----|--------------------------------------------------|
| t   | Reset camera tracking                            |
| m   | Switch map expansion ON/OFF                      |
| e   | Switch copilot ON/OFF                            |
| u   | Update map manual request                        |
| p   | Switch ON/OFF appending to the global pointcloud |
| r   | Reset ROVIO tracker                              |
| x   | Disable listening for keypresses                 |
| k   | Quit the program                                 |


