# Running the demo live

Run roscore in a terminal and keep it running.

    roscore

In another terminal, run the cameras:

    roslaunch davis_ros_driver stereo.launch

In a separate terminal, launch the camera tracking node:

    roslaunch dvs_tracking live_mcemvs_demo.launch

In a separate terminal, launch the mapping node:

    roslaunch mapper_emvs_stereo live_evo_demo.launch

In a separate terminal run the following: 

    python3 rqt_evo/src/rqt_evo/reset_rovio.py

This will allow to reset the rovio tracker in case it collapses.

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


