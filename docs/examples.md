## Running on Datasets

Run roscore in a terminal and keep it running.

    roscore

In a separate terminal, launch the camera tracking node

    roslaunch dvs_tracking live_tracker_[dataset].launch

In a separate terminal, launch the mapping node

    roslaunch mapper_emvs_stereo live_mapper_[dataset].launch

Finally, in another terminal, play the stereo events and GT camera pose for bootstrapping. 
Note that ES-PTAM is non-deterministic, i.e., results may vary each time you run it on the same rosbag file.
This is due to stochastic operations involved in the tracking, and also, the parallelism of the system. 
The performance differs according to the condition of your PC.
Try playing the bag at a slower rate (especially for higher camera resolution) to improve performance as well as reproducibility.

    rosbag play [ROSBag file].bag -r [rate] --clock

Below, we provide details for running on different example datasets.
  

### RPG_ECCV18_edited

Download the [rpg_monitor_edited.bag](https://drive.google.com/file/d/1P8N3YfYnF5lgOgZGqkMU73otEnedztgy/view?usp=drive_web) file from the [ESVO project page](https://sites.google.com/view/esvo-project-page/home#h.tl1va3u667ae), which contains stereo events and camera poses.\
Run the tracking and mapping nodes using the following launch files.
* Tracking: [live_tracker_eccv18_edited.launch](/dvs_tracking/launch/live_tracker_eccv18_edited.launch)
* Mapping: [live_mapper_eccv18_edited.launch](/mapper_emvs_stereo/launch/live_mapper_eccv18_edited.launch)

Play the ROSBag file

    rosbag play rpg_monitor_edited.bag -r 0.4 --clock

### DSEC

You can download pre-packaged ROSBag files from [here](https://github.com/arclab-hku/Event_based_VO-VIO-SLAM?tab=readme-ov-file#modified-dsec-dataset) which contains stereo event data.\
Alternatively, you can generate stereo event ROSBag files from the [original DSEC h5 data](https://dsec.ifi.uzh.ch/dsec-datasets/download/) using [our h52bag converter](https://github.com/tub-rip/events_h52bag) like:

	events_h52bag zurich_city_04_a_events_left/events.h5 zurich_city_04_a_events_left /dvs/left/events 480 640
	events_h52bag zurich_city_04_a_events_right/events.h5 zurich_city_04_a_events_right /dvs/right/events 480 640
	
Then, download camera poses (obtained via LiDAR-IMU odometry) from [here](https://github.com/tub-rip/dvs_mcemvs/tree/main/data/DSEC). Thanks to [Mathias Gehrig](https://magehrig.github.io/) for the data.

Run the tracking and mapping nodes using the following launch files.
* Tracking: [live_tracker_dsec.launch](/dvs_tracking/launch/live_tracker_dsec.launch)
* Mapping: [live_mapper_dsec.launch](/mapper_emvs_stereo/launch/live_mapper_dsec.launch)

Play the ROSBag file. \
Make sure that the ROS topics are according to what is expected in the launch files. If using the pre-package bag files above, they need to be remapped during playing.\
You also need to align the playing of the events and pose ROSBags, hence the offset needs to be set using the `-s` flag.

    rosbag play dsec_zurich_city_04_a.bag zurich_city_04-odometry/pose.bag /davis/left/events:=/dvs/left/events /davis/right/events:=/dvs/right/events -r 0.02 -s 132 -u 34 --clock

For other sequences of zurich_city_04, the offset `-s` and duration `-u` can be computed using the `start_end_time.yaml` files provided in [our output trajectories](/trajectory_eval/results/dsec) and GT pose timestamps.

### TUM-VIE

From the [TUM-VIE dataset](https://vision.in.tum.de/data/datasets/visual-inertial-event-dataset), download the following files:
* [mocap-desk-events_left.h5](https://tumevent-vi.vision.in.tum.de/mocap-desk/mocap-desk-events_left.h5)
* [mocap-desk-events_right.h5](https://tumevent-vi.vision.in.tum.de/mocap-desk/mocap-desk-events_right.h5) 
* [mocap-desk-vi_gt_data.tar.gz](https://tumevent-vi.vision.in.tum.de/mocap-desk/mocap-desk-vi_gt_data.tar.gz) for camera poses.

Convert left and right events from h5 format to ROSBag using [our h52bag converter](https://github.com/tub-rip/events_h52bag):

	./events_h52bag mocap-desk-events_left.h5 mocap-desk-events_left /dvs/left/events 720 1280
	./events_h52bag mocap-desk-events_right.h5 mocap-desk-events_right /dvs/left/events 720 1280
	
Extract the contents of `mocap-desk-vi_gt_data.tar.gz` into a folder `mocap-desk-vi_gt_data`. Then, convert poses from `mocap_data.txt` to ROSBag using [this script](/mapper_emvs_stereo/scripts/mocap_txt2bag.py):
	
	python mapper_emvs_stereo/scripts/mocap_txt2bag.py --path_prefix mocap-desk-vi_gt_data

This should generate `pose.bag` as output inside the `mocap-desk-vi_gt_data` folder.

Run the tracking and mapping nodes using the following launch files.
* Tracking: [live_tracker_tumvie_calibA.launch](/dvs_tracking/launch/live_tracker_tumvie_calibA.launch)
* Mapping: [live_mapper_tumvie_calibA.launch](/mapper_emvs_stereo/launch/live_mapper_tumvie_calibA.launch)

Play the ROSBag file, leaving out the first few seconds containing a lot of flashing lights and no camera motion:

    rosbag play mocap-desk-events_left.bag mocap-desk-events_right.bag mocap-desk-vi_gt_data/pose.bag -r 0.01 -s 5 --clock
