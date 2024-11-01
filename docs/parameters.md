## Parameter Tuning

### Mapper

The following parameters are passed using ROS launch files:
* **num_ev_per_map**: Number of events per camera used for generating one local map
* **max_duration**: Maximum time duration acceptable for a single batch of events, beyond which data is treated as noise because the event rate is too low. 
* **max_duration**: Minimum time duration acceptable for a single batch of events. This is used for filtering data when event rate is too high, like with flashing lights.
* **on_demand**: If set to `true`, a new map is generated only when camera view has moved suffieciently far away from the last available map. This can be controlled using the Map Expansion parameters described below. If set to `false`, mapping is done at a fixed rate (10 Hz), which improves overall VO performance but increases computational overheads.
* **auto_copilot**: If set to `true`, mapping is automatically done using the tracked event camera pose as soon as it is available. 
* **init_wait_time**: Initial buffer time for collecting enough events and camera pose information before starting to map.
* **calib_path**: Path to intrinsic and extrinsic camera calibration file, needed for certain datasets specified by `calib_type`.
* **mocap_calib_path**: Path to hand-eye calibration file, needed for certain datasets specified by `calib_type`.
<br>

The following legacy parameters are also passed via `.conf` files:
* **out_path**: Path to output results for saving depthmaps etc.
* **calib_type**: Refers to how calibration data has been provided (YAML, JSON, hard-coded etc.), depending on the source of the dataset. Refer to the `.conf` files to find the correct value of this parameter for each dataset.
* **event_topic0**: Left event camera topic
* **event_topic1**: Right event camera topic
* **event_topic2** : Third event camera topic for trinocular fusion
* **pose_topic**: Topic containing the pose of the stereo rig
* **min_depth**: Minimum estimable depth of the scene
* **max_depth**: Maximum estimable depth of the scene
* **dimZ**: Number of depth planes to place within the assumed scene depth range. By default, the planes are uniformly distributed in _inverse depth_ space. To place depth planes uniformly along _depth_ space, turn the setting `DEFINE_USE_INVERSE_DEPTH` in `mapper_emvs_stereo/CMakeLists.txt` to `OFF`.
* **process_method**: 1 (fusion only across cameras);
* **stereo_fusion**: A number between 1-6. It decides the method used to fuse DSIs across cameras. 
 	- 1: min;
 	- 2: Harmonic mean;
 	- 3: Geometric mean;
 	- 4: Arithmetic mean;
 	- 5: RMS mean; 
 	- 6: max
* **adaptive_threshold_c**: Parameter for Gaussian adaptive thresholding filter applied on the DSI before extracting depth map
* **median_filter_size**: Kernel size of median filter applied on DSI for removing noise
* **max_confidence**: Use this value to manually set the upper limit of the range of values in each DSI, before normalization to [0, 255]. If set to 0 (by default), the maximum value of each DSI is used as its upper limit. For long sequences, setting a constant upper limit for all DSIs regularizes depth map filtering, and prevents sections with low event count from generating noisy depth estimates. In conjunction with `adaptive_threshold_c`, this values regulates the trade-off between reconstructing more 3D points and filtering out noise. Icreasing this parameter makes noise filtering stricter.

### Map Expansion

* **visibility_threshold**: Minimum acceptable number of points in the point cloud currently in
the camera view before triggering map expansion. Increasing it triggers an earlier
expansion of the map.
* **coverage_threshold**: Minimum percentage of overlapping pixels between the current
accumulated binary event image and reprojected point clouds at the current view, before
triggering map expansion. Increasing it also triggers earlier map expansion.
* **baseline_threshold**: Maximum ratio of camera baseline to mean depth allowed before
creating a new map. Reducing it triggers earlier map expansion.
* **rate**: The rate for checking whether map expansion is required, using the above
threshold parameters.

### Tracker
Please refer to the [EVO tracker](https://github.com/uzh-rpg/rpg_dvs_evo_open/?tab=readme-ov-file#tuning) for details about the tracking parameters passed via launch file.

