# ES-PTAM: Event-based Stereo Parallel Tracking and Mapping

[![ES-PTAM: Event-based Stereo Parallel Tracking and Mapping](docs/es_ptam_thumbnail.jpg)](https://youtu.be/z7J3lZOYwKs)

Official repository for [**ES-PTAM: Event-based Stereo Parallel Tracking and Mapping**](https://arxiv.org/pdf/2408.15605), by [Suman Ghosh](https://www.linkedin.com/in/suman-ghosh-a8762576/), [Valentina Cavinato](https://ch.linkedin.com/in/valentina-cavinato) and [Guillermo Gallego](https://sites.google.com/view/guillermogallego), published at the **European Conference on Computer Vision (ECCV) Workshops 2024** Milan, Italy.

It has been accepted as a **Spotlight** paper at the [NeVi](https://sites.google.com/view/nevi2024/home-page) Workshop.

The [PDF of the paper is available](https://arxiv.org/pdf/2408.15605). If you use this work in your research, please cite it as follows:

```bibtex
@InProceedings{Ghosh24eccvw,
  author = {Suman Ghosh and Valentina Cavinato and Guillermo Gallego},  
  title = {{ES-PTAM}: Event-based Stereo Parallel Tracking and Mapping},
  booktitle = {European Conference on Computer Vision (ECCV) Workshops},
  year = {2024}
}
```

## Data Processing Pipeline

![pipeline](docs/pipeline_esptam.png)

### Input
* Events from two or more cameras
* Camera calibration (intrinsic, extrinsic) parameters

### Output
* Camera (i.e., sensor rig) poses
* Depth map
* Confidence map
* Point cloud
* Intermediate ray density maps / Disparity Space Images (DSI)

## Code
* [Installation](docs/installation.md)
* [Running examples on different datasets](docs/examples.md)
* [Running live with DAVIS cameras]()
* [Parameter tuning](docs/parameters.md)

## Results
The original ES-PTAM trajectories and GT poses for various sequences are available [here](trajectory_eval).

They have been evaluted using [this tool](https://github.com/uzh-rpg/rpg_trajectory_evaluation/tree/master).

## License

The license is available [here](TBD).

Additional Resources on Event-based Vision
-------
* [MC-EMVS: Multi-Event-Camera Depth Estimation and Outlier Rejection by Refocused Events Fusion](https://github.com/tub-rip/dvs_mcemvs)
* [EVO: Event based Visual Odometry](https://github.com/uzh-rpg/rpg_dvs_evo_open/)
* [Research page (TU Berlin RIP lab)](https://sites.google.com/view/guillermogallego/research/event-based-vision)
* [Course at TU Berlin](https://sites.google.com/view/guillermogallego/teaching/event-based-robot-vision)
* [Survey paper](http://rpg.ifi.uzh.ch/docs/EventVisionSurvey.pdf)
* [List of Resources](https://github.com/uzh-rpg/event-based_vision_resources)
