# ES-PTAM: Event-based Stereo Parallel Tracking and Mapping

[![ES-PTAM: Event-based Stereo Parallel Tracking and Mapping](docs/es_ptam_thumbnail.jpg)](https://youtu.be/z7J3lZOYwKs)

This repository contains code for the paper [**ES-PTAM: Event-based Stereo Parallel Tracking and Mapping**](), by [Suman Ghosh](https://www.linkedin.com/in/suman-ghosh-a8762576/), [Valentina Cavinato](https://ch.linkedin.com/in/valentina-cavinato) and [Guillermo Gallego](https://sites.google.com/view/guillermogallego), published at the **European Conference on Computer Vision (ECCV) Workshops 2024**, Milan, Italy.

If you use this work, please cite it as:

```bibtex
@InProceedings{Ghosh24eccvw,
  author = {Suman Ghosh and Valentina Cavinato and Guillermo Gallego},  
  title = {ES-PTAM: Event-based Stereo Parallel Tracking and Mapping},
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
* [Installation instructions](docs/installation.md)
* [Running the code with various configurations](docs/running.md)
* [Datasets used](docs/datasets.md)
* [Running Examples](docs/examples.md)
* [Evaluation scripts](docs/evaluation.md)


## License

The license is available [here](TBD).

Additional Resources on Event-based Vision
-------
* [MC-EMVS: Multi-Event-Camera Depth Estimation and Outlier Rejection by Refocused Events Fusion](https://github.com/tub-rip/dvs_mcemvs)
* [Research page (TU Berlin RIP lab)](https://sites.google.com/view/guillermogallego/research/event-based-vision)
* [Course at TU Berlin](https://sites.google.com/view/guillermogallego/teaching/event-based-robot-vision)
* [Survey paper](http://rpg.ifi.uzh.ch/docs/EventVisionSurvey.pdf)
* [List of Resources](https://github.com/uzh-rpg/event-based_vision_resources)
