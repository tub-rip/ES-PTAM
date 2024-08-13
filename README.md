# ES-TAM: Event-based Stereo Parallel Tracking and Mapping

[![Multi-Event-Camera Depth Estimation and Outlier Rejection by Refocused Events Fusion](docs/mcemvs_thumbnail.jpg)](https://youtu.be/o7Bxg9XlHmg)

This repository contains code for the paper [**Event-based Stereo Parallel Tracking and Mapping**](), by [Suman Ghosh](https://www.linkedin.com/in/suman-ghosh-a8762576/), [Valentina Cavinato](https://ch.linkedin.com/in/valentina-cavinato) and [Guillermo Gallego](https://sites.google.com/view/guillermogallego), published at the [ECCV Workshop on Neuromorphic Vision (NeVi) 2024](https://sites.google.com/view/nevi2024/home-page).

```bibtex
@InProceedings{Ghosh22aisy,
  author = {Ghosh, Suman and Cavinato, Valentina and Gallego, Guillermo},  
  title = {Event-based Stereo Parallel Tracking and Mapping},
  booktitle = {European Conference on Computer Vision (ECCV) Workshops},
  year = {2024}
}
```

## Data Processing Pipeline


![pipeline](docs/block_all.png)

### Input
* Events from multiple cameras
* Pose of camera rig
* Camera calibration (instrinsic, extrinsic, hand-eye) parameters

### Output
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

The license is available [here](Software_License_Agreement_TUB_dvs_mcemvs.pdf).

Additional Resources on Event-based Vision
-------
* [Research page (TU Berlin RIP lab)](https://sites.google.com/view/guillermogallego/research/event-based-vision)
* [Course at TU Berlin](https://sites.google.com/view/guillermogallego/teaching/event-based-robot-vision)
* [Survey paper](http://rpg.ifi.uzh.ch/docs/EventVisionSurvey.pdf)
* [List of Resources](https://github.com/uzh-rpg/event-based_vision_resources)
