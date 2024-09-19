# Enhancing Visual Place Recognition via Fast and Slow Adaptive Biasing in Event Cameras

[![Project Video](https://img.shields.io/badge/Video-Watch-red)](https://www.youtube.com/watch?v=8D9gtHqteEQ) [![Website](https://img.shields.io/badge/Website-Visit-blue)](https://gokulbnr.github.io/publication/dvs-biasing-vpr) [![Dataset](https://img.shields.io/badge/Dataset-Download-green)](https://huggingface.co/datasets/gokulbnr/QCR-Fast-Slow-Event-Dataset) [![Preprint](https://img.shields.io/badge/Preprint-Read-orange)](https://arxiv.org/abs/2403.16425)

Welcome to the official repository for the paper [**Enhancing Visual Place Recognition via Fast and Slow Adaptive Biasing in Event Cameras**](https://arxiv.org/abs/2403.16425), to be presented at the 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2024). This work introduces feedback control algorithms that dynamically change bias parameters for event-cameras to stabilize event-rate in an online manner. The work reports improvements in visual place recognition performances across variations in environment brightness conditions, validated through comprehensive real-time evaluations using a new [QCR-Fast-and-Slow-Event-Dataset](https://huggingface.co/datasets/gokulbnr/QCR-Fast-Slow-Event-Dataset).

## Code and Environment Setup
```bash
git clone git@github.com:gokulbnr/fast-slow-biased-event-vpr.git
cd fast-slow-biased-event-vpr
mamba env create -f environment.yaml
pip install git+ssh://git@github.com/gokulbnr/tonic.git@develop
```

## Fast and Slow Biasing of Event Cameras
We make use of a ROS1 node `fast_and_slow_controller` to update Event-Camera bias parameters in a online manner. This node has to run with [jAER](https://github.com/SensorsINI/jaer)v1.9.5 with [unicast datagram (UDP) output enabled](https://docs.google.com/document/d/1fb7VA8tdoxuYqZfrPfT46_wiT1isQZwTHgX8O22dJ0Q/edit#heading=h.9zam901lyzxx).

### Setup jAER
To sort out your dependencies for jAER, please use its [user guide](https://docs.google.com/document/d/1fb7VA8tdoxuYqZfrPfT46_wiT1isQZwTHgX8O22dJ0Q/edit#heading=h.ukkzt7422992). The proposed approach has been rigorously tested on devices running Ubuntu 20 and Ubuntu 22, both utilizing x86_64 architecture. The instructions to setup jAER up is as follows: 
```bash
git clone git@github.com:SensorsINI/jaer.git
cd jaer
git checkout 1.9.5
time ant jar
```

## Dataset Collection

The event streams were recorded using the [jAER](https://github.com/SensorsINI/jaer)v1.9.5 library for a [DAVIS346Red](https://inivation.com/wp-content/uploads/2019/08/DAVIS346.pdf) device. Information regarding setting up the jAER can be found in it's [documentation](https://docs.google.com/document/d/1fb7VA8tdoxuYqZfrPfT46_wiT1isQZwTHgX8O22dJ0Q/edit#heading=h.en40wtalica4).

## Dataset Download and Processing

Link to the dataset: https://huggingface.co/datasets/gokulbnr/QCR-Fast-Slow-Event-Dataset

To process raw data from traverses into geotagged image sequences, please use `scripts/process_data.sh`.

```bash
mamba activate evpr
cd data_processing
bash scripts/process_data.sh <experiment_name> <iteration_number> <path_to_experiment_home> <save_path_for_processed_data>
```

## Cite us at
```
@article{nair2024enhancing,
  title={Enhancing Visual Place Recognition via Fast and Slow Adaptive Biasing in Event Cameras},
  author={Nair, Gokul B and Milford, Michael and Fischer, Tobias},
  journal={arXiv preprint arXiv:2403.16425},
  year={2024}
}
```

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
