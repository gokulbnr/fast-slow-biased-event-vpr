# Enhancing Visual Place Recognition via Fast and Slow Adaptive Biasing in Event Cameras

[![Website](https://img.shields.io/badge/Website-Visit-blue)](https://gokulbnr.github.io/publication/dvs-biasing-vpr) [![Dataset](https://img.shields.io/badge/Dataset-Download-green)](https://huggingface.co/datasets/gokulbnr/QCR-Fast-Slow-Event-Dataset) [![Preprint](https://img.shields.io/badge/Preprint-Read-orange)](https://arxiv.org/abs/2403.16425)

[![Video](https://img.youtube.com/vi/8D9gtHqteEQ/0.jpg)](https://www.youtube.com/watch?v=8D9gtHqteEQ)

Welcome to the official repository for **Enhancing Visual Place Recognition via Fast and Slow Adaptive Biasing in Event Cameras**, to be presented at the 2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2024). This work introduces feedback control algorithms that dynamically change bias parameters for event-cameras to stabilize event-rate in an online manner. The work reports improvements in visual place recognition performances across variations in environment brightness conditions, validated through comprehensive real-time evaluations using a new [QCR-Fast-and-Slow-Event-Dataset](https://huggingface.co/datasets/gokulbnr/QCR-Fast-Slow-Event-Dataset).

## Code and Environment Setup

```bash
git@github.com:gokulbnr/fast-slow-biased-event-vpr.git
cd fast-slow-biased-event-vpr
mamba env create -f environment.yaml
pip install git+ssh://git@github.com/gokulbnr/tonic.git@develop
```

## Dataset Download and Processing

Link to the dataset: https://huggingface.co/datasets/gokulbnr/QCR-Fast-Slow-Event-Dataset

The event streams were recorded using the [jAER](https://github.com/SensorsINI/jaer)v1.9.5 library for a [DAVIS346Red](https://inivation.com/wp-content/uploads/2019/08/DAVIS346.pdf) device. Information regarding setting up the jAER can be found in it's [documentation](https://docs.google.com/document/d/1fb7VA8tdoxuYqZfrPfT46_wiT1isQZwTHgX8O22dJ0Q/edit#heading=h.en40wtalica4). To process raw data from traverses into geotagged image sequences, please use `scripts/process_data.sh`.

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
