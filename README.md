# Enhancing Visual Place Recognition via Fast and Slow Adaptive Biasing in Event Cameras

[![Project Video](https://img.shields.io/badge/Video-Watch-red)](https://youtu.be/8D9gtHqteEQ) [![Website](https://img.shields.io/badge/Website-Visit-blue)](https://gokulbnr.github.io/publication/dvs-biasing-vpr) [![Dataset](https://img.shields.io/badge/Dataset-Download-green)](https://huggingface.co/datasets/gokulbnr/QCR-Fast-Slow-Event-Dataset) [![Preprint](https://img.shields.io/badge/Preprint-Read-orange)](https://arxiv.org/abs/2403.16425)


Code and Dataset coming soon!

## Code and Environment Setup

```
git@github.com:gokulbnr/fast-slow-biased-event-vpr.git
cd fast-slow-biased-event-vpr
mamba env create -f environment.yaml
pip install git+ssh://git@github.com/gokulbnr/tonic.git@develop
```

## Dataset Download and Processing

Link to the dataset: https://huggingface.co/datasets/gokulbnr/QCR-Fast-Slow-Event-Dataset

The event streams were recorded using the [jAER](https://github.com/SensorsINI/jaer)v1.9.5 library for a [DAVIS346Red](https://inivation.com/wp-content/uploads/2019/08/DAVIS346.pdf) device. Information regarding setting up the jAER can be found in it's [documentation](https://docs.google.com/document/d/1fb7VA8tdoxuYqZfrPfT46_wiT1isQZwTHgX8O22dJ0Q/edit#heading=h.en40wtalica4). To process raw data from traverses into geotagged image sequences, please use `scripts/process_data.sh`.

```
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
