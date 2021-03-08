# grvc-ual
[![Releases](https://img.shields.io/github/release/grvcTeam/grvc-ual.svg)](https://github.com/grvcTeam/grvc-ual/releases)

A repository for the GRVC UAV abstraction layer.

## Installation and use

Download the latest stable version from [here](https://github.com/grvcTeam/grvc-ual/releases).

Start configuring which backends should be active:

```
    $ cd ~/catkin_ws/src/grvc-ual
    $ ./configure.py
```

You can find detailed instructions for installation and how to use the UAL in the [Wiki](https://github.com/grvcTeam/grvc-ual/wiki).

## Compatibile autopilots

### DJI A3/N3

 * Via [ual_backend_dji_ros](https://github.com/grvcTeam/grvc-ual/wiki/Backend-DJI-ROS)
 * ROS dji_sdk version [TBD]()
 * DJI Onboard SDK version [TBD]()

 * https://github.com/grvcTeam/grvc-ual/wiki/Backend-Crazyflie)

## Citation
If you find UAL useful in your research, please consider citing:

```
@inproceedings{real2018ual,
    Author = {Fran Real, Arturo Torres-González, Pablo Ramón Soria, Jesús Capitán and Aníbal Ollero},
    Title = {UAL: an abstraction layer for unmanned vehicles},
    Booktitle= {2nd International Symposium on Aerial Robotics (ISAR)},
    Year = {2018}
}
```
