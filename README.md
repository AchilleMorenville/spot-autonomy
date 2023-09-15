# Spot Autonomy
An improved mapping and navigation system for the Spot robot of Boston Dynamics.

## Dependencies

* Docker

## Installation

Once the repository has been cloned on the Spot CORE payloads, the docker container can be build and launched using:

```
bash build_and_run.bash
```

## References

Parts of the code of the package `aut_lidar_odometry` is inspired from [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM).
```
@inproceedings{legoloam2018shan,
  title={LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain},
  author={Shan, Tixiao and Englot, Brendan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={4758-4765},
  year={2018},
  organization={IEEE}
}
```