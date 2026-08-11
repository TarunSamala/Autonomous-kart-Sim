# VSLAM

This package adds RTAB-Map keyframe mapping and loop closure on top of the
validated RGB-D VIO baseline.

```text
RGB + registered depth + camera IMU
                 |
                 v
        /localization/vio/odometry
                 |
                 v
     RTAB-Map graph + loop closure
                 |
                 v
      map -> vio/odom -> fsds/FSCar
```

## Mapping

Start FSDS and the bridge, stop any standalone `vio-start` process, then run:

```zsh
vslam-start
```

The map is persisted in `~/.ros/fsds_vslam.db`. To intentionally discard it
and begin a new map:

```zsh
vslam-start delete_db:=true
```

Drive slowly around the track and revisit previously observed areas from a
similar viewpoint so loop closures can be detected.

## Localization in a saved map

```zsh
vslam-start localization:=true
```

Useful options:

```zsh
vslam-start gui:=false
vslam-start point_cloud:=true
vslam-start database_path:=/absolute/path/to/map.db
```

Ground truth at `/fsds/testing_only/odom` is reserved for evaluation and is not
an input to VIO or VSLAM.
