## CI Image

### Summary

A dockerfile is provided to be used for CI.
This image simply contains:
 - Ubuntu Focal (As BASE image)
 - ros2-apt-source
 - Base essentials tools

### Why?

Typically we should be able to rely on GH actions like:
 - https://github.com/ros-tooling/setup-ros
 - https://github.com/ros-tooling/action-ros-ci

However, after ROS 2 key expiring during 2025/06 plus Foxy being EOL we ran out of support for foxy in these
useful github actions.

### Usage

```
docker build -t ubuntu_focal_foxy .
```

```
docker run --rm -it ubuntu_focal_foxy bash
```
