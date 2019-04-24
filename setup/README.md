# Maliput Workspace Setup

## Supported platforms

* For regular workspaces, only Ubuntu Bionic Beaver 18.04 LTS is supported as host OS.

* For dockerized workspaces, only Nvidia powered machines are supported as host machines.

## Prerequisites

* To get all necessary scripts and repos files, clone this repo locally at some path.

* To actually use the scripts, make sure you have the following packages installed:

```sh
sudo apt install python3-jinja2
sudo apt install pciutils
```

* To pull private repositories, the current user default SSH keys will be used (and thus assumed as both necessary and suficient for the purpose).

* Dockerized workspaces require *nvidia-docker2*, so make sure you have an NVIDIA card and the drivers installed.
You can check out [nvidia-docker2 installation instructions](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)) if needed
* On Ubuntu Xenial hosts, you may also need to install docker from their repository. Instructions can be found
[at this helpful post](https://gist.github.com/Brainiarc7/a8ab5f89494d053003454efc3be2d2ef)

## Basic setup

To setup a regular workspace, run:

```sh
path/to/dsim-repos-index/setup/setup_workspace path/to/my/workspace
```

To setup a dockerized workspace, run:

```sh
path/to/dsim-repos-index/setup/setup_dockerized_workspace path/to/my/workspace
```

Both operations will setup a `colcon` like workspace that uses binaries for
upstream dependencies whenever possible.

Note: You can add `-i` to any of the above scripts to force the installation on a given path:
```sh
setup_dockerized_workspace -i /path/to/workspace
```

To bring up any of these workspaces, run:

```sh
source path/to/my/workspace/bringup
```

You can always leave the workspace by `exit`ing it.

## Advanced setup

None of the basic setup scripts described so far actually build up a workspace, but rely
on another script that is generated from proper default options. These options, however,
are not the only possible combinations available but just the most commonly used ones.

You can check all available workspace options by running:

```sh
path/to/dsim-repos-index/setup/generate_setup_script -h
```

The output of this script is pure bash code that can be verified, modified and executed. It
is also self contained and thus portable.

## How to build your workspace

1. Make sure you `source bringup`.

2. Source ament cmake.
If you are using Ubuntu 18.04 (Bionic) then run
```sh
source /opt/ros/crystal/setup.bash
```
If you are using Ubuntu 16.04 (Xenial) then run
```sh
source /opt/ros/bouncy/setup.bash
```

3. Once you are sitting in the workspace, run the following if you are using **drake's binary**

```sh
colcon build
```
If you are using **drake's source** then run:
```sh
colcon build --cmake-args -DWITH_PYTHON_VERSION=3
```

4. Finally, run
```sh
source install/setup.bash
```

**Note:** We recommend you to run `delphyne-gazoo` and `delphyne-mali` (type them on your terminal) to see if everything is working properly.

## How to build and run tests

At the moment tests are built by default, so once colcon finishes building, you can run
```sh
colcon test --event-handlers=console_direct+ --return-code-on-test-failure --packages-skip PROJ4
```
