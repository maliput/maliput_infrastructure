# Setup

## Supported platforms

* For regular workspaces, only Ubuntu Bionic Beaver 18.04 LTS is supported as
  the host OS.

* For dockerized workspaces, only Nvidia powered machines are supported as host
  machines.

## Prerequisites

* To get all necessary scripts and repos files, clone this repo locally at some
  path.

* To actually use the scripts, ensure you have the following packages installed:

```sh
sudo apt install python3-jinja2
sudo apt install pciutils
```

* To pull private repositories, the current user default SSH keys will be used
  (and thus assumed as both necessary and sufficient for the purpose).

* Dockerized workspaces require *nvidia-docker2*, so ensure you have a NVIDIA
  card and the drivers installed. You can check out
  [nvidia-docker2 installation instructions](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0))
  if needed.
* On Ubuntu Xenial hosts, you may also need to install docker from their
  repository. Instructions can be found
  [at this helpful post](https://gist.github.com/Brainiarc7/a8ab5f89494d053003454efc3be2d2ef).

## Basic Setup

To setup a regular workspace, run:

```sh
dsim-repos-index/setup/setup_workspace dsim_workspace
```

To setup a dockerized workspace, run:

```sh
dsim-repos-index/setup/setup_dockerized_workspace dsim_workspace
```

Note: The docker image will be called maliput-devel:<unix_timestamp>. We decided
to use a unix timestamp as a tag so we can create as many docker images as we
want whenever we run the above command without worrying about potentially
overwrite something. The process for updating your docker image is explained
below.

Both operations will setup a `colcon`-like workspace that uses binaries for
upstream dependencies whenever possible.

## Bringup

To bring up any of these workspaces, run:

```sh
source dsim_workspace/bringup
```

You can always leave the workspace by `exit`ing it.

## Advanced Setup

None of the basic setup scripts described so far actually build up a workspace,
but rely on another script that is generated from proper default options. These
options, however, are not the only possible combinations available but just the
most commonly used ones.

You can check all available workspace options by running:

```sh
dsim-repos-index/setup/generate_setup_script -h
```

The output of this script is pure bash code that can be verified, modified and
executed. It is also self contained and thus portable.

## Build Your Workspace

1. Ensure you `source bringup`.

2. Once you are in the workspace, run the following if you are using
   **drake's binary**:

```sh
colcon build
```

If you are using **drake's source**, run:

```sh
colcon build --cmake-args -DWITH_PYTHON_VERSION=3
```

3. Finally, run:

```sh
source install/setup.bash
```

**Note:** We recommend you to run `delphyne-gazoo` and `delphyne-mali` (type
  them on your terminal) to see if everything is properly working.

## How to Build and Run Tests

At the moment, tests are built by default, so once colcon finishes building, you can run:

```sh
colcon test --event-handlers=console_direct+ --return-code-on-test-failure --packages-skip PROJ4
```

## How to Update Your Workspace

To update your workspace:

1. Update your local clone of `dsim-repos-index`.
2. Run

```sh
cd dsim_workspace
source bringup -u [dsim-repos-index]
```

Note: You need to provide the **full path to the dsim-repos-index repository**
to the -u option. This will **increase your docker image size** because it will
commit the image with the new changes. Also, we **don't recommend you to update
your workspace using this command** if you decided to customize your setup
instead of using **setup_dockerized_workspace** or **setup_workspace**

## How to Check Drake Version

After entering your workspace, building it, and executing `source ./install/setup.bash`, execute:

```
which-drake
```

## How to Update Drake

To update Drake within your workspace:

1. Select a Drake SHA that you would like to use. Ensure it is the last commit
   of a nightly release, see
   [nightly build](https://drake-jenkins.csail.mit.edu/view/Nightly%20Production/).
2. In your local clone of `dsim-repos-index`, open `drake.repos` and update the
   SHA.
3. In your workspace, execute `source bringup -u [dsim-repos-index]`.
4. PR the change to `drake.repos` and merge it into master.
