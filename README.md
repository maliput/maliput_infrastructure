# Important:
  Given that the way the workspace is set up is currently being modified a new readme file takes place.
  **This readme file is work in progress.**

  For full information please refer to [previous README.md](DEPRECATED_README.md)

# Table of Contents

- [Introduction](#introduction)
- [Workspaces](#workspaces)
  - [Supported platforms](#supported-platforms)
  - [Prerequisites](#prerequisites)
  - [Usage Instructions](#usage-instructions)
    - [Create a workspace](#create-a-workspace)
      - [Create a containerized workspace](#create-a-containerized-workspace)
      - [Create a non-containerized workspace](#create-a-non-containerized-workspace)
    - [Check your workspace](#check-your-workspace)
    - [Build your workspace](#build-your-workspace)
    - [Test your workspace](#test-your-workspace)
    - [Build your workspace using Static Analyzer](#build-your-workspace-using-static-analyzer)
    - [Build doxygen documentation](#build-doxygen-documentation)
    - [Delete your workspace](#delete-your-workspace)
- [Contributing](#contributing)
  - [Usual workflow](#usual-workflow)
  - [Using binary underlays](#using-binary-underlays)
  - [List of repositories](#list-of-repositories)
  - [How to use CI](#how-to-use-ci)
- [Troubleshooting](#troubleshooting)
  - [Issue forensics](#issue-forensics)

# Introduction

This repository contains `.repos` files and tools that enable the creation and
maintenance of development workspaces. Each `.repos` file brings in a subset
of all needed packages.

For instance, [`maliput.repos`](maliput.repos) pulls all `maliput` packages on road network
descriptions, plus the backend packages like `maliput_malidrive`, `maliput_dragway` and `maliput_multilane` and other packages for
integration proposes and documentation.

# Workspaces

## Supported platforms

* Docker containerized workspaces (**recommended**): A docker image is provided in order to
  shows the steps needed to set up the environment in a containerized workspace.
  When setting up docker, do *not* add yourself to the "docker" group
  since that represents a security risk
  ([it is equivalent to password-less `sudo`](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user)).
  The instructions below use `sudo` for building the image and running the container as a workaround.
* Non-containerized workspaces: Ubuntu Bionic Beaver 18.04 LTS only.


## Prerequisites

* To get all necessary tools and repos files, clone this repo locally.
    ```sh
    git clone git@github.com:ToyotaResearchInstitute/dsim-repos-index.git
    ```
* To pull private repositories, the current user default SSH keys will be used
  (and thus assumed as both necessary and sufficient for the purpose).

* Containerized workspaces require having [`docker engine`](https://docs.docker.com/engine/install/) installed in host machine.
  Also, you can use `nvidia-docker2`. Follow their [instructions](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) if you want to install it.


## Usage instructions

In the following, it is assumed that you want to create a workspace containing
all of DSIM's (Driving Simulation's) repositories, but with a focus on Maliput
and Malidrive development. As such, it suggests the creation of a workspace in a
`maliput_ws` directory and pulling sources from the [`maliput.repos`](maliput.repos)
file.

### Create a workspace

Whether you would like to have a containerized or a non-containerized workspace the instructions are similar.

---
**NOTE**
:  Bear in mind that using a non-containerized workspace makes reproducing and troubleshooting
   issues harder for others. Thus, we highly recommend that you use a containerized workspace.

---

 #### **Create a containerized workspace**
1. #### Build the docker image.
   ```sh
   ./dsim-repos-index/docker/build.sh
   ```
   If you are using nvidia-docker2 add the `--nvidia` option.
   ```sh
   ./dsim-repos-index/docker/build.sh --nvidia
   ```
   ---
   **NOTE**: `build.sh --help` for more options:
      1.  `-i` `--image_name`	Name of the image to be built (default maliput_ws_ubuntu)
      1.  `-w` `--workspace_name`	Name of the workspace folder (default maliput_ws)
   ---

2. #### Create the workspace folder:
   ```sh
   mkdir -p maliput_ws
   ```
   ---
   **NOTE**:
   Instructions assumes `maliput_ws` folder name as default and its location at the same level as the cloned repository folder `dsim-repos-index`.

   ---

3. #### Copy .repos file:
    Copy `dsim-repos-index/maliput.repos` file into `maliput_ws` workspace folder.
    It will be used to bring all the repositories later on.
   ```sh
   cp dsim-repos-index/maliput.repos maliput_ws/
   ```
4. #### Run the container:
   ```sh
   ./dsim-repos-index/docker/run.sh
   ```
   If you are using nvidia-docker2 add the `--nvidia` option.
   ```sh
   ./dsim-repos-index/docker/run.sh --nvidia
   ```
    ---
    **NOTE**:
    `run.sh --help` for more options:
      1.	`-i` `--image_name`	Name of the image to be run (default maliput_ws_ubuntu)
      1.	`-c` `--container_name`	Name of the container(default maliput_ws)
      1.	`-w` `--workspace`	Relative or absolute path to the workspace you want to bind. (default to location of dsim-repos-index folder)
    ---
5. #### Install dependencies:
   During docker build stage a script is copied into the container at `/home/$USER/`.
   ```sh
   sudo ./../install_dependencies.sh
   ```
6. #### Bring/update all the repositories in your workspace:
   Standing at the root of your workspace folder.
   ```sh
   mkdir -p src
   vcs import src < maliput.repos  # clone and/or checkout
   vcs pull src  # fetch and merge (usually fast-forward)
   ```

   This will clone repositories and/or checkout branches, tags or commits as necessary,
   followed by fetching and (likely) fast-forward merging to get branches up to date with
   their upstream counterpart. No merging takes place when a repository is at a given tag
   or commit. Also, note that you can equally bring other repositories as well by repeating
   this `import` and `pull` operation using additional `.repos` files.


7. #### Install all packages' dependencies:

   First update the `ROS_DISTRO` environment variable with your `ros2` version, e.g.:
   ```sh
   export ROS_DISTRO=dashing
   ```
   Install dependencies via `rosdep`:
   ```sh
   rosdep update
   rosdep install -i -y --rosdistro $ROS_DISTRO --skip-keys "ignition-transport7 ignition-msgs4 ignition-math6 ignition-common3 ignition-gui0 ignition-rendering2 libqt5multimedia5 pybind11" --from-paths src
   ```

   Warning
   :   Package dependencies are installed system wide. `rosdep` does not provide any support
       to remove the dependencies it brings. In this regard, disposable containerized workspaces
       help keep development environments clean (as system wide installations within a container
       are limited to that container).

8. #### Install drake:
    ```sh
    sudo ./src/drake_vendor/drake_installer
    ```

9. #### Source ROS environment:

   ```sh
   source /opt/ros/$ROS_DISTRO/setup.bash
   ```


 #### **Create a non-containerized workspace**

  If the workspace is not meant to be run using a container the steps are pretty similar but
  docker related commands must be avoided:

 1. [Create the workspace folder](#create-the-workspace-folder)
 2. [Copy .repos file](#copy-.repos-file)
 3. Install dependencies:
     ```sh
       sudo ./dsim-repos-index/tools/install_dependencies.sh
     ```
 4. [Bring/update all the repositories in your workspace](#bring/update-all-the-repositories-in-your-workspace)
 5. [Install all packages' dependencies](#install-all-packages'-dependencies)
 6. [Install Drake](#install-drake)
 7. [Source ROS environment](#source-ros-environment)

### Check your workspace

Workspace state as a whole encompasses both current local repositories' state plus the state of
the filesystem that hosts it. However, if a workspace is containerized and no customizations are
applied by the user, repositories alone carry the source code and state the list of system dependencies
necessary to build and execute. And we can easily inspect repositories.

1. To check repositories' status, run:

   ```sh
   vcs status src
   ```

2. To see changes in the repositories' working tree, run:

   ```sh
   vcs diff src
   ```

3. To see if (most of) our versioned packages' dependencies have been met, run:

   ```sh
   rosdep check --rosdistro $ROS_DISTRO --skip-keys "ignition-transport7 ignition-msgs4 ignition-math5 ignition-common2 ignition-gui0 ignition-rendering0 libqt5multimedia5 pybind11" --from-paths src
   ```

   Note though that currently not all workspace prerequisites are nor can be dealt with using `rosdep`
   alone and thus `rosdep check` may fall short. When it comes down to pure binary dependencies, `drake`'s
   binary tarball is a good example, but prerequisites may go beyond that, `apt` source lists being another
   good example. See `prereqs` executable files in each repository for further details on what's currently
   being handled outside `rosdep`.

In any given case, one can always resort to the specific tool used for repository versioning (e.g. `git`)
if `vcs` isn't enough or to the specific package managers (e.g. `apt` or `pip`) if `rosdep` isn't enough.

### Build your workspace

1. Build the workspace, which can be done in full or partially.
   Standing at `maliput_ws` root folder:
   ```sh
   cd ~/maliput_ws
   ```

   To build all packages:

   ```sh
   colcon build
   ```

   To build some packages, along with their dependencies (recursively), use the
   `--packages-up-to` flag. For instance, to build `maliput` and `malidrive`:

   ```sh
   colcon build --packages-up-to maliput malidrive
   ```

   To build some packages and only those packages (i.e. without their dependencies),
   use the `--packages-select` flag instead:

   ```sh
   colcon build --packages-select maliput malidrive
   ```

   Note that if dependencies cannot be met, because they are not installed or not built,
   the build will fail. Thus, this flag is usually helpful only to quickly rebuild a package
   after building it along with its dependencies.

Note
:  If you are building `drake` from source as well, make sure `--cmake-args -DWITH_PYTHON_VERSION=3` is
passed to `colcon`. Otherwise, python packages and scripts in `delphyne` and `delphyne-gui` packages
won't find `pydrake`.

Note
:  To build with debug symbols, and given that we use CMake packages only, just make sure
that `CMAKE_BUILD_TYPE=Debug`. You can force it by passing `--cmake-args -DCMAKE_BUILD_TYPE=Debug`
to `colcon`.

Note
:  If you want to build with `clang`, run the following:

   ```sh
   CC=clang CXX=clang++ colcon build --packages-up-to maliput malidrive
   ```

2. Source the workspace:

   ```sh
   source install/setup.bash
   ```

Note
: If `delphyne` is available, we recommend you to run `delphyne-gazoo` and `delphyne-mali` (type them in
your terminal) to see if everything is properly working.

Note
:  See [`colcon` documentation](https://colcon.readthedocs.io/en/released/user/how-to.html#build-only-a-single-package-or-selected-packages) for further reference on `build` support.

### Test your workspace

In a built workspace, run:

```sh
colcon test --event-handlers=console_direct+ --return-code-on-test-failure --packages-skip pybind11
```

Note
:  See [`colcon` documentation](https://colcon.readthedocs.io/en/released/user/how-to.html#run-specific-tests)
for further reference on `test` support.

### Build your workspace using Static Analyzer

  TODO

### Build doxygen documentation

1. Build the workspace, which can be done in full or partially. In particular,
   we are interested in compiling `dsim-docs-bundler`

   Standing at `maliput_ws` root folder:
   ```sh
   cd ~/maliput_ws
   ```

  ```sh
  colcon build --packages-up-to dsim-docs-bundler
  ```

2. Open the documentation with your favorite browser. If Google Chrome is available, you can run:

  ```sh
  google-chrome install/dsim-docs-bundler/share/dsim-docs-bundler/doc/dsim-docs/html/index.html
  ```

### Delete your workspace

  TODO

# Contributing

## Usual workflow

Ours is similar to ROS2's development workflow, and thus many of their tools and practices apply equally.

Workspaces are managed via [`vcs`](https://github.com/dirk-thomas/vcstool), a tool that helps in dealing with
sources distributed across multiple repositories, not necessarily versioned with the same tool (support for `git`,
`hg`, `svn` and `bazaar` is readily available). `vcs` uses `.repos` files for a listing of version pinned sources.

Dependency management is taken care of by [`rosdep`](https://docs.ros.org/independent/api/rosdep/html/commands.html),
a tool that can crawl `package.xml` files and resolve dependencies into a call to the appropriate package
manager for the current platform by means of a public database known as [rosdistro](https://github.com/ros/rosdistro).

To build and test packages, [`colcon`](https://colcon.readthedocs.io/en/released/) abstracts away the details of the
specific build system and testing tools in use and arbitrates these operations to take place in topological order.
Operations will be run in parallel by default.

---
**NOTE**: In all three cases above, the tools delegate the actual work to the right tool for each package and
focus instead on bridging the gap between them. Thus, for instance, `colcon` builds interdependent
CMake packages by running `cmake` and `make` in the right order and setting up the environment for
the artifacts to be available. Same applies for `vcs` and `rosdep`.

---
**NOTE**: These tools do not strive to act like a proxy for every configuration setting or command line option
that underlying tools they delegate work to may have. Thus, it may be necessary to configure the underlying
tool in addition to the configuration for these tools to attain a desired behavior. For instance, limiting
`colcon` parallelism with the `--parallel-workers` switch has no impact on `make` parallelization settings
if this tool is being used.

---

## Using binary underlays

In ROS 2 workspace parlance, an overlay workspace is a workspace that builds on top of another, previously
built workspace i.e. the underlay workspace. A binary underlay is thus the install space of a pre-built
workspace, that packages in downstream workspaces can use to meet their dependencies. As a result, the amount
of code that needs to be compiled when building downstream workspaces gets reduced, enabling faster builds. You may
refer to [`colcon` documentation and tutorials](https://index.ros.org/doc/ros2/Tutorials/Colcon-Tutorial/#source-an-underlay)
for further details.

Several binary underlays are available for download and installation:

- `dsim-desktop-YYYYMMDD-bionic-tar.gz`

  Built nightly, targeting Ubuntu Bionic 18.04 LTS. Contains all known packages in all our repositories as of
  the specified date (DD/MM/YYYY). To be found at `s3://driving-sim/projects/maliput/packages/nightlies/`.

- `dsim-desktop-latest-bionic.tar.gz`

  Built nightly, targeting Ubuntu Bionic 18.04 LTS. Contains the most recent versions of all packages known in
  all our repositories. To be found at `s3://driving-sim/projects/maliput/packages/nightlies/`.

In the following, it is assumed that you want to use a full `dsim-desktop` underlay for working on a
downstream package of your own. As such, it suggests the installation of a `dsim-desktop` binary underlay,
that brings all known packages in all our repositories. You should choose an underlay that is appropriate for
your intended purpose.

1. Download the binary underlay tarball of choice from dsim's S3 bucket:

   ```sh
   aws s3 cp s3://driving-sim/projects/maliput/packages/nightlies/dsim-desktop-latest-bionic.tar.gz \
      /path/to/workspace/dsim-desktop-latest-bionic.tar.gz
   ```

   It is assumed that you have the right AWS credentials configured in your system.
   See [AWS CLI user guide to configuration](https://docs.aws.amazon.com/cli/latest/userguide/cli-chap-configure.html) for further reference.

1. Extract binary underlay tarball:

   ```sh
   sudo mkdir -p /opt/dsim-desktop
   sudo tar -zxvf dsim-desktop-latest-bionic.tar.gz -C /opt/dsim-desktop --strip 1
   ```

1. Install all underlay packages' prerequisites, including drake and ignition binaries:

   TODO

1. Install all underlay packages' dependencies:

   ```sh
   rosdep update
   rosdep install -i -y --rosdistro $ROS_DISTRO --skip-keys "ignition-transport7 ignition-msgs4 ignition-math6 ignition-common3 ignition-gui0 ignition-rendering2 libqt5multimedia5 pybind11" --from-paths /opt/dsim-desktop/*
   ```

1. When exiting the workspace, make sure changes are saved!

  TODO

From then on, before building the workspace, you must source the underlay as follows:

```sh
source /opt/dsim-desktop/setup.bash
```

Note
:  Having an underlay around does not make it a requirement for all workspace builds, but only for
   those that rely on that underlay to get their dependencies met.

## List of repositories

The following is an exhaustive list of the repositories where all relevant packages live, excluding
repositories for upstream dependencies:

- [ToyotaResearchInstitute/delphyne](https://github.com/ToyotaResearchInstitute/delphyne/), that contains a package with the back-end for a traffic level simulator for autonomous driving.
- [ToyotaResearchInstitute/delphyne-gui](https://github.com/ToyotaResearchInstitute/delphyne-gui/), that contains a package with the front-end for a traffic level simulator for autonomous driving.
- [ToyotaResearchInstitute/drake-vendor](https://github.com/ToyotaResearchInstitute/drake-vendor/), that contains a vendoring package for [`drake`](https://github.com/RobotLocomotion/drake).
- [ToyotaResearchInstitute/malidrive](https://github.com/ToyotaResearchInstitute/malidrive/), that contains a package with a `maliput` backend for the OpenDrive specification.
- [ToyotaResearchInstitute/maliput](https://github.com/ToyotaResearchInstitute/maliput/), that contains packages introducing `maliput` and a road network runtime interface.
- [ToyotaResearchInstitute/maliput_malidrive](https://github.com/ToyotaResearchInstitute/maliput_malidrive/), that contains a package with a `maliput` backend for the OpenDrive specification.
- [ToyotaResearchInstitute/maliput-dragway](https://github.com/ToyotaResearchInstitute/maliput-dragway/), that contains an implementation of Maliput's API that allows users to instantiate a multilane dragway.
- [ToyotaResearchInstitute/maliput-integration](https://github.com/ToyotaResearchInstitute/maliput-integration/), that contains integration examples and tools that unify `maliput` core and its possible backends.
- [ToyotaResearchInstitute/maliput-multilane](https://github.com/ToyotaResearchInstitute/maliput-multilane/), that contains an implementation of Maliput's API that allows users to instantiate a multilane road.
- [ToyotaResearchInstitute/maliput_documentation](https://github.com/ToyotaResearchInstitute/maliput_documentation/), that contains a high level documentation (sphinx) and Changelog for maliput & family.

## How to use CI

CI jobs build and test relevant packages for each repository on every PR. Being a multi-repository project,
patches that are not limited to a single repository must be separately PR'd but built and tested together.
To that end, make sure that all PR'd branches that are part of the same patch have the same name
e.g. `my_github_user/my_patch_name`.

**Warning**
: Fork based development is currently not supported. All PRs must come from origin and not a fork.

# Troubleshooting

## Issue Forensics

When reproducing issues, either related to the codebase or to the infrastructure
that supports it, recreating the environment in which these issues arose is crucial.
