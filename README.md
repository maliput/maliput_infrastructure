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
    - [Check your workspace](#check-your-workspace)
    - [Build your workspace](#build-your-workspace)
    - [Test your workspace](#test-your-workspace)
    - [Build your workspace using Static Analyzer](#build-your-workspace-using-static-analyzer)
    - [Build doxygen documentation](#build-doxygen-documentation)
- [Contributing](#contributing)
  - [Usual workflows](#usual-workflows)
  - [Using binary underlays](#using-binary-underlays)
  - [List of repositories](#list-of-repositories)
  - [How to use CI](#how-to-use-ci)
- [Troubleshooting](#troubleshooting)
  - [Issue forensics](#issue-forensics)

# Introduction

This repository contains `.repos` files and tools that enable the creation and
maintenance of development workspaces. Each `.repos` file brings in a subset
of all needed packages.

For instance, [`dsim.repos`](dsim.repos) pulls all `maliput` packages on road network
descriptions, plus the `malidrive` backend package and `delphyne` packages for
visualization and prototyping.

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
`maliput_ws` directory and pulling sources from the [`dsim.repos`](dsim.repos)
file.

### Create a workspace

Whether you would like to have a containerized or a non-containerized workspace the instructions are similar.

---
**NOTE**
:  Bear in mind that using a non-containerized workspace makes reproducing and troubleshooting
   issues harder for others. Thus, we highly recommend that you use a containerized workspace.

---

To create a containerized workspace, run:

 **Create a containerized workspace**
1. Build the docker image.
   ```sh
   sudo ./dsim-repos-index/docker/build.sh
   ```
   If you are using nvidia-docker2 add the `--nvidia` option.
   ```sh
   sudo ./dsim-repos-index/docker/build.sh --nvidia
   ```
   ---
   **NOTE**: `build.sh --help` for more options:
      1.  `-in` `--image_name`	Name of the image to be built (default maliput_ws_ubuntu)
      1.  `-ws` `--workspace_folder`	Name of the workspace folder (default maliput_ws)
   ---

1. Create the workspace folder, `maliput_ws` by default:
   ```sh
   mkdir -p maliput_ws
   ```
   ---
   **NOTE**:
   Instructions assumes that `maliput_ws` folder is at the same level as the cloned repository folder `dsim-repos-index`.

   ---

1. Copy `dsim-repos-index/tools/install_dependencies` file into `maliput_ws` workspace folder.
    It will be used to install all the system and build system dependencies.
   ```sh
   cp dsim-repos-index/dsim.repos maliput_ws/.
   ```
1. Copy `dsim-repos-index/dsim.repos` file into `maliput_ws` workspace folder.
    It will be used to bring all the repositories later on.
   ```sh
   cp dsim-repos-index/dsim.repos maliput_ws/.
   ```
1. Run the container.
   ```sh
   sudo ./dsim-repos-index/docker/run.sh
   ```
   If you are using nvidia-docker2 add the `--nvidia` option.
   ```sh
   sudo ./dsim-repos-index/docker/run.sh --nvidia
   ```
    ---
    **NOTE**:
    `run.sh --help` for more options:
      1.	`-in` `--image_name`	Name of the image to be run (default maliput_ws_ubuntu)
      1.	`-cn` `--container_name`	Name of the container(default maliput_ws)
    ---
1. Install dependencies.
   ```sh
   sudo ./install_dependencies.sh
   ```
1. Bring/update all the repositories in your workspace.
   ```sh
   mkdir -p src
   vcs import src < dsim.repos  # clone and/or checkout
   vcs pull src  # fetch and merge (usually fast-forward)
   ```

   This will clone repositories and/or checkout branches, tags or commits as necessary,
   followed by fetching and (likely) fast-forward merging to get branches up to date with
   their upstream counterpart. No merging takes place when a repository is at a given tag
   or commit. Also, note that you can equally bring other repositories as well by repeating
   this `import` and `pull` operation using additional `.repos` files.

1. Install drake.
    ```sh
    sudo ./src/drake_vendor/drake_installer
    ```

1. Install all packages' dependencies:

   ```sh
   rosdep update
   rosdep install -i -y --rosdistro $ROS_DISTRO --skip-keys "ignition-transport7 ignition-msgs4 ignition-math6 ignition-common3 ignition-gui0 ignition-rendering2 libqt5multimedia5 pybind11" --from-paths src
   ```

   Warning
   :   Package dependencies are installed system wide. `rosdep` does not provide any support
       to remove the dependencies it brings. In this regard, disposable containerized workspaces
       help keep development environments clean (as system wide installations within a container
       are limited to that container).

1. Source ROS environment:

   ```sh
   source /opt/ros/$ROS_DISTRO/setup.bash
   ```


 **Create a non-containerized workspace**

  TODO.

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

# Contributing

## Usual workflow

  TODO

## Using binary underlays
  TODO

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

Using .repos files does half the work by allowing codebase version pinning.
Containerized workspaces do the other half along with the `wsetup` tool, and thus
their use is encouraged. The tool itself does not setup workspaces but generates a
script that does the heavy lifting. Said script can be retrieved instead of executed
by means of the `-o` flag:

```sh
dsim-repos-index/tools/wsetup -o setup.sh [...other-args...]
```
