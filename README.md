# Table of Contents

- [Introduction](#introduction)
- [Workspaces](#workspaces)
  - [Supported platforms](#supported-platforms)
  - [Prerequisites](#prerequisites)
  - [Usage Instructions](#usage-instructions)
    - [Create a workspace](#create-a-workspace)
    - [Bring up your workspace](#bring-up-your-workspace)
    - [Update your workspace](#update-your-workspace)
    - [Check your workspace](#check-your-workspace)
    - [Build your workspace](#build-your-workspace)
    - [Test your workspace](#test-your-workspace)
    - [Delete your workspace](#delete-your-workspace)
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

* Docker containerized workspaces (**recommended**): only Nvidia-powered
  machines running an `nvidia-docker`
  [supported host OS](https://github.com/NVIDIA/nvidia-docker#quickstart) are
  supported. When setting up docker, do *not* add yourself to the "docker" group
  since that represents a security risk
  ([it is equivalent to password-less `sudo`](https://docs.docker.com/install/linux/linux-postinstall/#manage-docker-as-a-non-root-user)).
  The instructions below use `sudo -g docker [command]` as a workaround.

* Non-containerized workspaces: Ubuntu Bionic Beaver 18.04 LTS only.


## Prerequisites

* To get all necessary tools and repos files, clone this repo locally. Tools require
  Python 3.5 or superior.

* To pull private repositories, the current user default SSH keys will be used
  (and thus assumed as both necessary and sufficient for the purpose).

* Containerized workspaces require `nvidia-docker2` by default, so ensure you have
  an NVIDIA card and drivers are properly installed.

* All system dependencies required by the tools included in this repository can
  be installed by running:

  ```sh
  cd dsim-repos-index
  sudo ./tools/prereqs-install .
  ```

  To install `nvidia-docker2`, also run:

  ```sh
  cd dsim-repos-index
  sudo ./tools/prereqs-install -t nvidia .
  ```

## Usage instructions

In the following, it is assumed that you want to create a workspace containing
all of DSIM's (Driving Simulation's) repositories, but with a focus on Maliput
and Malidrive development. As such, it suggests the creation of a workspace in a
`maliput_ws` directory and pulling sources from the [`dsim.repos`](dsim.repos)
file. You should choose a name that is appropriate for your intended purpose.

### Create a workspace

To create a containerized workspace, run:

```sh
sudo -g docker dsim-repos-index/tools/wsetup maliput_ws
```

Note
:  The container image will be called maliput_ws:<UNIX timestamp>.
   Using UNIX timestamps for tags reduce the likelihood of name collision
   when multiple workspaces are present on the same host machine.

To create a non-containerized workspace, run:

```sh
sudo dsim-repos-index/tools/wsetup --no-container maliput_ws
```

Setting up non-containerized workspaces requires sudo credentials to carry out all necessary
installations.

Note
:  Bear in mind that using a non-containerized workspace makes reproducing and troubleshooting
   issues harder for others. Thus, we highly recommend that you use a containerized workspace.

If need be, additional prerequisites for the workspace can be supplied upon workspace creation.
Example using a containerized workspace:

```sh
sudo -g docker dsim-repos-index/tools/wsetup -e path/to/custom/prereqs maliput_ws
```

Example using non-containerized workspace:

```sh
dsim-repos-index/tools/wsetup -e path/to/custom/prereqs maliput_ws
```

These operations will setup a workspace, but without any sources yet. Follow the instructions on
how to [update your workspace](#update-your-workspace) to get them for the first time.

### Bring up your workspace

To bring up your workspace, run:

```sh
cd maliput_ws
source bringup
```

You can always leave the workspace by `exit`ing it.

Note
:  Upon exiting a containerized workspace, you'll be prompted as to whether container changes
   should be saved. For the sake of storage efficiency, only save them if you've applied changes
   outside the workspace directory and to the container filesystem itself (e.g. you installed a
   new package or tool using `apt`) and you wish to keep them.

In certain cases e.g. after installing packages, you may want to reset the workspace shell
environment as if you had just brought it up. To do so, from within the workspace run:

```sh
bounce
```

Note
:  Bouncing a workspace **will not save it**. To save, you must exit the workspace.

### Update your workspace

Whether you are doing this for the first time or updating an existing workspace, the same procedure
applies.

1. Copy the latest [`dsim.repos`](dsim.repos) file into your workspace:

   ```sh
   cp dsim-repos-index/dsim.repos maliput_ws/.
   ```

2. [Bring up your workspace.](#bring-up-your-workspace)

3. Update all repositories in your workspace:

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

4. Install all packages' prerequisites, including drake and ignition binaries:

   ```sh
   sudo prereqs-install -t all src
   ```

   Warning
   :   Package prerequisites are satisfied system wide. `prereqs-install` does not provide
       any support to undo its effects. In this regard, disposable containerized workspaces
       help keep development environments clean (as system wide installations within a container
       are limited to that container).

   You will need to `bounce` your workspace for installation to take effect. Alternatively, you may
   exit and re-enter your workspace -- just **make sure changes are saved** if you do so.

   Also, check each package `prereqs` file to see what other tags are available and their
   implications. For instance, if building drake from source and using ignition binaries,
   you may want to run:

   ```sh
   sudo prereqs-install -t default -t ignition src
   ```

   Likewise, if building ignition from source and using drake binaries, run:

   ```sh
   sudo prereqs-install -t default -t drake src
   ```

5. Install all packages' dependencies:

   ```sh
   rosdep update
   rosdep install -i -y --rosdistro $ROS_DISTRO --skip-keys "ignition-transport5 ignition-msgs2 ignition-math6 ignition-common3 ignition-gui0 ignition-rendering2 libqt5multimedia5 PROJ4 pybind11" --from-paths src
   ```

   Warning
   :   Package dependencies are installed system wide. `rosdep` does not provide any support
       to remove the dependencies it brings. In this regard, disposable containerized workspaces
       help keep development environments clean (as system wide installations within a container
       are limited to that container).

   You will need to `bounce` your workspace for installation to take effect. Alternatively, you may exit
   and re-enter your workspace -- just **make sure changes are saved** if you do so.

   If having issues with this step, make sure the `$ROS_DISTRO` environment variable is defined. If not, first ensure
   you didn't skip step 4, including either bouncing the workspace or exiting and re-entering it while saving all
   changes in the process, to make sure that prerequisites installation took effect.

6. When exiting the workspace, **make sure changes are saved**!

The above sequence allows for full workspace updates, but it's not binding. A conscious user may want
to only update dependencies for a patch he's working on (and thus, only steps 4 and 5 apply, and
maybe not even both depending on if the dependency was declared in a `package.xml` or is being brought
by a `prereqs` file) or even customize the workspace for a one time use. If in a containerized workspace,
upon exit one may choose to not save any modifications and keep the environment clean.  Note that this does
not apply to the workspace directory itself, as it exists outside and beyond the container lifetime, but
since repositories are versioned, changes can be checked out, stashed or even reverted. In extreme cases,
setting up disposable workspaces remains an option.

### Check your workspace

Workspace state as a whole encompasses both current local repositories' state plus the state of
the filesystem that hosts it. However, if a workspace is containerized and no customizations are
applied by the user, repositories alone carry the source code and state the list of system dependencies
necessary to build and execute. And we can easily inspect repositories.

1. [Bring up your workspace](#bring-up-your-workspace)

2. To check repositories' status, run:

   ```sh
   vcs status src
   ```

3. To see changes in the repositories' working tree, run:

   ```sh
   vcs diff src
   ```

4. To see if (most of) our versioned packages' dependencies have been met, run:

   ```sh
   rosdep check --rosdistro $ROS_DISTRO --skip-keys "ignition-transport5 ignition-msgs2 ignition-math5 ignition-common2 ignition-gui0 ignition-rendering0 libqt5multimedia5 pybind11 PROJ4" --from-paths src
   ```

   Note though that currently not all workspace prerequisites are nor can be dealt with using `rosdep`
   alone and thus `rosdep check` may fall short. When it comes down to pure binary dependencies, `drake`'s
   binary tarball is a good example, but prerequisites may go beyond that, `apt` source lists being another
   good example. See `prereqs` executable files in each repository for further details on what's currently
   being handled outside `rosdep`.

In any given case, one can always resort to the specific tool used for repository versioning (e.g. `git`)
if `vcs` isn't enough or to the specific package managers (e.g. `apt` or `pip`) if `rosdep` isn't enough.

### Build your workspace

1. [Bring up your workspace](#bring-up-your-workspace)

2. Build the workspace, which can be done in full or partially.

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

3. Source the workspace:

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
colcon test --event-handlers=console_direct+ --return-code-on-test-failure --packages-skip PROJ4 pybind11
```

Note
:  See [`colcon` documentation](https://colcon.readthedocs.io/en/released/user/how-to.html#run-specific-tests)
for further reference on `test` support.

### Delete your workspace

You may also dispose of a workspace. That is, to dispose of the container image, if any, and all workspace
specific files e.g. the `bringup` script. Once inside of your workspace, run:

```sh
nuke
```

Note that all files and directories added by the user within the workspace directory itself, like the typical
`build`, `install`, `log` and `src` directories, will not be affected by this operation. To get rid of those
as well, simply remove the workspace directory once outside the workspace:

```sh
rm -rf maliput_ws
```

**Warning**
:  You'll be prompted twice for confirmation. This is a permanent removal. It cannot be undone.

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

Note
: In all three cases above, the tools delegate the actual work to the right tool for each package and
focus instead on bridging the gap between them. Thus, for instance, `colcon` builds interdependent
CMake packages by running `cmake` and `make` in the right order and setting up the environment for
the artifacts to be available. Same applies for `vcs` and `rosdep`.

Note
: These tools do not strive to act like a proxy for every configuration setting or command line option
that underlying tools they delegate work to may have. Thus, it may be necessary to configure the underlying
tool in addition to the configuration for these tools to attain a desired behavior. For instance, limiting
`colcon` parallelism with the `--parallel-workers` switch has no impact on `make` parallelization settings
if this tool is being used.

In addition to these tools, we also count on `wsetup` to standardize and simplify the setup of our
development workspaces by means of containerization, and `prereqs-install` to deal with all the non-standard
preconditions that our packages introduce but `rosdep` cannot satisfy. **These tools are not part of the
standard ROS2's development workflow**, and therefore their usage and extension should be sparse at best.

Warning
:   Tools like `prereqs-install` and `rosdep` abstract away platform specific details only for the simplest dependency
    management tasks. To deal with more complex situations like version downgrading or even conflicts, one must fall back
    to the appropriate package manager e.g. `apt`.

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

2. [Bring up your workspace.](#bring-up-your-workspace)

3. Extract binary underlay tarball:

   ```sh
   sudo mkdir -p /opt/dsim-desktop
   sudo tar -zxvf dsim-desktop-latest-bionic.tar.gz -C /opt/dsim-desktop --strip 1
   ```

4. Install all underlay packages' prerequisites, including drake and ignition binaries:

   ```sh
   sudo prereqs-install -t all /opt/dsim-desktop
   ```

   You will need to exit and re-enter the workspace for installation to take effect.
   **Make sure changes are saved upon leave!**

5. Install all underlay packages' dependencies:

   ```sh
   rosdep update
   rosdep install -i -y --rosdistro $ROS_DISTRO --skip-keys "ignition-transport5 ignition-msgs2 ignition-math6 ignition-common3 ignition-gui0 ignition-rendering2 libqt5multimedia5 pybind11 PROJ4" --from-paths /opt/dsim-desktop/*
   ```

6. When exiting the workspace, make sure changes are saved!

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
- [ToyotaResearchInstitute/maliput](https://github.com/ToyotaResearchInstitute/maliput/), that contains packages introducing `maliput`, a road network runtime interface, and reference backends: `dragway` and `multilane`.
- [ToyotaResearchInstitute/maliput-integration](https://github.com/ToyotaResearchInstitute/maliput-integration/), that contains integration examples and tools that unify `maliput` core and its possible backends.

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
