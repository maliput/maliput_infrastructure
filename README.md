# Introduction

This repository contains `.repos` files and tools, that enable the creation and 
maintenance of development workspaces. Each `.repos` file helps bringing a subset
of the packages in the `dsim` ecosystem. 

For instance, while `maliput.repos` pulls all `maliput` packages on road network
descriptions, plus the `malidrive` backend package and `delphyne` packages for
visualization and prototyping, `malidrive.repos` leads to a smaller workspace
with `maliput` and `malidrive` packages only.

# Workspace Usage

## Supported platforms

* For regular workspaces, only Ubuntu Bionic Beaver 18.04 LTS is supported as
  the host OS.

* For dockerized workspaces, only nvidia powered machines are supported as host
  machines.

## Prerequisites

* To get all necessary tools and repos files, clone this repo locally.

* To pull private repositories, the current user default SSH keys will be used
  (and thus assumed as both necessary and sufficient for the purpose).

* Containerized workspaces require `nvidia-docker2` by default, so ensure you have
  an NVIDIA card and drivers are properly installed.

* All system dependencies required by the tools included in this repository can
  be installed by running:

  ```sh
  cd dsim-repos-index
  ./tools/prereqs-install .
  ```
 
  To install `nvidia-docker`, also run:
  
  ```sh
  cd dsim-repos-index
  ./tools/prereqs-install -t nvidia .
  ```

## Create a workspace

To create a regular (dockerized) workspace, run:

```sh
dsim-repos-index/tools/wsetup dsim_ws
```

Note
:  The docker image will be called dsim-devel:<UNIX timestamp>. 
   Using UNIX timestamps for tags reduce the likelihood of name collision
   when multiple workspaces are present on the same host machine.

To create a non dockerized workspace, run:

```sh
dsim-repos-index/tools/wsetup --no-container dsim_ws
```

Note
:  Bear in mind that non dockerized workspace make reproducing and 
   troubleshooting issues harder for others.

Both operations will setup a ROS2-like workspace. If need be, additional prerequisites
for the workspace can be supplied upon workspace creation:

```sh
dsim-repos-index/tools/wsetup -e path/to/custom/prereqs dsim_ws
```

## Bringup your workspace 

To bring up your workspace, run:

```sh
cd dsim_ws
source bringup
```

You can always leave the workspace by `exit`ing it.

Note
:  Upon exiting a containerized workspace, you'll be prompted as to whether container changes 
   should be persisted or not. For the sake of storage efficiency, only persist them if changes
   have been applied to the container filesystem itself but not just the workspace.

## Update your workspace

Whether you are doing this for the first time or updating
an existing workspace, the same procedure applies:

1. Copy a .repos file into your workspace, e.g. our default `maliput.repos`:

   ```sh
   cp dsim-repos-index/maliput.repos dsim_ws/.
   ```

2. Bring up the workspace to be updated:

   ```sh
   source bringup
   ```

3. Update all repositories in your workspace:

   ```sh
   mkdir -p src
   vcs import src < maliput.repos
   vcs pull src
   ```
   
   Note that you can equally bring other repositories as well by importing from more
   `.repos` files.

4. Install all packages' prerequisites, including drake and ignition binaries:

   ```sh
   sudo prereqs-install -t all src
   ```

   Check each package `prereqs` file to see what other tags are available and their 
   implications. For instance, if building drake from source and using ignition 
   binaries, you may want to run:

   ```sh
   sudo prereqs-install -t default -t ignition src
   ```

   Likewise, if building ignition from source and using drake binaries, run:

   ```sh
   sudo prereqs-install -t default -t drake src
   ```

5. Leave and re-enter the workspace for installation to take effect.
   **Make sure changes are persisted upon leave.**

6. Install all packages' dependencies.

   ```sh
   rosdep update
   sudo rosdep install -i -y --rosdistro $ROS_DISTRO --skip-keys "ignition-transport5 ignition-msgs2 ignition-math5 ignition-common2 ignition-gui0 ignition-rendering0 libqt5multimedia5 pybind11 PROJ4" --from-paths src
   ```

## Build your workspace

1. Bring up the workspace to build on:

   ```sh
   source bringup
   ```

1. Build all packages:

   ```sh
   colcon build
   ```

   If you are building `drake` from source as well, run instead:

   ```sh
   colcon build --cmake-args -DWITH_PYTHON_VERSION=3
   ```

6. Source the workspace:

   ```sh
   source install/setup.bash
   ```

Note
: If `delphyne` is available, we recommend you to run `delphyne-gazoo` and `delphyne-mali` (type them on 
your terminal) to see if everything is properly working.

## Test your workspace

In a built workspace, run:

```sh
colcon test --event-handlers=console_direct+ --return-code-on-test-failure --packages-skip PROJ4 pybind11
```

# Contributing

## Usual workflow

Ours is similar to ROS2's development workflow, and thus many of their tools and practices apply equally.

Workspaces are managed via [`vcs`](https://github.com/dirk-thomas/vcstool), a tool that helps in dealing with
sources distributed across multiple repositories, not necessarily versioned with the same tool (support for `git`,
`hg`, `svn` and `bazaar` is readily available). `vcs` uses `.repos` files for a listing of version pinned sources.

Dependency management is taken care of by [`rosdep`](https://docs.ros.org/independent/api/rosdep/html/commands.html),
a tool that can crawl `package.xml` files and resolve found dependencies into a call to the appropriate package 
manager for the current platform by means of a public database known as rosdistro.

To build and test packages, [`colcon`](https://colcon.readthedocs.io/en/released/) abstracts away the details of the
specific build system and testing tools in use and arbitrates these operations to take place in topological order.

Note
:  In all three cases above, the tools delegate the actual work to the right tool for each package and 
focus instead on bridging the gap between them. Thus, for instance, `colcon` builds interdependent 
CMake packages by running `cmake` and `make` in the right order and setting up the environment for
the artifacts to be available. Same applies for `vcs` and `rosdep`.

In addition to these tools, we also count with `wsetup`, to standardize and simplify the setup of our 
development workspaces by means of containerization, and `prereqs-install` to deal with all the non-standard
preconditions that our packages introduce and `rosdep` cannot deal with. These tools **are not part of the 
standard workflow**, and therefore their usage and extension of these tools should be sparse at best.

## How to use CI

CI jobs build and test relevant packages for each repository on every PR.
Being a multiple repositories' project, patches that are not limited to
a single repository must be separately PR'd but built and tested together.
To that end, make sure that all PR'd branches that are part of the same patch
have the same name e.g. `my_github_user/my_patch_name`.

**Warning**
:  Fork based development is currently not supported.

# Troubleshooting

## Issue Forensics

When reproducing issues, either related to the codebase or to the infrastructure
that supports it, recreating the environment in which these issues arose is crucial.

Using .repos files does half the work by allowing codebase version pinning.
Dockerized workspaces do the other half along with the `wsetup` tool, and thus 
their use is encouraged. The tool itself does not setup workspaces but generates a 
script that does the heavy lifting. Said script can be retrieved instead of executed
by means of the `-o` flag:

```sh
dsim-repos-index/tools/wsetup -o setup.sh [...other-args...]
```
