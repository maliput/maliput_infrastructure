# Introduction

This repository contains .repo files and tools, that enable the creation and 
maintenance of workspaces. For example, `malidrive.repos` can be used to create 
a workspace that enables Maliput and Malidrive development only.

# Workspace Usage

## Supported platforms

* For regular workspaces, only Ubuntu Bionic Beaver 18.04 LTS is supported as
  the host OS.

* For dockerized workspaces, only NVidia powered machines are supported as host
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
  ./tools/prereqs-install -t .
  ```
 
  To also install `nvidia-docker`, run:
  
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

4. Install **all** packages' prerequisites (this includes drake binaries):

   ```sh
   sudo prereqs-install -t all src
   ```

5. Leave and re-enter the workspace for installation to take effect.
   **Make sure changes are persisted upon leave.**

6. Install all packages' dependencies.

   ```sh
   rosdep update
   sudo rosdep install -i -y --from-paths src --rosdistro $ROS_DISTRO --skip-keys "ignition-transport5 ignition-msgs2 ignition-math5 ignition-common2 ignition-gui0 ignition-rendering0 pylint3 pycodestyle libqt5multimedia5 libboost-filesystem-dev pybind11 PROJ4"
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
: We recommend you to run `delphyne-gazoo` and `delphyne-mali` (type them on your terminal) to see if everything is properly working.

## Test your workspace

In a built workspace, run:

```sh
colcon test --event-handlers=console_direct+ --return-code-on-test-failure --packages-skip PROJ4 pybind11
```

# Contributing

## How to use CI

CI jobs build and test relevant packages for each repository on every PR.
Being a multiple repositories' project, patches that are not limited to
a single repository must be separately PR'd but built and tested together.
To that end, make sure that all PR'd branches that are part of the same patch
have the same name e.g. `my_github_user/my_patch_name`.

Note
:  Personal repository forks are currently not supported.

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
dsim-repos-index/wsetup -o setup.sh [...other-args...]
```
