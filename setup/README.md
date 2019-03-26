# Delphyne

## How to create the workspace

### How the building script works

Before installing anything, you should know how the script **create_environment.py** works. It has the following flags:
    * -p <path> : Path to create the workspace. By default it uses the current directory.
    * -d <name> : Docker image name. By default it uses delphyne:18.04
    * --use_drake_binary : By default it will use source files
    * --use_ignition_source : By default it will use binaries.
    * --no_docker : By default it will create a docker image. Use it if you want to install everything on the current user.

**Example:** `./create_environment.py -p /home/foo/bar -d delph --use_drake_binary`

### Using a docker image

You **MUST** install [nvidia-docker2](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)).

1. Run `create_environment.py` with the flags you want (remember to install [nvidia-docker2](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0)) before)

### Using the current user

1. mkdir -p <path_to_workspace>
2. mkdir -p <path_to_workspace>/src
3. cd <path_to_workspace>
4. git clone https://github.com/ToyotaResearchInstitute/dsim-repos-index
5. cd dsim-repos-index/setup
6. `./create_environment.py -p <path_to_workspace> --no_docker <options>` 

You finished! Everything is ready! :)

**IMPORTANT**: In case you close your shell, you'll have to run `source <path_to_workspace>/dsim-repos-index/setup/delphyne/env.bash` next time. This file will set up some environment variables that will enable you to run delphyne properly.

## How to build Delphyne

1. Go to your workspace folder. 
    - If you created a docker container, run `./start_ws.sh`. You should be on your container now sitting on your home directory. `cd` to your workspace directory.
2. Run `colcon build --packages-up-to delphyne-gui` (in case you decided to use **drake's source**, then run: `colcon build --cmake-args -DWITH_PYTHON_VERSION=3 --packages-up-to delphyne-gui`)
3. Once it finishes, run `source install/setup.bash`. **DON'T FORGET TO DO THIS, OTHERWISE NO DEMO WILL WORK.**

You can checkout our examples by typing delphyne on your shell and press **TAB** to display the options.
**Example**: `delphyne-gazoo`

Everything is ready for you to start developing! :)

**Note:** Everytime you stop the container and run it again (or close your shell), you'll have to run `source install/setup.bash` again

Additional info can be found in the [Delphyne Guide](https://docs.google.com/document/d/1tQ9vDp084pMuHjYmtScLB3F1tdr4iP9w7_OTcoSM1zQ).

