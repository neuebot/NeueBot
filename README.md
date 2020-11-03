# NeueBot Controller

The NeueBot Controller is a high-level architecture controller for surgical applications.

# Requirements

- **Ubuntu 16.04**
- **ROS Kinetic** (http://wiki.ros.org/kinetic/Installation/Ubuntu)
- **Eigen 3**
- **CoppeliaSim 3.6.2** (https://www.coppeliarobotics.com/previousVersions)

# Installation

## Setup OROCOS 

Follow the oficial OROCOS documentation at https://docs.orocos.org/docs_main/installation.html.

:point_right: This was tested in a fresh Ubuntu 16.04 install.

For an easier setup, we will install the pre-compiled packages shipped as ROS debian packages,

```bash
$ sudo apt install ros-kinetic-rtt-ros-integration
$ sudo apt install ros-kinetic-kdl-conversions ros-kinetic-kdl-parser ros-kinetic-kdl-typekit
```

## Setup Qt Creator

We followed this guide (https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html) to install a tweaked ROS Qt Creator. It will come in handy to configure the workspace.

In the [ROS Qt Creator Plug-in wiki](https://ros-qtc-plugin.readthedocs.io/en/latest/index.html) page, you can all the information on how to setup your ROS workspace, build and run settings, and much more.

:bulb: Don't forget that you want the **Xenial** version! 

## Setup ~~V-REP~~ CoppeliaSim

We provide the tools to test the controller in a simulated environment - [CoppeliaSim](https://www.coppeliarobotics.com/). For legacy reasons, most of the packages will be named `vrep` something.

To setup the simulation environment you should create a separate directory to build the plugins. 

```bash
$ cd ; mkdir coppelia_ws
$ git clone PROPER_REPO
$ tree -L 2 -d ~/coppelia_ws

coppelia_ws/
├── v_repExtSurgRobotControl
│   ├── include
│   ├── src
│   └── vrep
├── vrep_plugins
│   ├── v_repExtEndEffector
│   └── v_repExtKukaControl
└── vrep_scenes
```

For each of those directories, you want to create a `build/` folder and build the plugin library, e.g.,

```bash 
$ cd ~/coppelia_ws/v_repExtSurgRobotControl
$ mkdir build; cd build; cmake ..
$ make 
```


## Setup Workspace

Before setting up our workspace, just make sure we have all the required dependencies.

### Catkin tools

We can take advantage of the OROCOS ROS integration and use the awesome `catkin_tools` package to simplify our components' installation process. 

Follow their installation process at https://catkin-tools.readthedocs.io/en/latest/installing.html.

### Eigen

As I pointed out in the pre-requisites, make sure you have Eigen3 somewhere in your machine. You can use the debian package,

```bash
$ sudo apt install libeigen3-dev
```

We will need to create a symbolic link as `/usr/include/Eigen` from the `eigen3/Eigen` folder,

```bash
$ sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

### Reflexxes Motion Library

As of this moment, the PTP joint controller is based on the [Reflexxes Type II library](http://www.reflexxes.ws/).

You can find download link for the library at http://www.reflexxes.ws/software/typeiirml/v1.2.6/docs/page__download_instructions.html.

From here, I recommend that you create a directory `~/neuebot_ws/deps` to place any third party dependencies. Then, unzip the contents of the downloaded **ReflexxesTypeII.zip** there.

Go to `~/neuebot_ws/deps/ReflexxesTypeII/Linux` and compile the library,

```bash
$ cd ~/neuebot_ws/deps/ReflexxesTypeII/Linux 
$ make clean64 all64
```

You should end-up with something like this,

```bash
$ tree -L 2 -d ~/neuebot_ws/deps

└── ReflexxesTypeII
    ├── include
    ├── Linux
    ├── MacOS
    ├── src
    └── Windows
```

You will want to create a `*.cmake` file to create the variables that point to the Reflexxes include and library directories. You can run the following commands to do just that,

```bash
$ cd ~/neuebot_ws/deps/ReflexxesTypeII/
$ echo 'set(PREFIX ${CMAKE_CURRENT_LIST_DIR})
set(ReflexxesTypeII_INCLUDE_DIR ${PREFIX}/include)
set(ReflexxesTypeII_LIBRARY_DEBUG ${PREFIX}/Linux/x64/debug/lib/shared/libReflexxesTypeII.so)
set(ReflexxesTypeII_LIBRARY_RELEASE ${PREFIX}/Linux/x64/release/lib/shared/libReflexxesTypeII.so)' > ReflexxesTypeIIConfig.cmake
```

### Finally, the Workspace...

First, create a top-level directory where all the files related to the NeueBot Controller will be stored and clone our repo,

```bash
$ cd ; mkdir -p neuebot_ws/src; cd neuebot_ws
$ git clone <link>
```

We can now use the magic of `catkin_tools` to build our entire list of components,

```bash
$ cd ; cd neuebot_ws
$ catkin init
```

Make sure source your workspace environment, and create an environment variable to your workspace folder,
```bash
$ echo "source ~/neuebot_ws/devel/setup.bash" >> ~/.bashrc
$ echo "export WORKSPACE=~/neuebot_ws/src" >> ~/.bashrc
$ source ~/neuebot_ws/devel/setup.bash
```
**TODO:** Remove the need for the environment variable.

You can list the components visible in your `neuebot_ws/src` folder. Some folders/components have a `CATKIN_IGNORE` empty file. Those are not ignored by catkin.

```bash
$ catkin list

<OUTPUT>
```

You can now compile the listed components,

```bash
$ catkin build
```

Hopefully everything went well! :pray:

# Run the Controller

TODO: We use our own typekits to communicate between components. Although the OROCOS integration using ROS is pretty smooth, this feature is still lacking. We are testing some options to get this to work, will post an update soon.



