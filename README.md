# SVEA Starter Suite

### Quicklinks:
- [SVEA Website](https://svea.eecs.kth.se)
- [SVEA Docs](https://kth-sml.github.io/svea)
- [Tutorials](https://kth-sml.github.io/svea/tutorials/0_intro)

## A short description
This repo contains a basic library of python objects and scripts to make
development on the Small-Vehicles-for-Autonomy (SVEA) platform simpler
and cleaner.

The design principle of this library is to help create projects that are
more modular and easier to troubleshoot. As opposed to the standard
approach of creating a large web of Subscriber/Publisher nodes, we modularly
wrap different ROS entities in Python objects, while exposing only the useful
features with object-oriented interfaces.

## Useful to know before starting
Before continuing to the next sections, consider taking some time to read up on
two important concepts for this code base: the **Robotic Operating System (ROS 2)**
and **Object Oriented Programming (OOP)**.

To read up on ROS 2, check out the
[ROS Start Guide](https://docs.ros.org/en/jazzy/index.html#getting-started). However, do not spend
too much time diving into the guide. The structure and tutorials are not very
intuitive, but glossing over them will give a sense of what ROS is and how you
are meant to use it. The rest of the learning curve is overcome by trying it out
yourself.

To read up on OOP, check out Real Python's
[introduction on OOP](https://realpython.com/python3-object-oriented-programming/).

# Installation

In the instructions below, we describe the system requirements, installation
process, and run instructions using Docker. If you want to set up this code
base natively, then check out our [Building the SVEA Stack Natively](docs/development/native_build.md)
instructions.

## Install Docker Engine
For the instructions below to work, you need to first install Docker Engine.

For Windows users, you will need to install WSL 2 and set it's distribution to
Ubuntu (20 or higher recommended). Use the commands listed [here](https://learn.microsoft.com/en-us/windows/wsl/basic-commands)
to "Install", "List available Linux distributions", and "Set default WSL Version" to
Ubuntu 20 or higher.

To install Docker Engine, follow the instructions for your operating system
[here](https://docs.docker.com/engine/).

## Installing the Docker image
Start by going to the folder where you want the code to reside.
For example, choose the home directory or a directory for keeping projects in.
Once you are in the chosen directory, use the command:

```bash
git clone https://github.com/KTH-SML/svea.git
```

to download the library. Then, a new directory will appear called
`./svea`. Go into the directory with command:

```bash
cd svea
```

To build the Docker image containing the entire codebase run:

```bash
docker compose build
```

If it all runs without an error, you have installed the Docker image!

## Installing Foxglove Studio
For visualization, we recommend Foxglove Studio. To install Foxglove Studio,
follow the instructions for your operating system [here](https://foxglove.dev/download)

**Note**: alternatively, you can use the Web version of Foxglove Studio which is
also available from the installation link.

# Usage
The intended workflow with the code base is as follows:
1. Write new features/software
2. Debug the new contributions in simulation
3. Perform basic tuning and adjustments in simulation
4. Evaluate actual performance on a SVEA car

The simulated vehicles provide identical interfaces and information patterns
to the real SVEA cars, thus by following this workflow, development work
should always start in simulation and code can be directly ported to the real
cars. However, this does not mean the code will work on a
real vehicle without further tuning or changes.

There are pre-written scripts to serve as examples of how to use the
core library. See and read the source code in
[svea_examples/scripts](src/svea_examples/scripts).

Start the default container:

```bash
docker compose up -d
```

Then enter a shell in the running container:

```bash
docker compose exec svea bash
```

Then, for a simulated, pure pursuit example, call:

```bash
ros2 launch svea_examples floor2.launch
```

### Common Docker/Compose commands

Start container in background:

```bash
docker compose up -d
```

Stop and remove container/network:

```bash
docker compose down
```

Enter container shell:

```bash
docker compose exec svea bash
```

Rebuild image only when image-layer files changed (e.g. `docker/Dockerfile`,
`requirements.txt`, `entrypoint`, apt/rosdep setup):

```bash
docker compose build
```

Build and start in one command after image-layer changes:

```bash
docker compose up -d --build
```

After changing or adding anything in `src/` (new packages, launch files, nodes),
rebuild the ROS workspace inside the running container:

```bash
docker compose exec svea bash
colcon build --symlink-install
source /svea_ws/install/setup.bash
```

Then, open Foxglove Studio natively or in the browser, and on the first prompt
click "Open connection", then click "Open" with the default settings. Next,
click on the "Layout" dropdown menu and select "Import from file...". Finally,
navigate to `./foxglove` and select `Floor2 Pure Pursuit.json`. After it
finishes loading, you should see something that looks like this:

![purepursuit_foxglove](docs/media/foxglove_pure_pursuit.png)

Now you are ready to read through the tutorials! You can find them
[here](https://kth-sml.github.io/svea/tutorials/0_intro).

## Going from simulation to real

**Note, you only have to follow this section when running the real cars!**

### Adding the low-level interface

To your roslaunch file, add

```xml
<!--open serial connection for controlling SVEA-->
<executable cmd="$(find-pkg-share svea_core)/util/start_micro_ros.sh" output="screen"/>
```

### Running localization on the real SVEA

Running the localization amounts to adding `localize.launch` to your project launch:

```xml
<include file="$(find svea_localization)/launch/localize.launch"/>
```

### Hardware access

The default `svea` service is configured as privileged and mounts `/dev`,
so use the same container for hardware access:

```bash
docker compose up -d
docker compose exec svea bash
```
