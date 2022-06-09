# AMR-term-project
Aarhus University BSc Course Autonomous Mobile Robots Term Project (Spring 2022)


## Operating System Platform

The code has been developed and tested on Ubuntu-20.04, but should be able to run on Ubuntu-18.04. We 
have created and tested a patch that can be applied to make the code work.
IF you use Ubuntu-18.04 then run the this command to apply the patch.

```sh
git apply ./patches/ubuntu-18.04.patch
```

## Download

The code is structured as a ROS package, and is therefore intended to be downloaded and places into the
`src` folder in a catkin workspace e.g. `~/catkin_ws/src`.

```sh
git clone https://github.com/KJK-group/AMR-term-project.git 
```
 
## Dependencies

### Task 1 and 2

[octomap](https://github.com/OctoMap/octomap)

```sh
sudo apt install "${ROS_DISTRO}-octomap*"
```
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot/tree/46c9d1e2885eca6e3ea095afcfb6a6583260fd95) follow the installation instructions
in the projects README. We have used version `v1.12.3`. 
if you are using Ubuntu-18.04 you might have to use version `v1.11.3`

You can switch versions by checking out the commit tagged with the given version number
```sh
git checkout <tag> # e.g. v1.11.3
```

### Task 3

follow the installation instructions for [pytorch](https://pytorch.org/) on their website. Remember to install the `torchvision` along with `pytorch`.

```sh
pip install matplotlib numpy Pillow
```

## Build
Build the package using `catkin`. Go to the directory where you downloaded this repository and run
```sh
catkin build --this
```

## How to run

### Setup for task 1 and 2

NOTE: this script assumes that the terminal emulator used is `terminator`.
```sh
rossun amr_term_project launch_sim.sh <PX4 root directory> <catkin workspace root e.g. ~/catkin_ws/> 
```

If you are not using `terminator` you have to run the following two scripts in order to start PX4 and mavros.
`./scripts/run.sh <PX4 root directory> <catkin workspace root e.g. ~/catkin_ws/> ` and `roslaunch amr_term_project sim.launch`

### Task 1
```sh
roslaunch amr_term_project pid_test.launch
```

### Task 2
```sh
roslaunch amr_term_project mission.launch
```

### Task 3

For testing the model used in task 3 see the README section in `./model_testing.ipynb`.

## Report
The report can be found in `./paper/amr-term-project.pdf`

## Authors

- Jens Jensen - 201907928@post.au.dk
- Kevork Donato - 201907831@post.au.dk
- Kristoffer Sørensen - 201908140@post.au.dk
