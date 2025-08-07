# Motion-planning-of-the-Jaco-robotic-arm-with-the-goal-of-writing-text-from-speech

Code repository for project: "Motion planning of the Jaco robotic arm with the goal of writing text from speech"

## Overview

This project implements a robotic system that converts spoken words into physical text written by a Jaco robotic arm. The system combines speech recognition, text processing, motion planning, and robotic control to enable the robot to "write" text using a pen attached to its end-effector.

## System Architecture

The project consists of several key components:

### Core Modules

1. **`audio_processor.py`** - Handles audio recording and speech-to-text conversion using OpenAI's Whisper model
2. **`text_to_waypoints.py`** - Converts text into robot waypoints using Hershey fonts for path planning
3. **`robot_drawer.py`** - Controls the Jaco arm movement and path execution using MoveIt!
4. **`visualizer.py`** - Provides RViz visualization for the planned and executed robot trajectories
5. **`start_drawing.py`** - Main application entry point that controls the entire workflow


## System Requirements

**Tested Environment:**
- **OS**: Ubuntu 20.04 LTS
- **ROS Version**: Noetic

**Dependencies:**
- MoveIt!
- OpenAI Whisper
- HersheyFonts library
- sounddevice
- scipy
- numpy

## Installation and Setup

### Prerequisites

1. **Install ROS Kinova packages:**
   ```bash
   # Clone the Kinova ROS repository
    cd ~/catkin_ws/src
    git clone -b <branch-name> git@github.com:Kinovarobotics/kinova-ros.git kinova-ros
    cd ~/catkin_ws
    catkin_make
   ```

2. **Install Python dependencies:**
   ```bash
   pip install whisper sounddevice scipy numpy HersheyFonts
   ```
3. **Install Moveit:**
    ```bash
    sudo apt-get install ros-<distro>-moveit
   ```

## Usage

### Running the System

1. **Launch the Kinova arm:**
   ```bash
    roslaunch <robot_name>_moveit_config <robot_name>_virtual_robot_demo.launch
   ```

2. **In a separate terminal, run the drawing application:**
   ```bash
   rosrun your_package start_drawing.py
   ```
