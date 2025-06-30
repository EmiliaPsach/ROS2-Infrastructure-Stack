# ROS 2 Install Instructions

It is generally recommended to install ROS 2 from Debian packages, and specifically discouraged to install ROS 2 on a macOS machine (both by source and binary) because it is not officially supported.

*Note*: This ROS 2 packages repository should work agnostic of your system's OS and which installer you use to install ROS 2. You may need to adjust your `PYTHONPATH`, along with other system variables, if you are not using a ROS 2 virtual environment.

## Prerequisites

### System characterization

These are my system's specs for installing ROS 2 with [Foxy](https://docs.ros.org/en/foxy/index.html).
*Note*: you may need to install via [Kilted](https://docs.ros.org/en/kilted/Installation/Alternatives/Ubuntu-Development-Setup.html) if you have a newer system (i.e. Ubuntu 24.04).

- **Operating System**: Ubuntu 20.04 LTS (Focal Fossa)
- **CPU**: Intel® Core™ i7-7500U @ 2.70GHz
- **Architecture**: 64-bit (x86_64; "amd64")

### Packages

The [Foxy Ubuntu (deb packages)](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html) instructions have steps to download required packages, but the instructions also implies the user already has a lot of implicitly required software (C++ compiler, Python 3.8, etc.) Follow the error messages while installing ROS 2 Foxy to install any missing packages/software.

## Instructions

1. Install [Foxy Ubuntu (deb packages)](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
    - *Note*: ROS 2 Foxy requires Python v3.8. I have Python v3.12.5, so I had to install ROS 2's dependencies inside a Python virtual environment (`python3 -m venv ~/ros2_venv`) to avoid system conflicts
        - *Note*: those Python pip installs often required the `--break-system-packages` flag 
