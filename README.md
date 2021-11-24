[![Contributors][contributors-shield]][contributors-url]
[![Stargazers][stars-shield]][stars-url]
[![Forks][forks-shield]][forks-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

# Introduction
GammaGo is an interactive go game system that integrates image recognition with robot arm control. Game rule depends on the AI algorithm used, which is currently Gomoku.

This repository contains all the necessary materials to reproduce our GammaGo system:
- Hardware design and materials we used. (Model files for 3D printing, etc.)
- Image sensing code to find stone positions on the go board. (Both pure python and ROS python version provided.)
- Control code for the robot arm (Using ROS).

Contents within this repository are under MIT license, feel free to use it for your own project. 

The purpose of creating this repository is to provide an easy way for everyone to start a robotics project. So you can suffer less pain on setting up the environment, figuring out how to even make a motor move. Instead, you can spend more time making your own ideas come true. It will be great if you can help add new features here. We will add you to our contributor list.

# Contribution Guidelines
1.  **Fork** this repo
2.  **Clone** the forked repo to your local system
3.  **Commit** changes locally
4.  **Push** your commits to the forked repo
5.  Submit a **Pull Request** and **Request Review** from us

# Overview

### Highlights

### Issues
- When motors overheat, their strength are not enough
- Image recognition does not work well consistently for a very reflective board surface

### Future Development
- **Mechanical**
  - Use gear set to reduce the torque required from motors
  - Use stronger motors and 3D-printing materials
  - Improve the stone feeder
- **Software**
  - Improve image recognition for reflective board sufaces
- **Control**
  - Clean the board after a game completes
  - Replay past games
- **Others**
  - Display visible signs when a game ends

# Steps to Set Up the Project

# Contributors

<a href="https://github.com/JinZihang/GammaGo/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=JinZihang/GammaGo" />
</a>

Special thanks to [ros_sony_cam](https://github.com/arcoslab/ros_sony_cam) and [all-contributors](https://github.com/all-contributors/all-contributors).

[contributors-shield]: https://img.shields.io/github/contributors/JinZihang/GammaGo?style=for-the-badge
[contributors-url]: https://github.com/JinZihang/GammaGo/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/JinZihang/GammaGo?style=for-the-badge
[forks-url]: https://github.com/JinZihang/GammaGo/network/members
[stars-shield]: https://img.shields.io/github/stars/JinZihang/GammaGo?style=for-the-badge
[stars-url]: https://github.com/JinZihang/GammaGo/stargazers
[issues-shield]: https://img.shields.io/github/issues/JinZihang/GammaGo?style=for-the-badge
[issues-url]: https://github.com/JinZihang/GammaGo/issues
[license-shield]: https://img.shields.io/github/license/JinZihang/GammaGo?style=for-the-badge
[license-url]: https://github.com/JinZihang/GammaGo/blob/main/LICENSE
