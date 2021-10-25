[![Contributors][contributors-shield]][contributors-url]
[![Stargazers][stars-shield]][stars-url]
[![Forks][forks-shield]][forks-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]

# Introduction
GammaGo is an interactive go game system that integrates image recognition with robot arm control.

This repository contains all the necessary materials to reproduce our GammaGo system:
- Hardware design and materials we used. (Model files for 3D printing, etc.)
- Image sensing code to find stone positions on the go board. (Both pure python and ROS python version provided.)
- Control code for the robot arm (Using ROS).

Contents within this repository are under MIT license, feel free to use it for your own project. 

The purpose of creating this repository is to provide an easy way for everyone to start a robotics project. So you can suffer less pain on setting up the environment, figuring out how to even make a motor move. Instead, you can spend more time making your own ideas come true. It will be great if you can help add new features here. We will add you to our contributor list.

# Instructions for Making Changes to this Repository
1. Fork this repository.
2. Make edits on your forked repository.
3. Create a pull request.

# Overview

### Highlights
### Issues
### Future Development
- Mechanical
- Software
  - Improve contour and corner detection to make image recognition work stably on reflective board surfaces.
- Control
  - Correct stones that offset too much.
  - Clean the board after finishing a game.
  - Replay past games.
- Others
  - Display visible signs when a game ends.

# Steps to Set Up the Project
1. Clone this repository using `git clone https://github.com/JinZihang/GammaGo.git`.

# Contributors

**Mechanical Team:** Yuwen, Yuxin

**Software Team:** Bryant, Zihang

**Control Team:** Jianing, Vamsi

**Special Thanks:** [ros_sony_cam](https://github.com/arcoslab/ros_sony_cam) by arcoslab.

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
