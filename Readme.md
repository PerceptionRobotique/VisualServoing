[![Github Releases](https://img.shields.io/github/release/PerceptionRobotique/VisualServoing.svg)](https://github.com/PerceptionRobotique/VisualServoing/releases)

# Visual Servoing

Repository containing the routines to launch the visual servoing based on [LibPeR](https://github.com/PerceptionRobotique/libPeR_base) under different use cases.

Two kinds of approaches are presented:

- `PGM` (Photometric Gaussian Mixtures)
- `MPP` (Mixture of Photometric Potentials)

PGM based visual servoing example is available in two different versions:

- `PGM_Perspective_VisualServoing` which refers to eye-in-hand photometric Gaussian Mixtures-based visual servoing
- `PGM_Perspective_VisualServoing_EyeToHand` which refers to eye-to-hand photometric Gaussian Mixtures-based visual servoing

## Dependencies

### Robots and cameras

- `PGM` visual servoing example relies on an `UR10` robot and a `Flir` camera. The corresponding libraries are built directly with the project via api calls to corresponding hardware libraries.
- `MPP` visual servoing example relies on a `Franka Emika Panda` robot and a `Kodak Pixpro` camera. The user needs to install the `libFranka` library (see [the dedicated install page](https://frankaemika.github.io/docs/installation_linux.html)). On the other hand, the `Kodak` camera is controlled through the `OpenCV` library via basic camera handling.

### Visual Servoing dependencies

The Visual Servoing control law is based on [LibPeR](https://github.com/PerceptionRobotique/libPeR_base) and consists of the minimization of the cost function computed as the sum of squared differences between a reference and a desired image. The list below summarizes the different needed libraries:

- [LibPeR](https://github.com/PerceptionRobotique/libPeR_base)
- [ViSP](https://visp.inria.fr/)
- Boost

## How to use

### Compilation

To run one of the available examples, build the executable in the desired directory using:


```bash
mkdir build && cd build
cmake ..
make -j12
```

### PGM-based Visual Servoing

The programs are able to perform the servoing on images of a perspective camera, whose intrinsic parameters are described in an `.xml` file compatible with the libPeR format (download examples here: [2020_PGM_Perspective_VisualServoing_media](http://mis.u-picardie.fr/~g-caron/data/PeR/2020_PGM_Perspective_VisualServoing_media.zip)).

This method was originally presented in

> Crombez, N., Mouaddib, E. M., Caron, G., & Chaumette, F. (2018). Visual servoing with photometric gaussian mixtures as dense features. IEEE Transactions on Robotics, 35(1), 49-63.

### MPP-based Visual Servoing

The program relies on two successives expansion parameters to achieve a rough positioning and a more precise one (typically 0.2 and 0.03).

Furthermore, the transformation between the end-effector of the robot and the camera needs to be given (as a `.yaml` file).

A `bash` example can be found in the `MPP` directory to launch the Visual Servoing task with a basic setup.

## Credits

```text
Copyright (C) 2017-2023 by MIS lab (UPJV). All rights reserved.

See http://mis.u-picardie.fr/~g-caron/fr/index.php?page=7 for more information.

This software was developed at:
MIS - UPJV
33 rue Saint-Leu
80039 AMIENS CEDEX
France

and at
CNRS - AIST JRL (Joint Robotics Laboratory)
1-1-1 Umezono, Tsukuba, Ibaraki
Japan

This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

Description:
Insight about how to set the project and build the program
Authors:
Guillaume CARON

```
