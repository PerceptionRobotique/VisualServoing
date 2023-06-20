Visual Servoing

Repository containing the routines to launch the visual servoing based on [LibPeR](https://github.com/PerceptionRobotique/libPeR) under different use cases.

The directories are organized under three different categories:

- `PGM_Perspective_VisualServoing` which refers to eye-in-hand photometric Gaussian Mixtures-based visual servoing
- `PGM_Perspective_VisualServoing_EyeToHand` which refers to eye-to-hand photometric Gaussian Mixtures-based visual servoing

In either case, all provided servoing are based on the minimization of the cost function computed as the sum of squared differences between a reference and a desired image.

The programs are able to perform the servoing on images of a perspective camera, whose intrinsic parameters are described in an xml file compatible with the libPeR format (download examples here: http://mis.u-picardie.fr/~g-caron/data/PeR/2020_PGM_Perspective_VisualServoing_media.zip 
).

```
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
