#############################################################################
#
# This file is part of the PGM_Perspective_VisualServoing software.
# Copyright (C) 2020 by MIS lab (UPJV). All rights reserved.
#
# See http://mis.u-picardie.fr/~g-caron/fr/index.php?page=7 for more information.
#
# This software was developed at:
# MIS - UPJV
# 33 rue Saint-Leu
# 80039 AMIENS CEDEX
# France
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Insight about how to set the project and build the program
#
# Authors:
# Guillaume Caron
#
#############################################################################

0. create a new directory named build in PGM_Perspective_VisualServoing
1. use cmake (twice) to fill the build directory in, from command line (add `-D USE_UR=True` to use UR robot classes, `-D USE_THETA=True` to use Theta X camera classes, `-D USE_FLIR=True` to use Flir camera classes. Note: for the moment only combinations of THETA+UR or FLIR+UR are possible.): 
	- in the build directory: `cmake path/to/source/dir -DCMAKE_BUILD_TYPE=Release -DPER_DIR=/path/to/libPeR/install/dir -D USE_UR=True -D USE_THETA=True`
2. update the program to match your image acquisition tool (in case you use a Ricoh Theta X: `https://codetricity.github.io/theta-linux/`)
3. open the project in build or use make in the latter directory to build the exe file
4. run the program from the command line...:

./build/Release/PGMCameraVS ./PGM_Perspective_VisualServoing_media/calibration/calib.xml 4 0.325 ./PGM_Perspective_VisualServoing_media/images_resize4/maskFull.png

that are, in the reading order:
- the camera calibration xml file
- the image scale reduction factor 
- the initial Gaussian expansion parameter
- the image file of the mask (white pixels are to be considered whereas black pixels are not)

Sample calibration and mask files are provided in the "PGM_Perspective_VisualServoing_media" archive to be downloaded from here: http://mis.u-picardie.fr/~g-caron/data/PeR/2020_PGM_Perspective_VisualServoing_media.zip, 
or: http://mis.u-picardie.fr/~g-caron/data/PeR/2023_PGM_Omni_VisualServoing_media.zip, or: http://mis.u-picardie.fr/~g-caron/data/PeR/2024_PGM_Equi_VisualServoing_media.zip

Note: validated with libPeR-0.3.0 (PGMPerspVS_levels and PGMPerspVS_test still need to be updated)

