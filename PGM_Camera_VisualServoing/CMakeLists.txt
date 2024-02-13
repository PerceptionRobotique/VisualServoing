#############################################################################
#
# This file is part of the libPR software.
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
# libPR overall configuration file. Freely inspired from the CMakeList tree of the 
# ViSP library. 
#
# Authors:
# Guillaume Caron
#
#############################################################################

project(PGM_Perspective_VisualServoing)

cmake_minimum_required(VERSION 3.1)

find_package(PER REQUIRED per_core per_io per_features per_sensor_pose_estimation per_estimation)
if(PER_FOUND)
	include(${PER_USE_FILE})
endif(PER_FOUND)

# ViSP
find_package(VISP REQUIRED  visp_core visp_robot visp_visual_features visp_io visp_gui visp_vs visp_vision)
if(VISP_FOUND)
	include(${VISP_USE_FILE})
endif(VISP_FOUND)

if(${USE_UR})
	find_package(ur_rtde REQUIRED)
endif()

#for linking with libav
if(${USE_THETA})
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
        libavdevice
        libavfilter
        libavformat
        libavcodec
        libswresample
        libswscale
        libavutil
        )

  #IF USE UVC Theta X
  INCLUDE_DIRECTORIES(${UVCTHETA_DIR}/include)
  LINK_DIRECTORIES(${UVCTHETA_DIR})
endif()

#for compiling and linking with Flir
if(${USE_FLIR})
  #INCLUDE_DIRECTORIES(/usr/include/flycapture)
	INCLUDE_DIRECTORIES(/opt/spinnaker/include/)
	LINK_DIRECTORIES(/opt/spinnaker/lib/)
endif()


set(PGMPerspVS_cpp
  PGMCameraVS
  PGMPerspVS_levels
  PGMPerspVS_test
)

if(${USE_THETA})
	if(${USE_UR})
    SET (SRC_SOURCES
        src/CamUVCtheta.hpp
        src/thirdparty/libuvc-theta-sample/thetauvc.c
        src/thirdparty/insta360-crombez/videoDecoder.cpp

    		src/C_UR.h
		    src/C_UR.cpp)
  else()
    SET (SRC_SOURCES
        src/CamUVCtheta.hpp
        src/thirdparty/libuvc-theta-sample/thetauvc.c
        src/thirdparty/insta360-crombez/videoDecoder.cpp)
  endif()
endif()

if(${USE_FLIR})
	if(${USE_UR})
    SET (SRC_SOURCES
#       src/CamFlir.hpp
        src/CamFlirSpinnaker.hpp

    		src/C_UR.h
		    src/C_UR.cpp)
  else()
    SET (SRC_SOURCES
#       src/CamFlir.hpp
        src/CamFlirSpinnaker.hpp)
  endif()
endif()

foreach(cpp ${PGMPerspVS_cpp})
  # target 
	add_executable(${cpp} ${cpp}.cpp ${SRC_SOURCES})

  if(${USE_UR})
    target_link_libraries(${cpp} ur_rtde::rtde)
  endif()

  if(${USE_FLIR})
    #target_link_libraries(${cpp} -lflycapture)
    target_link_libraries(${cpp} -lSpinnaker)
  endif()

  if(${USE_THETA})
    target_link_libraries(${cpp} -luvc PkgConfig::LIBAV)
  endif()

endforeach()

SET(USE_THETA FALSE CACHE BOOL "Sets the use of Theta X camera" FORCE)
SET(USE_FLIR FALSE CACHE BOOL "Sets the use of FLIR camera" FORCE)
SET(USE_UR FALSE CACHE BOOL "Sets the use of UR robot" FORCE)
