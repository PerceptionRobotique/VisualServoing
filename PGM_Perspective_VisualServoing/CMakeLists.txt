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

find_package(PER REQUIRED per_core per_core_extended per_io per_features per_sensor_pose_estimation per_estimation)
if(PER_FOUND)
	include(${PER_USE_FILE})
endif(PER_FOUND)

# ViSP (to do: list modules only)
find_package(VISP REQUIRED)
if(VISP_FOUND)
	include(${VISP_USE_FILE})
endif(VISP_FOUND)

#IF USE UR10
find_package(ur_rtde REQUIRED)

#IF USE FLIR
INCLUDE_DIRECTORIES(/usr/include/flycapture)

set(PGMPerspVS_cpp
  PGMPerspVS.cpp
  PGMPerspVS_levels.cpp
  PGMPerspVS_test.cpp
)

SET (SRC_SOURCES
#IF USE UR10
		src/C_UR.h
		src/C_UR.cpp
#IF USE FLIR
    src/CamFlir.hpp
)

foreach(cpp ${PGMPerspVS_cpp})
  per_add_target(${cpp})
  if(COMMAND pr_add_dependency)
    pr_add_dependency(${cpp} "PGMPerspVS")
  endif()
endforeach()

#IF USE UR10 AND/OR FLIR
target_sources(PGMPerspVS PUBLIC ${SRC_SOURCES})
#IF USE UR10
target_link_libraries(PGMPerspVS ur_rtde::rtde)
#IF USE FLIR
target_link_libraries(PGMPerspVS -lflycapture)

#IF USE UR10 AND/OR FLIR
target_sources(PGMPerspVS_levels PUBLIC ${SRC_SOURCES})
#IF USE UR10
target_link_libraries(PGMPerspVS_levels ur_rtde::rtde)
#IF USE FLIR
target_link_libraries(PGMPerspVS_levels -lflycapture)

