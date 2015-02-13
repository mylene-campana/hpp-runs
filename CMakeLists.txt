# Copyright (c) 2014-2015 CNRS
# Author: Mylene Campana
#
# This file is part of hpp-runs.
# hpp-hrp2 is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-runs is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-hrp2.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp-runs)
SET(PROJECT_DESCRIPTION "Launch several sessions of hpp-corbaserver and Python scripts")
SET(PROJECT_URL "")

SETUP_PROJECT()

FINDPYTHON()
ADD_REQUIRED_DEPENDENCY ("hpp-corbaserver >= 3")

#SET(CATKIN_PACKAGE_SHARE_DESTINATION
#  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

#install(FILES urdf/object_point.urdf
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
#  )
  
SETUP_PROJECT_FINALIZE()
