# AMZ-Driverless
# Copyright (c) 2018 Authors:
#   - Juraj Kabzan <kabzanj@gmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${SDFormat_INCLUDE_DIRS}
                    ${OGRE_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

link_directories(${catkin_LIBRARY_DIRS}
                 ${OGRE_LIBRARY_DIRS}
                 ${OGRE-Terrain_LIBRARY_DIRS}
                 ${OGRE-Paging_LIBRARY_DIRS})

add_library(gazebo_track SHARED
            src/gazebo_track.cpp)
target_include_directories(gazebo_track PUBLIC
                           include
                           ${CMAKE_SOURCE_DIR}
                           ${EIGEN3_INCLUDE_DIR}
                           ${PCL_INCLUDE_DIRS})
target_link_libraries(gazebo_track
                      ${GAZEBO_LIBRARIES}
                      ${IGNITION-MSGS_LIBRARIES}
                      yaml-cpp
                      ${EIGEN3_INCLUDE_DIR}
                      ${PCL_INCLUDE_DIRS})

install(TARGETS gazebo_track
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
