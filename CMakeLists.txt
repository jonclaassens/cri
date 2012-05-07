cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/)

# Why isn't this working?
find_package(OGRE REQUIRED)
find_package(OIS REQUIRED)
find_package(OgreProcedural REQUIRED)
find_package(PythonLibs)

rosbuild_add_boost_directories()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Generate message headers
rosbuild_genmsg()

include_directories(${CMAKE_SOURCE_DIR}/src/cri
		    ${CMAKE_SOURCE_DIR}/src/cri/gadgets
		    ${CMAKE_SOURCE_DIR}/src/converters/RobotModel
            ${OIS_INCLUDE_DIR}  	 
			${OGRE_INCLUDE_DIR}
			${PYTHON_INCLUDE_DIRS}
			${OgreProcedural_INCLUDE_DIR})

rosbuild_add_executable(cri 
			src/cri/gadgets/GActiveMarker.cpp 
			src/cri/gadgets/GActiveMarkerScripts.cpp 
			src/cri/gadgets/GFrame.cpp 
			src/cri/gadgets/GGrid.cpp 
			src/cri/gadgets/GGridCells.cpp 
			src/cri/gadgets/GPath.cpp
			src/cri/gadgets/GPointCloud.cpp
			src/cri/gadgets/GReticle.cpp 
			src/cri/gadgets/GReticleAxes.cpp 
			src/cri/gadgets/GTextureQuad.cpp
			src/cri/gadgets/GQuiver.cpp
			src/cri/PyBlock.cpp 
			src/cri/CRIMain.cpp 
			src/cri/IntersectionMath.cpp
			src/cri/OgreTools2.cpp	
			src/cri/ROSTools.cpp	
			src/cri/MovableText.cpp
			src/cri/CRI.cpp)

rosbuild_add_executable(robot_model_converter
			src/converters/RobotModel/RobotModel.cpp
			src/converters/RobotModel/RobotModelMain.cpp)

rosbuild_add_executable(battle_pr2
			src/tutorials/battle_pr2/main.cpp)

rosbuild_add_executable(technology_test
			src/cri/ROSTools.cpp
			src/converters/RobotModel/RobotModel.cpp
			src/tutorials/technology_test/technology_test.cpp)

target_link_libraries(cri ${OIS_LIBRARIES})
target_link_libraries(cri ${OGRE_LIBRARIES})
target_link_libraries(cri ${OgreProcedural_LIBRARIES})

target_link_libraries(cri assimp)

target_link_libraries(cri ${PYTHON_LIBRARIES})

rosbuild_link_boost(cri python thread signals)

