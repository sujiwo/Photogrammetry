cmake_minimum_required (VERSION 2.6)
project (Photogrammetry)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

find_package (Boost REQUIRED COMPONENTS serialization python)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (PCL REQUIRED)
find_package (OpenMP REQUIRED)
find_package (PythonLibs REQUIRED)

set (gflags_LIBRARIES gflags)


add_subdirectory (g2o)
add_subdirectory (DBoW2)


include_directories (

	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${PYTHON_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	
	g2o
	DBoW2
)

set(REQUIRED_LIBS
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	boost_serialization
	g2o
	DBoW2
)

add_library(
	photogrammetry STATIC
		VMap.cpp
		MapBuilder.cpp
		KeyFrame.cpp
		MapPoint.cpp
		Viewer.cpp
		ImageDatabase.cpp
		triangulation.cpp
		optimizer.cpp		
)

add_executable (map_test
	map_test.cpp
)

target_link_libraries (map_test
	photogrammetry
	${REQUIRED_LIBS}
)

add_executable(map_load
	map_load.cpp
)

target_link_libraries(map_load
	photogrammetry
	${REQUIRED_LIBS}
)

add_library(photogrampy SHARED python_mod.cpp)
set_target_properties(photogrampy PROPERTIES PREFIX "")
target_link_libraries(photogrampy
	photogrammetry
	${REQUIRED_LIBS}
	boost_python
)
