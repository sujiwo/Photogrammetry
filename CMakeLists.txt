cmake_minimum_required (VERSION 2.6)
project (Photogrammetry)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

find_package (Boost REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (PCL REQUIRED)
find_package(OpenMP REQUIRED)

#find_package (Theia REQUIRED)
set (gflags_LIBRARIES gflags)


add_subdirectory (g2o)
add_subdirectory (DBoW2)


include_directories (
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	g2o
#	${THEIA_INCLUDE_DIRS}
)

set(SOURCE_FILES
	VMap.cpp
	MapBuilder.cpp
	KeyFrame.cpp
	MapPoint.cpp
	Viewer.cpp
	ImageDatabase.cpp
	triangulation.cpp
	optimizer.cpp
)

set(REQUIRED_LIBS
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	g2o
	DBoW2
#	${THEIA_LIBRARIES}
)

add_executable (map_test
	map_test.cpp
	${SOURCE_FILES}
)

target_link_libraries (map_test
	${REQUIRED_LIBS}
)

add_executable(map_load
	map_load.cpp
	${SOURCE_FILES}
)

target_link_libraries(map_load
	${REQUIRED_LIBS}
)
