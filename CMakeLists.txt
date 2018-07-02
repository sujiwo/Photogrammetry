cmake_minimum_required (VERSION 2.6)
project (Photogrammetry)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

find_package (Boost REQUIRED)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (PCL REQUIRED)

#find_package (Theia REQUIRED)
set (gflags_LIBRARIES gflags)


add_subdirectory (g2o)


include_directories (
	${EIGEN3_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	g2o
#	${THEIA_INCLUDE_DIRS}
)

add_executable (photogram
	main.cpp
	VMap.cpp
	MapBuilder.cpp
	KeyFrame.cpp
	MapPoint.cpp
	triangulation.cpp
	optimizer.cpp
)

target_link_libraries (
	photogram
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	g2o
#	${THEIA_LIBRARIES}
)

