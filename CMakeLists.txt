cmake_minimum_required (VERSION 2.6)
project (Photogrammetry)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

find_package (Boost REQUIRED COMPONENTS serialization python graph)
find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (PCL REQUIRED)
find_package (OpenMP REQUIRED)
find_package (PythonLibs REQUIRED)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

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
	
	include
	${CMAKE_CURRENT_SOURCE_DIR}
)

set(REQUIRED_LIBS
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	Boost::serialization
	g2o
	DBoW2
)

add_library(
	photogrammetry STATIC
		src/VMap.cpp
		src/MapBuilder.cpp
		src/MapBuilder2.cpp
		src/KeyFrame.cpp
		src/MapPoint.cpp
		src/Viewer.cpp
		src/ImageDatabase.cpp
		src/Frame.cpp
		src/Localizer.cpp
		src/triangulation.cpp
		src/optimizer.cpp
		src/OxfordDataset.cpp		
)

add_executable (map_test
	src/map_test.cpp
)

target_link_libraries (map_test
	photogrammetry
	${REQUIRED_LIBS}
)

add_executable(map_load
	src/map_load.cpp
)

target_link_libraries(map_load
	photogrammetry
	${REQUIRED_LIBS}
)


if(${OpenCV_VERSION_MAJOR} EQUAL 3)
	set(PyMatSrc pymat3.cpp)
else()
	set(PyMatSrc pymat2.cpp)
endif()

add_library(photogrampy SHARED 
	python/photogrampy.cpp
	python/${PyMatSrc})
set_target_properties(photogrampy PROPERTIES PREFIX "")
target_link_libraries(photogrampy
	photogrammetry
	${REQUIRED_LIBS}
)


# These are experimental targets
add_executable(map_oxford
	src/map_oxford.cpp
)

target_link_libraries(map_oxford
	photogrammetry
	${REQUIRED_LIBS}
)

#add_executable(table_creator
#	src/table_creator.cpp
#)
