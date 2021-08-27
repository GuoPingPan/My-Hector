cmake_minimum_required(VERSION 3.0.2)
project(hector_slam)

set(RPLIDAR_SDK_PATH "./sdk/")

FILE(GLOB RPLIDAR_SDK_SRC 
  "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
  "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

find_package(OpenCV 3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${Boost_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIRS}
${RPLIDAR_SDK_PATH}/include
${RPLIDAR_SDK_PATH}/src
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
	src/hector_slam.cpp
)
target_link_libraries(${PROJECT_NAME}
	${Boost_LIBRARIES}
	${OpenCV_LIBS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(hector_mapping
	src/node.cpp
	${RPLIDAR_SDK_SRC})
target_link_libraries(hector_mapping
	${PROJECT_NAME})


set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

add_executable(main
	test/main.cpp
	${RPLIDAR_SDK_SRC})
target_link_libraries(main
	${PROJECT_NAME})


add_executable(testparam test/testparam.cpp)
target_link_libraries(testparam
	${OpenCV_LIBS})


