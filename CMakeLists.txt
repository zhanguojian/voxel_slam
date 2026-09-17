cmake_minimum_required(VERSION 3.16)

project(voxel_slam VERSION 0.2.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters io)
find_package(yaml-cpp REQUIRED)
find_package(glog REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(PCAP REQUIRED IMPORTED_TARGET libpcap)

add_library(voxel_slam_core
  src/config.cpp src/io.cpp src/imu_process.cpp src/voxel_map.cpp src/mapper.cpp)

target_include_directories(voxel_slam_core PUBLIC include ${PCL_INCLUDE_DIRS} PRIVATE src)

target_link_directories(voxel_slam_core PUBLIC ${PCL_LIBRARY_DIRS})

target_compile_options(voxel_slam_core PRIVATE -Wall -Wextra ${PCL_DEFINITIONS})

if(TARGET yaml-cpp::yaml-cpp)
  set(VOXEL_YAML_TARGET yaml-cpp::yaml-cpp)
else()
  set(VOXEL_YAML_TARGET yaml-cpp)
endif()

target_link_libraries(voxel_slam_core PUBLIC Eigen3::Eigen ${PCL_LIBRARIES}
  ${VOXEL_YAML_TARGET} glog::glog OpenMP::OpenMP_CXX PkgConfig::PCAP)

find_package(Pangolin REQUIRED)
target_sources(voxel_slam_core PRIVATE src/ui/pangolin_window.cc)
target_include_directories(voxel_slam_core PRIVATE ${Pangolin_INCLUDE_DIRS})
target_link_libraries(voxel_slam_core PUBLIC ${Pangolin_LIBRARIES})
target_compile_definitions(voxel_slam_core PUBLIC VOXEL_SLAM_WITH_PANGOLIN=1)


add_executable(run_lio_mapping src/run_lio.cpp)
target_link_libraries(run_lio_mapping PRIVATE voxel_slam_core)

install(TARGETS run_lio_mapping RUNTIME DESTINATION bin)
install(DIRECTORY config/ DESTINATION share/voxel_slam/config)