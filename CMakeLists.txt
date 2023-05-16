cmake_minimum_required(VERSION 3.0.2)
project(reflector_localization)


## 添加 glog 库
find_package(glog REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)


find_package(catkin REQUIRED COMPONENTS
 roscpp
 rospy
 std_msgs
 message_generation
 pcl_ros
)
find_package(Boost REQUIRED system serialization filesystem thread)

add_compile_options(-std=c++14)


add_message_files(
 DIRECTORY
 FILES 
 IntensityPoint2D.msg #  单个点
 IntensityRange2D.msg #  集合
 PointDBSCAN.msg #    聚类后的点
 Feature2DWithID.msg
 Feature2DList.msg
 Feature2DAdjacency.msg
 AdjacencyList.msg
 RobotPose.msg
 MappingOptions.msg

)


generate_messages( 
 DEPENDENCIES
 std_msgs
 )


include_directories(
    include 
    include/reflector_localization
    include/pointCluster
    include/mapSaver
    include/mapping
    include/carto_mapping
    include/common
    ${catkin_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${GFLAGS_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reflector_localization
CATKIN_DEPENDS 
roscpp 
sensor_msgs 
tf 
std_msgs 
rospy 
message_generation 
geometry_msgs 
nav_msgs 
message_runtime
pcl_ros
DEPENDS PCL
#  DEPENDS system_lib
${Boost_INCLUDE_DIRS}
Boost
)




add_executable(reflector_localization 
src/node_main.cpp 
src/rosnode.cpp 
src/pointCluster/DBscanCluster.cpp
src/mapping/abstract_mapping.cpp
src/mapping/dustin_mapping.cpp
src/mapSaver/feature_map.cpp
src/pointCluster/DBscanCluster.cpp)


target_link_libraries(reflector_localization
 ${catkin_LIBRARIES}
 ${PCL_LIBRARIES}
 ${CERES_LIBRARIES}
 ${GFLAGS_LIBRARIES}
 ${Boost_LIBRARIES}
 glog::glog
 Boost::serialization)



install(TARGETS reflector_localization 
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
