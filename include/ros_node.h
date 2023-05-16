
using namespace std;

#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <fstream>
#include <deque>
#include <mutex>
#include <memory>
#include <glog/logging.h>
#include <std_msgs/Int32.h>
// #include <vector>


#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//自定义msgs
#include <IntensityPoint2D.h>
#include <IntensityRange2D.h>

//pcl
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include<pcl/octree/octree.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/kdtree/io.h>
#include <pcl/filters/filter.h>



//项目头文件
#include "DBscanCluster.h"
#include "DBscanClusterInterface.h"
#include "mapping/dustin_mapping.h"
#include "mapping/abstract_mapping.h"

namespace Reflector_localization{
class Node
{
    public:
        Node();
        ~Node();

        /*节点参数*/
        struct NodeOptions
        {
                //rosnode 
                std::string param_version;
                bool use_laser;
                bool use_point_cloud;
                bool use_imu;

                bool using_dustin_mapping;
                bool using_carto_mapping;
                bool using_fullGrapthing_mapping;

                std::string scan_topic_name;
                std::string points_topic_name;
                std::string odom_topic_name;
                std::string save_map_topic_name;

                //
                int intensity_median_filter_param;
                int intensity_down_threshold;

                //DBscanCluster
                double reflect_cylinder_diameter;
                double dbscan_eps_radius;
                int dbscan_min_points;
                double merge_distance;

                //Dusting Mapping 
                bool use_load_map;
                bool wait_initial_pose;
                int feature_point_cnt;
                double distance_ceil_threshold;
                double distance_match_tolerance;
                double relocate_match_tolerance;
                double dustin_merge_distance;
                std::string save_map_filename;
                std::string load_map_filename;

        };
        /*节点参数初始化*/
        NodeOptions options_;
        ros::NodeHandle node_handle_;   

        /*DustingMapping 指针*/
        std::unique_ptr<Reflector_localization::DustinMapping> DustinMapping_;

        std::unique_ptr<Reflector_localization::DBscanCluster> DBscanCluster_;

        reflector_localization::RobotPose odom_pose_;
        reflector_localization::RobotPose amcl_pose_;

        /*函数定义*/
        // void LoadNodeOptions(); //加载参数
        void ScanCallback(const sensor_msgs::LaserScanConstPtr &scan_ptr);
        void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
        void OdometryCallback(const nav_msgs::OdometryConstPtr &msg);
        void SaveMapCallback(const std_msgs::String::ConstPtr &msg);
        void AmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
        reflector_localization::IntensityRange2D laserScanToIntensityRange2D(const sensor_msgs::LaserScanConstPtr &scan_ptr);

        void clusterFilteredRange(const reflector_localization::IntensityRange2D &input_range, 
                                    reflector_localization::Feature2DList &output_feature_list);
        bool Extract(reflector_localization::IntensityRange2D &cloud, 
                     reflector_localization::IntensityRange2D &cloud_candidate);
        void PubGlobalMarkerArray();
        void PubRobotPose(reflector_localization::RobotPose robot_pose);
        // void PubLocalMarkerArray();


        /*发布、订阅话题*/
        ros::Publisher laserPoint_publisher_;
        ros::Publisher transfered_point_;
        ros::Publisher glocal_feature_points_publisher_;
        ros::Publisher polygon_pub_;
        // ros::Publisher landmark_publisher_;
        // ros::Publisher pose_publisher_;
        // ros::Publisher path_publisher_;
        // ros::Publisher global_reflector_publisher_;
        // ros::Publisher occupancy_grid_publisher_;
        // ros::Publisher matched_point_cloud_publisher_;

        // ros::Subscriber odometry_subscriber_;
        ros::Subscriber laser_subscriber_;
        ros::Subscriber odometry_subscriber_;
        ros::Subscriber save_map_subscriber_;
        ros::Subscriber amcl_pose_subscriber_;
        // ros::Subscriber point_cloud_subscriber_;

        tf::TransformListener tf_listener_;
        tf::TransformListener baselinkToMap_listener_;


        /***** 获取参数 *****/
        void LoadNodeOptions()
        {    
                // rosnode 参数
                bool version_flag = node_handle_.param("/reflector_localization_node/param_version", options_.param_version, std::string("未正确读取"));
                node_handle_.param("/reflector_localization_node/use_laser", options_.use_laser, true);
                node_handle_.param("/reflector_localization_node/use_point_cloud", options_.use_point_cloud, false);
                node_handle_.param("/reflector_localization_node/use_imu", options_.use_imu, false);
                        //默认使用using_dustin_mapping
                node_handle_.param("/reflector_localization_node/using_dustin_mapping", options_.using_dustin_mapping, true);
                node_handle_.param("/reflector_localization_node/using_carto_mapping", options_.using_carto_mapping, false);
                node_handle_.param("/reflector_localization_node/using_fullGrapthing_mapping", options_.using_fullGrapthing_mapping, false);
                        //话题名称
                node_handle_.param("/reflector_localization_node/scan_topic_name", options_.scan_topic_name, std::string("/scan"));
                node_handle_.param("/reflector_localization_node/points_topic_name", options_.points_topic_name, std::string(""));
                node_handle_.param("/reflector_localization_node/odom_topic_name", options_.odom_topic_name, std::string(""));
                node_handle_.param("/reflector_localization_node/save_map_topic_name", options_.save_map_topic_name, std::string("/save_reflector_map"));


                        //反光点提取参数
                node_handle_.param("/reflector_localization_node/intensity_median_filter_param", options_.intensity_median_filter_param, 5 );
                node_handle_.param("/reflector_localization_node/intensity_down_threshold", options_.intensity_down_threshold, 1000 );
                //DBscanCluster
                node_handle_.param("/reflector_localization_node/reflect_cylinder_diameter", options_.reflect_cylinder_diameter, 0.050);
                node_handle_.param("/reflector_localization_node/dbscan_eps_radius", options_.dbscan_eps_radius, 0.1);
                node_handle_.param("/reflector_localization_node/dbscan_min_points", options_.dbscan_min_points, 4);
                node_handle_.param("/reflector_localization_node/merge_distance", options_.merge_distance, 0.2);
                //Dustin_mapping 参数默认值
                node_handle_.param("/reflector_localization_node/use_load_map", options_.use_load_map, false);
                node_handle_.param("/reflector_localization_node/wait_initial_pose", options_.wait_initial_pose, false);
                node_handle_.param("/reflector_localization_node/feature_point_cnt", options_.feature_point_cnt, 0);
                node_handle_.param("/reflector_localization_node/distance_ceil_threshold", options_.distance_ceil_threshold, 20.0);
                node_handle_.param("/reflector_localization_node/distance_match_tolerance", options_.distance_match_tolerance, 0.05);
                node_handle_.param("/reflector_localization_node/relocate_match_tolerance", options_.relocate_match_tolerance, 0.01);
                node_handle_.param("/reflector_localization_node/dustin_merge_distance", options_.dustin_merge_distance, 0.5);
                node_handle_.param("/reflector_localization_node/save_map_filename", options_.save_map_filename, std::string("/home"));
                node_handle_.param("/reflector_localization_node/load_map_filename", options_.load_map_filename, std::string("/home"));

                //cluster 参数

                if(version_flag)
                {
                        // 在控制台中显示加载的参数
                        ROS_INFO("Parameters loaded successfully!");
                        ROS_INFO("param_version: %s", options_.param_version.c_str());
                }
        }


};

}

