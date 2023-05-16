#include "ros_node.h"

Reflector_localization::Node::Node()
{
    LoadNodeOptions(); 


    Reflector_localization::DBscanClusterInterface::DBscanClusterOptions dboptions_;
            //DBscanCluster
    dboptions_.reflect_cylinder_diameter = options_.reflect_cylinder_diameter;
    dboptions_.dbscan_eps_radius = options_.dbscan_eps_radius;
    dboptions_.dbscan_min_points = options_.dbscan_min_points;
    dboptions_.merge_distance = options_.merge_distance;
    DBscanCluster_ = std::make_unique<Reflector_localization::DBscanCluster>(dboptions_);

    if(options_.using_dustin_mapping)
    {
        Reflector_localization::DustinMapping::DustinOptions  dustin_options_;

        dustin_options_.use_load_map = options_.use_load_map;
        dustin_options_.wait_initial_pose = options_.wait_initial_pose;
        dustin_options_.feature_point_cnt = options_.feature_point_cnt;
        dustin_options_.distance_ceil_threshold = options_.distance_ceil_threshold;
        dustin_options_.distance_match_tolerance = options_.distance_match_tolerance;
        dustin_options_.relocate_match_tolerance = options_.relocate_match_tolerance;
        dustin_options_.dustin_merge_distance = options_.dustin_merge_distance;
        dustin_options_.save_map_filename = options_.save_map_filename;
        dustin_options_.load_map_filename = options_.load_map_filename;
        //initialise DUSTIN 方法
        DustinMapping_ = std::make_unique<Reflector_localization::DustinMapping>(dustin_options_, node_handle_);
    }





    /**********话题订阅********/

    if (options_.use_laser)
    {
        laser_subscriber_ = node_handle_.subscribe(options_.scan_topic_name, 1, &Node::ScanCallback, this);
        save_map_subscriber_ = node_handle_.subscribe(options_.save_map_topic_name, 1, &Node::SaveMapCallback, this);
        odometry_subscriber_ = node_handle_.subscribe(options_.odom_topic_name, 1, &Node::OdometryCallback, this);
        amcl_pose_subscriber_ = node_handle_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 1, &Node::AmclPoseCallback, this);
    }
    // if (options_.use_point_cloud)
    // {
    //     point_cloud_subscriber_ =
    //         node_handle_.subscribe(options_.points_topic_name, 1, &Node::PointCloudCallback, this);
    // }
    /**********话题发布********/
    laserPoint_publisher_ = node_handle_.advertise<reflector_localization::IntensityRange2D>("laser_points", 10);
    transfered_point_= node_handle_.advertise<sensor_msgs::PointCloud2>("transfered_point_", 10);
    glocal_feature_points_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("feature_points", 1); 
    polygon_pub_ = node_handle_.advertise<geometry_msgs::PolygonStamped>("reflector_robot_pose_", 1);
}

Reflector_localization::Node::~Node()
{
}


/*保存地图函数*/
void Reflector_localization::Node::SaveMapCallback(const std_msgs::String::ConstPtr& msg)
{
    if (DustinMapping_ != nullptr) 
    {
        if (msg->data == "1") 
        {
            // reflector_localization::Feature2DWithID nwfpt;
            // reflector_localization::AdjacencyList full_adjacency_list;
            // unordered_map<int, pair<double, double> > full_feature_points; //扫图完成后，所有的反光柱信息

            DustinMapping_-> SaveMap();
        }
        // else if(msg->data == "2")
        // {
        //     DustinMapping_->LoadExistMapFile();
        // }
    }
    else 
    {
        std::cerr << "Error: dustin_mapping_没有初始化，报错!" << std::endl;
    }
}


/***** 激光回调函数 *****/
void Reflector_localization::Node::ScanCallback(const sensor_msgs::LaserScanConstPtr &scan_ptr)
{
    //发布global反光柱地图
    // if(options_.use_load_map)
    // {
        PubGlobalMarkerArray();
    // }
    reflector_localization::IntensityRange2D range; //激光x & y & intensity的格式
    reflector_localization::IntensityRange2D filtered_range; //激光x & y & intensity的格式


    bool filtered_flag = false; //极光数据提取flag
    bool cluster_flag = false; //聚类flag

    // 转换激光雷达数据到IntensityRange2D格式
    range = Node::laserScanToIntensityRange2D(scan_ptr);

    //提取反光点
    filtered_flag = Node::Extract(range, filtered_range); 



    reflector_localization::Feature2DList clustered_feature_list;
    reflector_localization::AdjacencyList adjacency_list;
    unordered_map<int, pair<double, double> > feature_points;
    reflector_localization::Feature2DWithID nwfpt;
    reflector_localization::RobotPose robot_pose;
    robot_pose.x = 0;
    robot_pose.x = 0;
    robot_pose.theta = 0;
    if (filtered_flag)
    {
        // Node::clusterFilteredRange(filtered_range, clustered_feature_list);
        cluster_flag = DBscanCluster_->Association(filtered_range, clustered_feature_list);


    }
    else
    {
        // std::cout<< "filtered_flag ============= "<< filtered_flag << std::endl;
    }



    if(clustered_feature_list.feature_2d_with_ids.size() > 0)
    {
        for(int i = 0; i< clustered_feature_list.feature_2d_with_ids.size(); i++)
        {

        // D_INFO << "找到第 [ " << BASHR << i+1 << BASHW << "] 反光点。 ID[ " << BASHR << i << BASHW << "], x =  "  
        // << clustered_feature_list.feature_2d_with_ids[i].x << "y = " << clustered_feature_list.feature_2d_with_ids[i].y 
        // << BASHW;

        }

        // D_INFO << "----------------------------------------------------" << BASHW;


            if(DustinMapping_->InsertFeatureList(clustered_feature_list, adjacency_list, robot_pose))
            {
                D_INFO << BASHR << "robot_POS.x = "  << BASHW << robot_pose.x  << BASHR << "robot_POS.y = " << BASHW 
                << robot_pose.y << BASHR << "robot_POS.theta = "  << BASHW << robot_pose.theta ;
                if(robot_pose.x != 0 && robot_pose.y!= 0)
                {
                    Node::PubRobotPose(robot_pose);
                }
            }



        std::cout<< "DustinMapping_-> featurepoints = "<< DustinMapping_->feature_points.size()<<std::endl;

    }




    

    ///////////////////////////////////////////////////////////////////////////////////
    // laserPoint_publisher_.publish(range);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    for(int i = 0; i<clustered_feature_list.feature_2d_with_ids.size(); i++)
    {
        pcl::PointXYZ p;
        p.x = clustered_feature_list.feature_2d_with_ids[i].x;
        p.y = clustered_feature_list.feature_2d_with_ids[i].y;

        cloud->points.push_back(p);
    }


    //滤除重合点之后的点云
    sensor_msgs::PointCloud2 outcloud;
    pcl::toROSMsg(*cloud, outcloud);
    outcloud.header.frame_id = "/map";  
    outcloud.header.stamp = scan_ptr->header.stamp;
    transfered_point_.publish(outcloud);  //publish extract/points

    //////////////////////////////////////////////////////////////////////////////

}





/***** 相机点云回调函数 *****/
void Reflector_localization::Node::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &points_ptr)
{

}

// void Reflector_localization::Node::clusterFilteredRange(const reflector_localization::IntensityRange2D &input_range, 
// reflector_localization::Feature2DList &output_feature_list)
// {
//     double cluster_radius = 0.5;
//     int32_t point_id = 0;

//     for (size_t i = 0; i < input_range.data.size(); i++)
//     {
//         const auto &point = input_range.data[i];
//         bool is_duplicate = false;

//         for (size_t j = 0; j < output_feature_list.feature_2d_with_ids.size(); j++)
//         {
//             const auto &clustered_point = output_feature_list.feature_2d_with_ids[j];
//             double distance = std::sqrt(std::pow(point.x - clustered_point.x, 2) + std::pow(point.y - clustered_point.y, 2));
//             if (distance < cluster_radius)
//             {
//                 is_duplicate = true;
//                 break;
//             }
//         }

//         if (!is_duplicate)
//         {
//             reflector_localization::Feature2DWithID feature;
//             feature.x = point.x;
//             feature.y = point.y;
//             feature.ID = point_id;
//             output_feature_list.feature_2d_with_ids.push_back(feature);
//             point_id++;
//         }
//     }
// }


//////////////////////////////////////////////////////////////////////////////////////
//点云数据处理
reflector_localization::IntensityRange2D Reflector_localization::Node::laserScanToIntensityRange2D(const sensor_msgs::LaserScanConstPtr &scan_ptr)
{

    reflector_localization::IntensityRange2D points_range;
    points_range.frame_id = scan_ptr->header.frame_id;
    points_range.timestamp = scan_ptr->header.stamp.toNSec();


    float angle_min = scan_ptr->angle_min;
    float angle_max = scan_ptr->angle_max;
    float angle_resolution = scan_ptr->angle_increment;

    for(int i = 0; i < scan_ptr->ranges.size(); ++i)
    {
        float theta = angle_min + i * angle_resolution;
        if(theta > -2.0 * M_PI && theta < 2.0 * M_PI)
        {
            geometry_msgs::PointStamped p;
            geometry_msgs::PointStamped p_transformed;
            reflector_localization::IntensityPoint2D pointIntensity;
            p.header.stamp = scan_ptr->header.stamp;
            p.header.frame_id = scan_ptr->header.frame_id;
            p.point.x = scan_ptr->ranges[i] * cos(theta);
            p.point.y = scan_ptr->ranges[i] * sin(theta);

            try
            {
                tf_listener_.waitForTransform("/map", scan_ptr->header.frame_id, ros::Time(0), ros::Duration(1.0));
                tf_listener_.transformPoint("/map", ros::Time(0), p, scan_ptr->header.frame_id, p_transformed);
            }
            catch(tf::TransformException& ex)
            {
                ROS_ERROR("%s",ex.what());
                exit(0);
            }

            pointIntensity.x = p_transformed.point.x;
            pointIntensity.y = p_transformed.point.y;
            pointIntensity.intensity =  scan_ptr->intensities[i];
            

            points_range.data.push_back(pointIntensity); 
        }
    }

    return points_range;

}

bool Reflector_localization::Node::Extract(reflector_localization::IntensityRange2D &cloud, 
                reflector_localization::IntensityRange2D &cloud_candidate)
{
    cloud_candidate.data.clear();

    int intensity_threshold = options_.intensity_down_threshold;
    int intensity_median_filter_param = options_.intensity_median_filter_param;
    int half_window = intensity_median_filter_param / 2;

    vector<double> mFilterBody;
    int cloud_sz = cloud.data.size();
    for (int id = 0; id < cloud_sz; id ++) 
    {
        auto iter = cloud.data.begin() + id;

        mFilterBody.clear();
        for (int bias = id - half_window; bias <= id + half_window; bias++) 
        {
            if (bias < 0) 
            {
                mFilterBody.push_back( (cloud.data[bias + cloud_sz].intensity));
            } 
            else if (bias >= cloud_sz) 
            {
                mFilterBody.push_back((cloud.data[bias - cloud_sz].intensity));
            } 
            else 
            {
                mFilterBody.push_back((cloud.data[bias].intensity));
            }
        }
        std::sort(mFilterBody.begin(), mFilterBody.end());


        if (mFilterBody[half_window] >= intensity_threshold) 
        {
            reflector_localization::IntensityPoint2D temp_point;
            temp_point.x = iter->x;
            temp_point.y = iter->y;
            temp_point.intensity = (iter->intensity) ;
            cloud_candidate.data.push_back(temp_point);
        }
    }

    if(cloud_candidate.data.size() < 0)
    {
        return false;
    }
    else
    {
        return true;
    }

}

/***** amcl回调函数 *****/
void Reflector_localization::Node::AmclPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    // 读取机器人的位姿信息
    amcl_pose_.x = msg->pose.pose.position.x;
    amcl_pose_.y = msg->pose.pose.position.y;
    // double z = msg->pose.pose.position.z;
    double roll, pitch, yaw;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    amcl_pose_.theta = yaw;
    // D_INFO << "amcl_pose_.x == " << BASHY << amcl_pose_.x << BASHW << "amcl_pose_.y == "<< BASHY << amcl_pose_.y 
    // << BASHW << "amcl_pose_.theta == " << BASHY << amcl_pose_.theta << BASHW;
    // D_INFO << "----------------------------------------------------" << BASHW;
}
/***** 里程计回调函数 *****/
void Reflector_localization::Node::OdometryCallback(const nav_msgs::OdometryConstPtr &msg)
{

    // 读取机器人的位姿信息
    odom_pose_.x = msg->pose.pose.position.x;
    odom_pose_.y = msg->pose.pose.position.y;
    // double z = msg->pose.pose.position.z;
    double roll, pitch, yaw;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    odom_pose_.theta = yaw;

    // D_INFO << "odom_pose_.x == " << BASHG << odom_pose_.x << BASHW << "odom_pose_.y == "<< BASHG << odom_pose_.y 
    // << BASHW << "odom_pose_.theta == " << BASHG << odom_pose_.theta << BASHW;
    // D_INFO << "----------------------------------------------------" << BASHW;
}


//发布global反光柱
void Reflector_localization::Node::PubGlobalMarkerArray()
{
    // Get the feature points from DustinMapping_
    auto feature_points = DustinMapping_-> feature_points;

    // Prepare a MarkerArray for publication
    visualization_msgs::MarkerArray cylinder_Array;
    visualization_msgs::Marker cylinder_arrow;

    // Iterate over feature points
    for(auto ite=feature_points.begin();ite!=feature_points.end();ite++)
    {
        cylinder_arrow.ns = "cylinder";    //命名空间namespace                                                                          
        cylinder_arrow.type = visualization_msgs::Marker::CYLINDER;     //类型
        cylinder_arrow.action = visualization_msgs::Marker::ADD;
        cylinder_arrow.lifetime = ros::Duration(); //(sec,nsec),0 forever
        cylinder_arrow.header.frame_id = "/map";
        cylinder_arrow.header.stamp = ros::Time::now();
        cylinder_arrow.id = ite->first;
        cylinder_arrow.pose.position.x = ite->second.first;
        cylinder_arrow.pose.position.y = ite->second.second;
        cylinder_arrow.pose.position.z = 0;
        cylinder_arrow.pose.orientation.w = 1.0;
        cylinder_arrow.pose.orientation.x = 0;
        cylinder_arrow.pose.orientation.y = 0;
        cylinder_arrow.pose.orientation.z = 0;

        //设置标记尺寸
        cylinder_arrow.scale.x = 0.09; //m
        cylinder_arrow.scale.y = 0.09;
        cylinder_arrow.scale.z = 0.50;

        ///设置标记颜色
        cylinder_arrow.color.a = 1.0; // Don't forget to set the alpha!
        cylinder_arrow.color.r = 1.0;
        cylinder_arrow.color.g = 0.0;
        cylinder_arrow.color.b = 0.0;
        
        // Add the cylinder marker to the MarkerArray
        cylinder_Array.markers.push_back(cylinder_arrow);
    }

    // Publish the MarkerArray
    glocal_feature_points_publisher_.publish(cylinder_Array);

    // Clear the MarkerArray for the next loop
    cylinder_Array.markers.clear();
}


void Reflector_localization::Node::PubRobotPose(reflector_localization::RobotPose robot_pose)
{
    geometry_msgs::PolygonStamped polygon_msg;


    polygon_msg.header.frame_id = "map";
    polygon_msg.header.stamp = ros::Time::now();

    // 创建机器人的四个角点
    std::vector<geometry_msgs::Point32> corners(4);
    corners[0].x = robot_pose.x - 0.5;
    corners[0].y = robot_pose.y - 0.5;

    corners[1].x = robot_pose.x + 0.5;
    corners[1].y = robot_pose.y - 0.5;

    corners[2].x = robot_pose.x + 0.5;
    corners[2].y = robot_pose.y + 0.5;

    corners[3].x = robot_pose.x - 0.5;
    corners[3].y = robot_pose.y + 0.5;

    // 从/base_link到/map的转换
    tf::StampedTransform transform;
    try
    {
        baselinkToMap_listener_.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
        baselinkToMap_listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    // 将机器人的四个角点坐标从/base_link转换到/map
    for (auto &corner : corners)
    {
        tf::Vector3 point_in_base_link(corner.x, corner.y, 0);
        tf::Vector3 point_in_map = transform * point_in_base_link;

        corner.x = point_in_map.x();
        corner.y = point_in_map.y();
    }

    // 将转换后的角点添加到polygon中
    polygon_msg.polygon.points = corners;

    // 发布polygon
    polygon_pub_.publish(polygon_msg);
}
