#pragma once

#include "gtest/gtest_prod.h"
#include "mapping/abstract_mapping.h"
#include "feature_map.h" //非侵入式序列化定义AdjacencyList类

#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <fstream>

#include <cmath>
#include <sstream>
#include <string>

#include "carto_mapping/feature_pair_cost.h"

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
// #include <opencv2/opencv.hpp>

#include "Feature2DList.h"
#include "AdjacencyList.h"
#include "RobotPose.h"
// #include "MappingOptions.h"


using std::pair;
using std::make_pair;
using std::unordered_set;
using std::unordered_map;
using std::set;
using std::map;
using std::priority_queue;
using std::queue;
using std::vector;

using std::max;
using std::min;
using std::stringstream;

namespace Reflector_localization
{

class DustinMapping { 


 public:

  struct DustinOptions
  {
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

public:
    ros::NodeHandle nh_;  // ros node handle

    DustinMapping(const DustinOptions &options, ros::NodeHandle& nh);


  // DustinMapping();
  unordered_map<int, pair<double, double> > feature_points;
  unordered_map<int, unordered_map<int, double> > feature_graph; // id core - id pair descriptor
  
  DustinOptions DustinOptions_;
  reflector_localization::AdjacencyList current_graph;
  reflector_localization::AdjacencyList loaded_adAdjacency;
  friend class MappingFactory;
  FRIEND_TEST(DustinMappingTest, FakeMapBuild);

  ~DustinMapping(){};

  // bool Init();
//将新的反光柱的位置存入feature_points中，并更新反光柱地图的结构图（就是反光柱之间的边）feature_graph
  bool InsertFeatureList(reflector_localization::Feature2DList &feature_list,
                         reflector_localization::AdjacencyList &adjacency_list,
                         reflector_localization::RobotPose &robot_pose);

  bool SetInitialPose(reflector_localization::RobotPose &pose);  

  bool RelocalizationRobot(vector<pair<double, double> > &input_fpts,
                           vector<pair<int, int> > &hit_id_pair);

  void SiftNewFeaturePoints(vector<pair<double, double> > &input_fpts,
                            Eigen::Isometry2d &EstimatePose, vector<pair<int, int> > &hit_id_pair,
                            vector<pair<double, double> > &feature_points_wait_insert);

  void OptimizeCurrentPose(vector<pair<pair<double, double>, pair<double, double> > > &match_result,
                           reflector_localization::RobotPose &robot_pose);

  bool InitialCurrentPose(vector<pair<pair<double, double>, pair<double, double> > > &match_result,
                          reflector_localization::RobotPose &robot_pose);
  //验证计算出的结果是否有效
  bool CheckResultValid(vector<double> &radius_list,
                        vector<pair<pair<double, double>, pair<double, double> > > &match_result,
                        pair<double, double> &result, int id1, int id2, double d12);

  inline void DropAdjacencyList(reflector_localization::AdjacencyList &adjacency_list);
  //将待插入的特征点添加到特征图中
  void InsertToFeatureGraph(vector<pair<double, double> > &fpts_wait_insert);


  void SaveMap();
  bool LoadExistMapFile();

  bool feature_map_saver(reflector_localization::Feature2DList& map_data, std::string& map_filename);

  bool feature_map_loader(std::string& map_filename, reflector_localization::Feature2DList* map_data);

  //计算两个点之间的距离
  inline double ComputeDistance(pair<double, double> &pt1, pair<double, double> &pt2);

  inline double ComputeRadius(pair<double, double> &pt);


  // private:
  // friend class boost::serialization::access;
  // template<typename Archive>
  // void serialize(Archive& archive, const unsigned int version)
  // {
  //   archive & BOOST_SERIALIZATION_NVP(current_graph);
  // }



};

}  // Reflector_localization
