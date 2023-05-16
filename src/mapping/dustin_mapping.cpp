#include "mapping/dustin_mapping.h"
#include "common/dustin_log.h"

namespace Reflector_localization{


// DustinMapping::DustinMapping(const DBscanClusterOptions &options): options_(options){

// }


DustinMapping::DustinMapping(const DustinOptions &options, ros::NodeHandle& nh): DustinOptions_(options), nh_(nh) 
{
  DustinOptions_.use_load_map = options.use_load_map;
  DustinOptions_.wait_initial_pose = options.wait_initial_pose;
  DustinOptions_.feature_point_cnt = options.feature_point_cnt;
  DustinOptions_.distance_ceil_threshold = options.distance_ceil_threshold;
  DustinOptions_.distance_match_tolerance = options.distance_match_tolerance;
  DustinOptions_.relocate_match_tolerance = options.relocate_match_tolerance;
  DustinOptions_.dustin_merge_distance = options.dustin_merge_distance;
  DustinOptions_.feature_point_cnt = options.feature_point_cnt;
  DustinOptions_.save_map_filename = options.save_map_filename;
  DustinOptions_.load_map_filename = options.load_map_filename;

  // PARAM_LOG(distance_match_tolerance) << DustinOptions_.distance_match_tolerance;
  // PARAM_LOG(merge_distance) << DustinOptions_.merge_distance;

  // use_load_map = DustinOptions_.load_map;
  //加载反光柱地图

  D_INFO<< BASHY << "DustinOptions_.use_load_map = options.use_load_map = " << BASHR << DustinOptions_.use_load_map;
  if (DustinOptions_.use_load_map) 
  {
    LoadExistMapFile();
    DustinOptions_.wait_initial_pose = true;
    D_WARN << BASHB << "Map load fpts: " << BASHW
           << current_graph.feature_list.feature_2d_with_ids.size();
    // TODO: initial map relative data;


    std::vector<reflector_localization::Feature2DWithID> load_feature_list = current_graph.feature_list.feature_2d_with_ids;
    std::vector<std::pair<double, double>> load_fpt_wait_insert;
    for (const auto &fpt_id : load_feature_list) 
    {
      load_fpt_wait_insert.push_back(std::make_pair(fpt_id.x, fpt_id.y));
    }
    InsertToFeatureGraph(load_fpt_wait_insert);





    current_graph.feature_list.feature_2d_with_ids.clear();
  } 
  else 
  {
    DustinOptions_.wait_initial_pose = false;
  }

} //DustinMapping::DustinMapping

void DustinMapping::SaveMap() 
{

    reflector_localization::AdjacencyList full_adjacency_list;
    DropAdjacencyList(full_adjacency_list);
    reflector_localization::Feature2DList feature_list = full_adjacency_list.feature_list;
    std::string map_filename = DustinOptions_.save_map_filename;
    feature_map_saver(feature_list, map_filename);
}


bool DustinMapping::LoadExistMapFile() 
{
    // static reflector_localization::Feature2DList feature_list;
    std::string map_filename = DustinOptions_.load_map_filename;

    D_WARN << BASHB << "map_filename: " << BASHW<< map_filename;


    feature_map_loader(map_filename, &current_graph.feature_list);
    D_WARN << BASHB << "current_graph.feature_list.feature_2d_with_ids.size(): " << BASHW<< current_graph.feature_list.feature_2d_with_ids.size();

    return true;
}




bool DustinMapping::SetInitialPose(reflector_localization::RobotPose &pose) { return false; }

bool DustinMapping::RelocalizationRobot(vector<pair<double, double> > &input_fpts,
                                        vector<pair<int, int> > &hit_id_pair) 
{
  
  // step1. generatre local graph
  auto fpt_iter = input_fpts.begin();
  unordered_map<int, pair<double, double> > local_feature_points;
  unordered_map<int, unordered_map<int, double> > local_feature_graph;

  for (int id = 0; fpt_iter < input_fpts.end(); fpt_iter++, id++) 
  {
    local_feature_points[id] = *fpt_iter;
  }
  int fpt_sz = local_feature_points.size();
  for (int i = 0; i < fpt_sz; i++) 
  {
    for (int j = i + 1; j < fpt_sz; j++) 
    {
      double distance =
          sqrt(pow(local_feature_points[i].first - local_feature_points[j].first, 2) +
               pow(local_feature_points[i].second - local_feature_points[j].second, 2));
      if (distance <= DustinOptions_.distance_ceil_threshold) 
      {
        local_feature_graph[i].insert(make_pair(j, distance));
        local_feature_graph[j].insert(make_pair(i, distance));
      }
    }
  }

  // step2. relocation pair estimate.
  unordered_map<int, int> match;
  unordered_set<int> visited;
  for (auto local_fpt : local_feature_points) 
  {
    int local_id = local_fpt.first;
    if (visited.find(local_id) != visited.end()) 
    {
      continue;
    }
    visited.insert(local_id);

    for (auto global_fpt : feature_points) 
    {
      vector<pair<int, int> > tmp_match_result;
      int global_id = global_fpt.first;
      // //D_INFO << "try match pair: [" << local_id << "] - [" << global_id << "]";
      for (auto edge : feature_graph[global_id]) 
      {
        double length = edge.second;
        for (auto local_edge : local_feature_graph[local_id]) 
        {
          if (fabs(local_edge.second - length) <= DustinOptions_.relocate_match_tolerance) 
          {
            //   //D_INFO << "hit global length: " << length;
            tmp_match_result.push_back(
                make_pair(local_edge.first, edge.first));  // local_id - global_id
          }
        }
      }
      // two edges support one vertex.
      if (tmp_match_result.size() >= 2) 
      {
        int match_size = tmp_match_result.size();
        for (int i = 0; i < match_size; i++) 
        {
          for (int j = i + 1; j < match_size; j++) 
          {
            double local_length =
                local_feature_graph[tmp_match_result[i].first][tmp_match_result[j].first];
            double global_length =
                feature_graph[tmp_match_result[i].second][tmp_match_result[j].second];
            if (fabs(local_length - global_length) > DustinOptions_.relocate_match_tolerance) 
            {
              continue;
            }
          }
        }

        match[local_id] = global_id;
        for (auto match_pr : tmp_match_result) 
        {
          match[match_pr.first] = match_pr.second;
          visited.insert(match_pr.first);
        }
      }
    }
  }

  // TODO: use BFS to extand match...
  for (auto local_fpt : local_feature_points) 
  {
    if (match.find(local_fpt.first) != match.end()) 
    {
      hit_id_pair.push_back(make_pair(local_fpt.first, match[local_fpt.first]));
    }
  }

  // only one right answer. --> return true.
  if (hit_id_pair.size() >= 3) 
  {
    //D_INFO << BASHG << "[RELOCATE] " << BASHW << "got valid initial pair.";
    return true;
  }
  //D_INFO << BASHY << "[RETRY] " << BASHW << "CANNOT search valid initial pair.";
  return false;
}

/**
 * @brief
 *
 * @param feature_list [input] feature points list without feature ID; [output] feature points
 * list with feature ID.
 * @param adjacency_list [output] the feature map graph now
 * @param robot_pose [input] robot's pose at the last time, [output] robot's pose calculate this
 * time.
 * @return true pose calculate success.
 * @return false pose calculate fail.
 */
bool DustinMapping::InsertFeatureList(reflector_localization::Feature2DList &feature_list,
                         reflector_localization::AdjacencyList &adjacency_list,
                         reflector_localization::RobotPose &robot_pose) 
{
  static reflector_localization::RobotPose last_pose;
  static Eigen::Isometry2d matrixDelt = Eigen::Isometry2d::Identity();
  //D_INFO << "========================================";



  if (feature_list.feature_2d_with_ids.size() < 3) 
  {
    if (!feature_points.empty()) 
    {
      DropAdjacencyList(adjacency_list);
    }
    //D_WARN << BASHR << "[REJECT]" << BASHW << " feature points' size not enough.";
    return false;
  }

  //传入的feature_list转换成(fpt.x, fpt.y)格式
  vector<pair<double, double> > input_fpts;
  for (reflector_localization::Feature2DWithID fpt : feature_list.feature_2d_with_ids) 
  {
    input_fpts.push_back(make_pair(fpt.x, fpt.y));
  }

  // 将传入的feature_list，放进feature_points
  if (feature_points.empty()) 
  {
    InsertToFeatureGraph(input_fpts);
    if (!feature_points.empty()) 
    {
      DropAdjacencyList(adjacency_list);
    }
    return true;
  }

  reflector_localization::RobotPose pose;
  ///////////////////////////////////////////////////////////////
  if (DustinOptions_.wait_initial_pose) 
  {
    vector<pair<int, int> > hit_id_pair;
    if (!RelocalizationRobot(input_fpts, hit_id_pair))  // relocation unsuccess.
    {
      //D_WARN << BASHR << "[REJECT]" << BASHW << " none robotic's initial pose.";
      if (!feature_points.empty()) 
      {
        DropAdjacencyList(adjacency_list);
      }
      return false;
    }

    if (hit_id_pair.size() < 3) 
    {
      // //D_WARN << BASHR << "[REJECT]" << BASHW
      //        << " DustinOptions_.wait_initial_pose-yes-hit pairs' number not enough, add more reflect features here.";
      if (!feature_points.empty()) 
      {
        DropAdjacencyList(adjacency_list);
      }
      return false;
    }

    // //D_INFO << BASHB << "Data association initial success" << BASHW << ", Hit:" << hit_id_pair.size();

    //匹配上的全局坐标和局部坐标对。
    vector<pair<pair<double, double>, pair<double, double> > > localization_hit;
    for (auto pr : hit_id_pair) 
    {
      localization_hit.push_back(make_pair(feature_points[pr.second], input_fpts[pr.first]));
    }

    if (!InitialCurrentPose(localization_hit, pose)) 
    {
      D_WARN << BASHR << "[REJECT] " << BASHW << "robot pose initial pose get failed.";
      if (!feature_points.empty()) 
      {
        DropAdjacencyList(adjacency_list);
      }
      return false;
    }
    //D_INFO << "initial value: " << pose.x << ", " << pose.y
          //  << ", angle: " << pose.theta / M_PI * 180.0;

    OptimizeCurrentPose(localization_hit, pose);

    if (!feature_points.empty()) 
    {
      DropAdjacencyList(adjacency_list);
    }

    last_pose = pose;
    robot_pose = pose;

    DustinOptions_.wait_initial_pose = false;
    return true;
  }
  //////////////////////////////////////////////////////////////////////////////
  vector<pair<int, int> > hit_id_pair;
  vector<pair<double, double> > wait_insert;
  Eigen::Isometry2d EstimatePose = Eigen::Isometry2d::Identity();
  EstimatePose.pretranslate(Eigen::Vector2d(last_pose.x, last_pose.y));
  EstimatePose.rotate(last_pose.theta);

  SiftNewFeaturePoints(input_fpts, EstimatePose, hit_id_pair, wait_insert);
  D_INFO << BASHB << "Hit:" << hit_id_pair.size() << BASHW << ", Unhit: " << wait_insert.size();

  if (hit_id_pair.size() < 3) 
  {
    D_WARN << BASHR << "[REJECT]" << BASHW
           << " hit pairs' number not enough, add more reflect features here.";
    if (!feature_points.empty()) {
      DropAdjacencyList(adjacency_list);
    }

    // std::cout << "======== feature_graph ==========" << std::endl;
    for (auto fpt : feature_points) 
    {
      // D_INFO << fpt.first << ": " << fpt.second.first << ", " << fpt.second.second;
    }
    return false;
  }

  vector<pair<pair<double, double>, pair<double, double> > > localization_hit;
  for (auto pr : hit_id_pair) 
  {
    localization_hit.push_back(make_pair(feature_points[pr.second], input_fpts[pr.first]));
  }

  if (!InitialCurrentPose(localization_hit, pose)) 
  {
    pose = last_pose;
    D_WARN << "robot pose initial pose get failed, use last pose as initial value.";
  }
  D_INFO << "initial value: " << pose.x << ", " << pose.y
         << ", angle: " << pose.theta / M_PI * 180.0;

  OptimizeCurrentPose(localization_hit, pose);
  D_INFO << "optimized value: " << pose.x << ", " << pose.y
         << ", angle: " << pose.theta / M_PI * 180.0;

  Eigen::Isometry2d matrixThis = Eigen::Isometry2d::Identity();
  matrixThis.pretranslate(Eigen::Vector2d(pose.x, pose.y));
  matrixThis.rotate(pose.theta);

  vector<pair<double, double> > new_fpt_wait_insert;
  vector<int> new_fpt_assigned_id;
  for (auto unhit : wait_insert) 
  {
    Eigen::Vector3d local_fpt(unhit.first, unhit.second, 1.0);
    auto global_fpt = matrixThis * local_fpt;
    auto new_fpt = make_pair(global_fpt[0], global_fpt[1]);
    new_fpt_wait_insert.push_back(new_fpt);
    //D_INFO << "unhit: (" << global_fpt[0] << "," << global_fpt[1] << ")";
  }

  InsertToFeatureGraph(new_fpt_wait_insert);
  if (!feature_points.empty()) 
  {
    DropAdjacencyList(adjacency_list);
  }

  Eigen::Isometry2d matrixLast = Eigen::Isometry2d::Identity();
  matrixLast.pretranslate(Eigen::Vector2d(last_pose.x, last_pose.y));
  matrixLast.rotate(last_pose.theta);

  matrixDelt = matrixThis * matrixLast.inverse();

  last_pose = pose;
  robot_pose = pose;

  return true;
}

// DustinMapping::DustinMapping() : feature_point_cnt(0) { CLASS_LOG(DustinMapping) << "built!!"; }

void DustinMapping::SiftNewFeaturePoints(
    vector<pair<double, double> > &input_fpts, Eigen::Isometry2d &EstimatePose,
    vector<pair<int, int> > &hit_id_pair,
    vector<pair<double, double> > &feature_points_wait_insert) 
{
  int local_sz = input_fpts.size();

  for (int i = 0; i < local_sz; i++) 
  {
    Eigen::Vector3d local_fpt(input_fpts[i].first, input_fpts[i].second, 1.0);
    auto global_fpt = EstimatePose * local_fpt;
    bool got_hit = false;
    double min_distance = INT_MAX;
    for (auto fpt : feature_points) 
    {
      pair<double, double> lfpt = make_pair(global_fpt[0], global_fpt[1]);
      double n_distance = ComputeDistance(lfpt, fpt.second);
      min_distance = n_distance < min_distance ? n_distance : min_distance;
      if (n_distance < DustinOptions_.dustin_merge_distance) 
      {
        hit_id_pair.push_back(make_pair(i, fpt.first));
        got_hit = true;
        // TODO: maybe it will got not only one hit.
        break;
      }
    }
    // //D_INFO << "min_distance: " << min_distance;

    if (!got_hit) 
    {
      feature_points_wait_insert.push_back(input_fpts[i]);
    }
  }
}
/**
 * @brief
 *
 * @param match_result
 * @param robot_pose
 */
void DustinMapping::OptimizeCurrentPose(vector<pair<pair<double, double>, pair<double, double> > > &match_result,
                           reflector_localization::RobotPose &robot_pose) {
  vector<double> radius_list;

  ceres::Problem problem;
  double pose[3] = {robot_pose.x, robot_pose.y, robot_pose.theta};

  for (auto fpt_pair : match_result) {
    problem.AddResidualBlock(new ceres::AutoDiffCostFunction<FeaturePairCost, 2, 3>(
                                 new FeaturePairCost(fpt_pair.second, fpt_pair.first)),
                             new ceres::CauchyLoss(0.2), pose);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  robot_pose.x = pose[0];
  robot_pose.y = (pose[1]);
  robot_pose.theta = (pose[2]);
}

/**
 * @brief
 *
 * @param match_result global points - local points
 * @param robot_pose
 * @return true
 * @return false
 */
bool DustinMapping::InitialCurrentPose(vector<pair<pair<double, double>, pair<double, double> > > &match_result,
                          reflector_localization::RobotPose &robot_pose)
{
  int result_sz = match_result.size();
  vector<double> radius_list;
  for (int i = 0; i < result_sz; i++) {
    radius_list.push_back(ComputeRadius(match_result[i].second));
  }

  for (int i = 0; i < result_sz - 1; i++) 
  {
    for (int j = i + 1; j < result_sz; j++) 
    {
      double dij = ComputeDistance(match_result[i].first, match_result[j].first);
      double score = (radius_list[i] + radius_list[j]) / dij;
      if (score > 1.2) 
      {
        pair<double, double> res;
        if (CheckResultValid(radius_list, match_result, res, i, j, dij)) 
        {
          pair<double, double> global_tar_pt = match_result[i].first;
          pair<double, double> local_pt = match_result[i].second;
          double angle_global =
              atan2(global_tar_pt.second - res.second, global_tar_pt.first - res.first);
          double angle_local = atan2(local_pt.second, local_pt.first);

          double initial_value_angle = angle_global - angle_local;
          if (initial_value_angle > 2 * M_PI) 
          {
            initial_value_angle -= 2 * M_PI;
          } else if (initial_value_angle < -2 * M_PI) 
          {
            initial_value_angle += 2 * M_PI;
          }
          robot_pose.x = (res.first);
          robot_pose.y = (res.second);
          robot_pose.theta = (initial_value_angle);
          return true;
        }
      }
    }
  }

  return false;
}

bool DustinMapping::CheckResultValid(
    vector<double> &radius_list,
    vector<pair<pair<double, double>, pair<double, double> > > &match_result,
    pair<double, double> &result, int id1, int id2, double d12) {
  int result_sz = match_result.size();
  double r1 = radius_list[id1], r2 = radius_list[id2];
  pair<double, double> pt1 = match_result[id1].first, pt2 = match_result[id2].first;

  double a = (r1 * r1 - r2 * r2 + d12 * d12) / (2 * d12);
  double x0 = pt1.first + a / d12 * (pt2.first - pt1.first);
  double y0 = pt1.second + a / d12 * (pt2.second - pt1.second);

  double h = sqrt(r1 * r1 - a * a);
  pair<double, double> res_1, res_2;
  res_1.first = x0 + h / d12 * (pt2.second - pt1.second);
  res_1.second = y0 - h / d12 * (pt2.first - pt1.first);
  res_2.first = x0 - h / d12 * (pt2.second - pt1.second);
  res_2.second = y0 + h / d12 * (pt2.first - pt1.first);

  double distance_match_tolerance = DustinOptions_.distance_match_tolerance;

  for (int id = 0; id < result_sz; id++) {
    if (id != id1 && id != id2) {
      pair<double, double> pt_check = match_result[id].first;
      double r_check = radius_list[id];

      double check_distance_1 = fabs(ComputeDistance(res_1, pt_check) - r_check);
      double check_distance_2 = fabs(ComputeDistance(res_2, pt_check) - r_check);

      if (check_distance_1 <= check_distance_2) 
      {
        if (check_distance_1 < distance_match_tolerance) 
        {
          result = res_1;
          return true;
        }
        //D_WARN << "[REJECT] min distance: " << check_distance_1 << " is larger than "
              //  << distance_match_tolerance;
      } 
      else 
      {
        if (check_distance_2 < distance_match_tolerance) 
        {
          result = res_2;
          return true;
        }
        //D_WARN << "[REJECT] min distance: " << check_distance_2 << " is larger than "
              //  << distance_match_tolerance;
      }
    }
  }

  //D_WARN << "None result has been checked success for: (" << pt1.first << ", " << pt1.second
        //  << "), (" << pt2.first << ", " << pt2.second << ")";
  return false;
}
//先把adjacency_list清空，从feature_points和feature_graph拿点填充adjacency_list，
inline void DustinMapping::DropAdjacencyList(reflector_localization::AdjacencyList &adjacency_list) 
{
    reflector_localization::Feature2DWithID nwfpt;
    adjacency_list.feature_list.feature_2d_with_ids.clear();

    for (auto fpt : feature_points) 
    {
      nwfpt.x = (fpt.second.first);
      nwfpt.y = (fpt.second.second);
      nwfpt.ID = (fpt.first);
      adjacency_list.feature_list.feature_2d_with_ids.push_back(nwfpt);
    }

    adjacency_list.feature_2d_pairs.clear();
    set<pair<int, int> > pair_set;
    reflector_localization::Feature2DAdjacency nwpr;

    for (auto fptpr : feature_graph) 
    {
      for (auto pr : fptpr.second) 
      {
        nwpr.ID1 = fptpr.first;
        nwpr.ID2 = pr.first;
        adjacency_list.feature_2d_pairs.push_back(nwpr);
      }
    }
  current_graph = adjacency_list;
}


/**
 * @brief generate global feature point's graph
 *
 * @param fpts_wait_insert [input] feature points wait insert
 */

//把数据插入feature_points和feature_graph
void DustinMapping::InsertToFeatureGraph(vector<pair<double, double> > &fpts_wait_insert) 
{
  // Filter raw feature points for merge, prevent from insert a same map point in map twice.
  vector<pair<double, double> > input_fpts_wait_insert;
  vector<int> input_fpts_assigned_id;
  for (auto fpt : fpts_wait_insert) 
  {
    bool insert_to_graph = true;

    for (auto _g_fpt : feature_points) 
    {
      if (ComputeDistance(fpt, _g_fpt.second) < DustinOptions_.dustin_merge_distance) 
      {
        insert_to_graph = false;
        break;
      }
    }

    if (insert_to_graph) 
    {
      feature_points[DustinOptions_.feature_point_cnt] = fpt;
      input_fpts_wait_insert.push_back(fpt);
      input_fpts_assigned_id.push_back(DustinOptions_.feature_point_cnt);
      DustinOptions_.feature_point_cnt++;
    }
  }

  //D_INFO << "insert " << BASHG << input_fpts_wait_insert.size() << BASHW
        //  << " points to feature graph.";
  int cnt = 0;
  for (auto fpt : input_fpts_wait_insert) 
  {
    int fid = input_fpts_assigned_id[cnt++];
    CHECK(!feature_points.empty()) << "feature_points should not empty, check if you use "
                                      "GenerateFeatureGraph after feature_points initialized.";
    for (auto fnode : feature_points) 
    {
      int id = fnode.first;
      if (id == fid) 
      {
        continue;
      }
      auto fpt_in_graph = fnode.second;
      double distance =
          sqrt(pow(fpt_in_graph.first - fpt.first, 2) + pow(fpt_in_graph.second - fpt.second, 2));
      if (distance <= DustinOptions_.distance_ceil_threshold) 
      {
        feature_graph[fid].insert(make_pair(id, distance));
        feature_graph[id].insert(make_pair(fid, distance));
      }
    }
  }
}

inline double DustinMapping::ComputeDistance(pair<double, double> &pt1, pair<double, double> &pt2) 
{
  return sqrt(pow(pt1.first - pt2.first, 2) + pow(pt1.second - pt2.second, 2));
}

inline double DustinMapping::ComputeRadius(pair<double, double> &pt) 
{
  return sqrt(pow(pt.first, 2) + pow(pt.second, 2));
}


//序列化
bool DustinMapping::feature_map_saver(reflector_localization::Feature2DList& map_data, std::string& map_filename) {
    std::ofstream ofs(map_filename, std::ios::binary);
    if(!ofs) {
        ROS_ERROR("Failed to open file for saving: %s", map_filename.c_str());
        return false;
    }

    boost::archive::binary_oarchive oa(ofs);
    try 
    {
        oa << BOOST_SERIALIZATION_NVP(map_data);
    } 
    catch(const boost::archive::archive_exception& e) 
    {
        ROS_ERROR("Failed to save map data: %s", e.what());
        return false;
    }

    D_INFO<< "序列化反光柱地图成功，其中feature_points 的个数为： " << BASHR << map_data.feature_2d_with_ids.size();
    return true;
}

//反序列化
bool DustinMapping::feature_map_loader(std::string& map_filename, reflector_localization::Feature2DList* map_data) {
    std::ifstream ifs(map_filename, std::ios::binary);
    if(!ifs) 
    {
        ROS_ERROR("Failed to open file for loading: %s", map_filename.c_str());
        return false;
    }

    boost::archive::binary_iarchive ia(ifs);
    try 
    {
        ia >> BOOST_SERIALIZATION_NVP(*map_data);
    } catch(const boost::archive::archive_exception& e) 
    {
        ROS_ERROR("Failed to load map data: %s", e.what());
        return false;
    }

    return true;
}

}  // namespace Reflector_localization
