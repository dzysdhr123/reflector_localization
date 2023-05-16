#include<DBscanCluster.h>

namespace Reflector_localization
{
    DBscanCluster::DBscanCluster(const DBscanClusterOptions &options) : options_(options)
    {

        reflect_cylinder_radius = options_.reflect_cylinder_diameter/ 2.0;
        dbscan_eps_radius_square = pow(options_.dbscan_eps_radius, 2);
        dbscan_min_points = options_.dbscan_min_points;



    }

    DBscanCluster::~DBscanCluster(){
    }


    
bool DBscanCluster::Association(reflector_localization::IntensityRange2D &candidate_cloud, 
                                reflector_localization::Feature2DList &feature_list) 
{
    dataset.clear();
    for (auto p : candidate_cloud.data) 
    {
        DBscanClusterInterface::PointDBSCAN point(p.x, p.y);
        dataset.push_back(point);
    }


    int cluster_id = 1;
    auto iter = dataset.begin();
    for (; iter < dataset.end(); ++iter) 
    {
        if (iter->status == UNCLASSIFIED) 
        {
            if (expand_cluster(iter, cluster_id)) 
            {
                cluster_id++;
            }
        }
    }

    unordered_map<int, vector<int> > feature_dict;
    iter = dataset.begin();
    for (; iter < dataset.end(); ++iter) 
    {
        if (iter->status == CLASSIFIED) 
        {
            feature_dict[iter->id].push_back(iter - dataset.begin());
        }
    }

    vector<pair<double, double> > tmp_res;
    
    for (auto pr : feature_dict) 
    {
        double center_x = 0;
        double center_y = 0;
        for (int idx : pr.second) 
        {
            center_x += dataset.at(idx).x;
            center_y += dataset.at(idx).y;
        }
        int sz = pr.second.size();
        tmp_res.push_back(std::make_pair(center_x / double(sz), center_y / double(sz)));
    }

    merge_cluster(tmp_res);

    reflector_localization::Feature2DWithID new_feature_2d;
    for (auto pr : tmp_res) 
    {
        reflector_localization::Feature2DWithID new_feature_2d;

        new_feature_2d.ID = -1; 

        double distance = sqrt(pow(pr.first, 2) + pow(pr.second, 2));
        double factor = 1 + reflect_cylinder_radius / distance;

        new_feature_2d.x = factor * pr.first; 
        new_feature_2d.y = factor * pr.second; 
        feature_list.feature_2d_with_ids.push_back(new_feature_2d); 
    }



  return true;
}

bool DBscanCluster::expand_cluster(vector<PointDBSCAN>::iterator point, int cluster_id) {
  vector<int> cluster_seeds;
  calculate_cluster(cluster_seeds, point);

  if (cluster_seeds.size() < dbscan_min_points) 
  {
    point->status = NOISE;
    return false;
  } 
  else 
  {
    int index = 0, index_core = 0;
    auto iter = cluster_seeds.begin();
    for (; iter < cluster_seeds.end(); ++iter) 
    {
      dataset.at(*iter).id = cluster_id;
      dataset.at(*iter).status = CLASSIFIED;

      if (dataset.at(*iter).x == point->x && dataset.at(*iter).y == point->y) 
      {
        index_core = index;
      }
      ++index;
    }
    cluster_seeds.erase(cluster_seeds.begin() + index_core);

    // BFS
    int sz = cluster_seeds.size();
    for (int i = 0; i < sz; i++) {
      vector<int> cluster_neighors;
      if (dataset.at(cluster_seeds[i]).status == UNCLASSIFIED) {
        calculate_cluster(cluster_neighors, dataset.begin() + cluster_seeds[i]);
      }

      if (cluster_neighors.size() >= dbscan_min_points) {
        auto iter = cluster_neighors.begin();
        for (; iter < cluster_neighors.end(); ++iter) {
          if (dataset.at(*iter).status == UNCLASSIFIED || dataset.at(*iter).status == NOISE) {
            if (dataset.at(*iter).status == UNCLASSIFIED) {
              cluster_seeds.push_back(*iter);
              sz = cluster_seeds.size();
            }
            dataset.at(*iter).id = cluster_id;
            dataset.at(*iter).status = CLASSIFIED;
          }
        }
      }
    }

    return true;
  }

}


void DBscanCluster::calculate_cluster(vector<int>& cluster_index,
                            vector<PointDBSCAN>::iterator point)
{
    int index = 0;
    auto iter = dataset.begin();
    for (; iter < dataset.end(); ++iter) 
    {
        if (distance_square(point, iter) <= dbscan_eps_radius_square) 
        {
        cluster_index.push_back(index);
        }
        index++;
    }

}

inline double DBscanCluster::distance_square(vector<PointDBSCAN>::iterator point_core,
                                                 vector<PointDBSCAN>::iterator point_target) 
{
  return pow(point_core->x - point_target->x, 2) + pow(point_core->y - point_target->y, 2);
}


void DBscanCluster::merge_cluster(vector<pair<double, double> >& tmp_cluster) {
  int sz = tmp_cluster.size();
  int i = 0, j;
  bool got_merger_target = false;
  for (; i < sz; i++) {
    for (j = i + 1; j < sz; j++) {
      double new_distance = pow(tmp_cluster[i].first - tmp_cluster[j].first, 2) +
                            pow(tmp_cluster[i].second - tmp_cluster[j].second, 2);
      if (new_distance < options_.merge_distance) 
      {
        got_merger_target = true;
        // D_INFO << BASHY << "got merge target!" << BASHW;
        break;
      }
    }
    if (got_merger_target) {
      break;
    }
  }

  if (!got_merger_target) {
    return;
  }

  // D_INFO << "merge: [" << i << "][" << j << "], " << BASHY << "remove: [" << i << "]" << BASHW;

  tmp_cluster[i].first = (tmp_cluster[i].first + tmp_cluster[j].first) / 2.0;
  tmp_cluster[i].second = (tmp_cluster[i].second + tmp_cluster[j].second) / 2.0;
  tmp_cluster.erase(tmp_cluster.begin() + i);

  merge_cluster(tmp_cluster);
}

}