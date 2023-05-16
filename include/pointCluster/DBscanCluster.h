#pragma once



#include "pointCluster/DBscanClusterInterface.h"

using std::unordered_map;
using std::vector;
using std::pair;

namespace Reflector_localization
{

    class DBscanCluster : public DBscanClusterInterface
    {

        public:
            DBscanCluster(const DBscanClusterOptions &options);
            DBscanCluster() = delete;
            ~DBscanCluster() override;


            bool Association(reflector_localization::IntensityRange2D &candidate_cloud, 
                                reflector_localization::Feature2DList &feature_list) override;

            bool expand_cluster(vector<PointDBSCAN>::iterator point, int cluster_id) override;

            void calculate_cluster(vector<int>& cluster_index,
                                            vector<PointDBSCAN>::iterator point) override;

            void merge_cluster(vector<pair<double, double> >& tmp_cluster);

            inline double distance_square(vector<PointDBSCAN>::iterator point_core,
                                                            vector<PointDBSCAN>::iterator point_target) ;

        private:

            // double reflect_cylinder_radius;
            // unsigned int dbscan_min_points;
            // float dbscan_eps_radius_square;


        
        
        
        
        
        
        //参数初始化
        DBscanClusterOptions options_;

    };

}  // namespace Reflector_localization