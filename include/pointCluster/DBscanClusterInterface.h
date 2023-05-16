#pragma once
using namespace std;


#include <unordered_map>
#include <vector>
#include<IntensityRange2D.h>
#include<PointDBSCAN.h>
#include<Feature2DList.h>
#include "common/dustin_log.h"
// #include<PointDBSCAN.h>
namespace Reflector_localization
{


    class DBscanClusterInterface
    {
        public:
            struct DBscanClusterOptions
            {
                double reflect_cylinder_diameter;
                double dbscan_eps_radius;
                int dbscan_min_points;
                double merge_distance;

            };
            enum PointStatus { FAILURE = -3, NOISE, UNCLASSIFIED, CLASSIFIED };
            struct PointDBSCAN 
            {
                PointStatus status;
                int id;
                double x;
                double y;

            PointDBSCAN(double px, double py) 
            {
                x = px;
                y = py;
                status = UNCLASSIFIED;
                id = 0;
                }
            };
            DBscanClusterOptions options_;

            DBscanClusterInterface() = default;
            virtual ~DBscanClusterInterface() = default;
            virtual bool Association(reflector_localization::IntensityRange2D &candidate_cloud, 
                                            reflector_localization::Feature2DList &feature_list) = 0;
            virtual bool expand_cluster(vector<PointDBSCAN>::iterator point, int cluster_id) = 0;
            virtual void calculate_cluster(vector<int>& cluster_index,
                                            vector<PointDBSCAN>::iterator point) = 0;


            void SetAssocoiationOptions(const DBscanClusterInterface &options);

            public:
                float reflect_cylinder_radius;
                unsigned int dbscan_min_points;
                float dbscan_eps_radius_square;
                vector<PointDBSCAN> dataset;
                vector<pair<double, double> > center_list;


            





    };



}  // namespace Reflector_localization

