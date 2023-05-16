// #pragma once

// #include <fstream>

// // #include "venus/common/config_file_resolver.h"
// // #include "venus/common/dustin_log.h"
// // #include "venus/common/lua_param_dictionary.h"

// // #include "venus/mapping/proto/mapping_options.pb.h"
// // #include "venus/sensor/proto/sensor.pb.h"
// // #include "venus/sensor/feature_map_io.h"
// // #include "ros_node.h"
// #include "reflector_localization/Feature2DList.h"
// #include "reflector_localization/AdjacencyList.h"
// #include "reflector_localization/RobotPose.h"
// #include "reflector_localization/MappingOptions.h"


// namespace Reflector_localization{

// class AbstractMapping {
//  public:
//   AbstractMapping(/* args */) {
    
//   }
//   virtual ~AbstractMapping() = default;

//   // virtual bool Init() = 0;
//   virtual bool InsertFeatureList(reflector_localization::Feature2DList &feature_list,
//                          reflector_localization::AdjacencyList &adjacency_list,
//                          reflector_localization::RobotPose &robot_pose) = 0;

//   virtual bool SetInitialPose(reflector_localization::RobotPose &pose) = 0;
//   // void SetMappingOptions(const reflector_localization::MappingOptions &options);

//   void SaveMap();
//   bool LoadExistMapFile();

//   reflector_localization::MappingOptions _options;
//   reflector_localization::AdjacencyList current_graph;
// };


// }  // namespace Reflector_localization
