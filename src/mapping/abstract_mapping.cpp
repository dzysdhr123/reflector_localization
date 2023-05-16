// #include "mapping/abstract_mapping.h"
// #include "mapSaver/feature_map.h"

// namespace Reflector_localization {

// // void AbstractMapping::SetMappingOptions(const reflector_localization::MappingOptions &options) {
// //     _options = options;
// //     if (options.load_map) {
// //         LoadExistMapFile();
// //     }
// // }

// void AbstractMapping::SaveMap() {
//     reflector_localization::Feature2DList feature_list = current_graph.feature_list;
//     std::string map_filename = _options.map_filename;
//     feature_map_saver(feature_list, map_filename);
// }

// bool AbstractMapping::LoadExistMapFile() {
//     static reflector_localization::Feature2DList feature_list;
//     std::string map_filename = _options.map_filename;
//     feature_map_loader(map_filename, &current_graph.feature_list);

//     return true;
// }

// }  // namespace Reflector_localization
