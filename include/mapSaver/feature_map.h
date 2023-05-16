
// #include <fstream>
// #include <string>


// // #include "ros_node.h"
// #include "reflector_localization/Feature2DList.h"
// #include "reflector_localization/IntensityPoint2D.h"



// using std::string;
// using std::fstream;
// using std::ios;

// namespace Reflector_localization 
// {

//     bool feature_map_saver(reflector_localization::Feature2DList& map_data, string& map_filename);

//     bool feature_map_loader(string& map_filename, reflector_localization::Feature2DList* map_data);

// }


#pragma once

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/vector.hpp>
#include "Feature2DList.h"

namespace boost {
namespace serialization {

template<class Archive>
void serialize(Archive& ar, reflector_localization::Feature2DWithID& feature, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(feature.x);
    ar & BOOST_SERIALIZATION_NVP(feature.y);
    ar & BOOST_SERIALIZATION_NVP(feature.ID);
}

template<class Archive>
void serialize(Archive& ar, reflector_localization::Feature2DList_<std::allocator<void> >& list, const unsigned int version)
{
    ar & BOOST_SERIALIZATION_NVP(list.feature_2d_with_ids);
}

} // namespace serialization
} // namespace boost
