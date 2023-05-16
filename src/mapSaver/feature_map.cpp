// #include "feature_map.h"
// #include "ros/ros.h"
// #include "ros/serialization.h"
// #include "ros/message_traits.h"

// #include <boost/archive/text_oarchive.hpp>
// #include <boost/archive/text_iarchive.hpp>
// #include <boost/serialization/serialization.hpp>
// #include <boost/archive/binary_oarchive.hpp>
// #include <boost/archive/binary_iarchive.hpp>


// namespace Reflector_localization
// {
//     bool feature_map_saver(reflector_localization::Feature2DList& map_data, string& map_filename)
//     {
//         // uint32_t serial_size = ros::serialization::serializationLength(map_data);
//         // boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

//         // ros::serialization::OStream stream(buffer.get(), serial_size);
//         // ros::serialization::serialize(stream, map_data);

//         // std::ofstream map_writer(map_filename, std::ios::out | std::ios::binary);
//         // map_writer.write(reinterpret_cast<const char*>(buffer.get()), serial_size);
//         // map_writer.close();

//         printf("Save To File %s \n", map_filename.c_str());
//         std::ofstream ofs(map_filename.c_str());
//         boost::archive::binary_oarchive oa(ofs, boost::archive::no_codecvt);
//         oa << BOOST_SERIALIZATION_NVP(map_data);

//         return true;
//     }

//     bool feature_map_loader(string& map_filename, reflector_localization::Feature2DList* map_data)
//     {
//         // std::ifstream map_reader(map_filename, std::ios::in | std::ios::binary | std::ios::ate);
//         // std::streamsize size = map_reader.tellg();
//         // map_reader.seekg(0, std::ios::beg);

//         // boost::shared_array<uint8_t> buffer(new uint8_t[size]);
//         // map_reader.read(reinterpret_cast<char*>(buffer.get()), size);
//         // map_reader.close();

//         // ros::serialization::IStream stream(buffer.get(), size);
//         // ros::serialization::deserialize(stream, *map_data);
//         printf("Load From File %s \n", map_filename.c_str());
//         std::ifstream ifs(map_filename.c_str());
//         boost::archive::binary_iarchive ia(ifs, boost::archive::no_codecvt);
//         ia >> BOOST_SERIALIZATION_NVP(map_data);

//         return true;
//     }
// }

