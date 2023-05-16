// Generated by gencpp from file reflector_localization/AdjacencyList.msg
// DO NOT EDIT!


#ifndef REFLECTOR_LOCALIZATION_MESSAGE_ADJACENCYLIST_H
#define REFLECTOR_LOCALIZATION_MESSAGE_ADJACENCYLIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <reflector_localization/Feature2DList.h>
#include <reflector_localization/Feature2DAdjacency.h>

namespace reflector_localization
{
template <class ContainerAllocator>
struct AdjacencyList_
{
  typedef AdjacencyList_<ContainerAllocator> Type;

  AdjacencyList_()
    : feature_list()
    , feature_2d_pairs()  {
    }
  AdjacencyList_(const ContainerAllocator& _alloc)
    : feature_list(_alloc)
    , feature_2d_pairs(_alloc)  {
  (void)_alloc;
    }



   typedef  ::reflector_localization::Feature2DList_<ContainerAllocator>  _feature_list_type;
  _feature_list_type feature_list;

   typedef std::vector< ::reflector_localization::Feature2DAdjacency_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::reflector_localization::Feature2DAdjacency_<ContainerAllocator> >> _feature_2d_pairs_type;
  _feature_2d_pairs_type feature_2d_pairs;





  typedef boost::shared_ptr< ::reflector_localization::AdjacencyList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reflector_localization::AdjacencyList_<ContainerAllocator> const> ConstPtr;

}; // struct AdjacencyList_

typedef ::reflector_localization::AdjacencyList_<std::allocator<void> > AdjacencyList;

typedef boost::shared_ptr< ::reflector_localization::AdjacencyList > AdjacencyListPtr;
typedef boost::shared_ptr< ::reflector_localization::AdjacencyList const> AdjacencyListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reflector_localization::AdjacencyList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reflector_localization::AdjacencyList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::reflector_localization::AdjacencyList_<ContainerAllocator1> & lhs, const ::reflector_localization::AdjacencyList_<ContainerAllocator2> & rhs)
{
  return lhs.feature_list == rhs.feature_list &&
    lhs.feature_2d_pairs == rhs.feature_2d_pairs;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::reflector_localization::AdjacencyList_<ContainerAllocator1> & lhs, const ::reflector_localization::AdjacencyList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace reflector_localization

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::reflector_localization::AdjacencyList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflector_localization::AdjacencyList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflector_localization::AdjacencyList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflector_localization::AdjacencyList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflector_localization::AdjacencyList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflector_localization::AdjacencyList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reflector_localization::AdjacencyList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3fde34b4533c17ee437a93a30ab9fb91";
  }

  static const char* value(const ::reflector_localization::AdjacencyList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3fde34b4533c17eeULL;
  static const uint64_t static_value2 = 0x437a93a30ab9fb91ULL;
};

template<class ContainerAllocator>
struct DataType< ::reflector_localization::AdjacencyList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reflector_localization/AdjacencyList";
  }

  static const char* value(const ::reflector_localization::AdjacencyList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reflector_localization::AdjacencyList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Feature2DList feature_list\n"
"Feature2DAdjacency[] feature_2d_pairs\n"
"================================================================================\n"
"MSG: reflector_localization/Feature2DList\n"
"Feature2DWithID[] feature_2d_with_ids\n"
"================================================================================\n"
"MSG: reflector_localization/Feature2DWithID\n"
"float64 x\n"
"float64 y\n"
"int32 ID\n"
"================================================================================\n"
"MSG: reflector_localization/Feature2DAdjacency\n"
"int32 ID1\n"
"int32 ID2\n"
;
  }

  static const char* value(const ::reflector_localization::AdjacencyList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reflector_localization::AdjacencyList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.feature_list);
      stream.next(m.feature_2d_pairs);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct AdjacencyList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reflector_localization::AdjacencyList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reflector_localization::AdjacencyList_<ContainerAllocator>& v)
  {
    s << indent << "feature_list: ";
    s << std::endl;
    Printer< ::reflector_localization::Feature2DList_<ContainerAllocator> >::stream(s, indent + "  ", v.feature_list);
    s << indent << "feature_2d_pairs[]" << std::endl;
    for (size_t i = 0; i < v.feature_2d_pairs.size(); ++i)
    {
      s << indent << "  feature_2d_pairs[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::reflector_localization::Feature2DAdjacency_<ContainerAllocator> >::stream(s, indent + "    ", v.feature_2d_pairs[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // REFLECTOR_LOCALIZATION_MESSAGE_ADJACENCYLIST_H