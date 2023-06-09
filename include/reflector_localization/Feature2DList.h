// Generated by gencpp from file reflector_localization/Feature2DList.msg
// DO NOT EDIT!


#ifndef REFLECTOR_LOCALIZATION_MESSAGE_FEATURE2DLIST_H
#define REFLECTOR_LOCALIZATION_MESSAGE_FEATURE2DLIST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <reflector_localization/Feature2DWithID.h>

namespace reflector_localization
{
template <class ContainerAllocator>
struct Feature2DList_
{
  typedef Feature2DList_<ContainerAllocator> Type;

  Feature2DList_()
    : feature_2d_with_ids()  {
    }
  Feature2DList_(const ContainerAllocator& _alloc)
    : feature_2d_with_ids(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::reflector_localization::Feature2DWithID_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::reflector_localization::Feature2DWithID_<ContainerAllocator> >> _feature_2d_with_ids_type;
  _feature_2d_with_ids_type feature_2d_with_ids;





  typedef boost::shared_ptr< ::reflector_localization::Feature2DList_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reflector_localization::Feature2DList_<ContainerAllocator> const> ConstPtr;

}; // struct Feature2DList_

typedef ::reflector_localization::Feature2DList_<std::allocator<void> > Feature2DList;

typedef boost::shared_ptr< ::reflector_localization::Feature2DList > Feature2DListPtr;
typedef boost::shared_ptr< ::reflector_localization::Feature2DList const> Feature2DListConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reflector_localization::Feature2DList_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reflector_localization::Feature2DList_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::reflector_localization::Feature2DList_<ContainerAllocator1> & lhs, const ::reflector_localization::Feature2DList_<ContainerAllocator2> & rhs)
{
  return lhs.feature_2d_with_ids == rhs.feature_2d_with_ids;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::reflector_localization::Feature2DList_<ContainerAllocator1> & lhs, const ::reflector_localization::Feature2DList_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace reflector_localization

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::reflector_localization::Feature2DList_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflector_localization::Feature2DList_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflector_localization::Feature2DList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflector_localization::Feature2DList_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflector_localization::Feature2DList_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflector_localization::Feature2DList_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reflector_localization::Feature2DList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7a6ad08655535596730538aa544995ac";
  }

  static const char* value(const ::reflector_localization::Feature2DList_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7a6ad08655535596ULL;
  static const uint64_t static_value2 = 0x730538aa544995acULL;
};

template<class ContainerAllocator>
struct DataType< ::reflector_localization::Feature2DList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reflector_localization/Feature2DList";
  }

  static const char* value(const ::reflector_localization::Feature2DList_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reflector_localization::Feature2DList_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Feature2DWithID[] feature_2d_with_ids\n"
"================================================================================\n"
"MSG: reflector_localization/Feature2DWithID\n"
"float64 x\n"
"float64 y\n"
"int32 ID\n"
;
  }

  static const char* value(const ::reflector_localization::Feature2DList_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reflector_localization::Feature2DList_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.feature_2d_with_ids);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Feature2DList_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reflector_localization::Feature2DList_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reflector_localization::Feature2DList_<ContainerAllocator>& v)
  {
    s << indent << "feature_2d_with_ids[]" << std::endl;
    for (size_t i = 0; i < v.feature_2d_with_ids.size(); ++i)
    {
      s << indent << "  feature_2d_with_ids[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::reflector_localization::Feature2DWithID_<ContainerAllocator> >::stream(s, indent + "    ", v.feature_2d_with_ids[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // REFLECTOR_LOCALIZATION_MESSAGE_FEATURE2DLIST_H
