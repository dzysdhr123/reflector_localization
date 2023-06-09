// Generated by gencpp from file reflector_localization/IntensityRange2D.msg
// DO NOT EDIT!


#ifndef REFLECTOR_LOCALIZATION_MESSAGE_INTENSITYRANGE2D_H
#define REFLECTOR_LOCALIZATION_MESSAGE_INTENSITYRANGE2D_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <reflector_localization/IntensityPoint2D.h>

namespace reflector_localization
{
template <class ContainerAllocator>
struct IntensityRange2D_
{
  typedef IntensityRange2D_<ContainerAllocator> Type;

  IntensityRange2D_()
    : timestamp(0)
    , frame_id()
    , data()  {
    }
  IntensityRange2D_(const ContainerAllocator& _alloc)
    : timestamp(0)
    , frame_id(_alloc)
    , data(_alloc)  {
  (void)_alloc;
    }



   typedef int64_t _timestamp_type;
  _timestamp_type timestamp;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _frame_id_type;
  _frame_id_type frame_id;

   typedef std::vector< ::reflector_localization::IntensityPoint2D_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::reflector_localization::IntensityPoint2D_<ContainerAllocator> >> _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::reflector_localization::IntensityRange2D_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::reflector_localization::IntensityRange2D_<ContainerAllocator> const> ConstPtr;

}; // struct IntensityRange2D_

typedef ::reflector_localization::IntensityRange2D_<std::allocator<void> > IntensityRange2D;

typedef boost::shared_ptr< ::reflector_localization::IntensityRange2D > IntensityRange2DPtr;
typedef boost::shared_ptr< ::reflector_localization::IntensityRange2D const> IntensityRange2DConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::reflector_localization::IntensityRange2D_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::reflector_localization::IntensityRange2D_<ContainerAllocator1> & lhs, const ::reflector_localization::IntensityRange2D_<ContainerAllocator2> & rhs)
{
  return lhs.timestamp == rhs.timestamp &&
    lhs.frame_id == rhs.frame_id &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::reflector_localization::IntensityRange2D_<ContainerAllocator1> & lhs, const ::reflector_localization::IntensityRange2D_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace reflector_localization

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::reflector_localization::IntensityRange2D_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::reflector_localization::IntensityRange2D_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::reflector_localization::IntensityRange2D_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4b02959040f7ff58b50fe6709f35b06b";
  }

  static const char* value(const ::reflector_localization::IntensityRange2D_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4b02959040f7ff58ULL;
  static const uint64_t static_value2 = 0xb50fe6709f35b06bULL;
};

template<class ContainerAllocator>
struct DataType< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "reflector_localization/IntensityRange2D";
  }

  static const char* value(const ::reflector_localization::IntensityRange2D_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# IntensityRange2D 激光点集合\n"
"int64 timestamp\n"
"string frame_id\n"
"IntensityPoint2D[] data\n"
"\n"
"================================================================================\n"
"MSG: reflector_localization/IntensityPoint2D\n"
"#IntensityPoint2D 单个点的强度\n"
"float64 x\n"
"float64 y\n"
"float64 intensity\n"
;
  }

  static const char* value(const ::reflector_localization::IntensityRange2D_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.timestamp);
      stream.next(m.frame_id);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IntensityRange2D_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::reflector_localization::IntensityRange2D_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::reflector_localization::IntensityRange2D_<ContainerAllocator>& v)
  {
    s << indent << "timestamp: ";
    Printer<int64_t>::stream(s, indent + "  ", v.timestamp);
    s << indent << "frame_id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.frame_id);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::reflector_localization::IntensityPoint2D_<ContainerAllocator> >::stream(s, indent + "    ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // REFLECTOR_LOCALIZATION_MESSAGE_INTENSITYRANGE2D_H
