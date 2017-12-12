// Generated by gencpp from file barc/obj_offset.msg
// DO NOT EDIT!


#ifndef BARC_MESSAGE_OBJ_OFFSET_H
#define BARC_MESSAGE_OBJ_OFFSET_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace barc
{
template <class ContainerAllocator>
struct obj_offset_
{
  typedef obj_offset_<ContainerAllocator> Type;

  obj_offset_()
    : pixel_center_offset(0.0)
    , est_dist(0.0)
    , est_center_offset(0.0)  {
    }
  obj_offset_(const ContainerAllocator& _alloc)
    : pixel_center_offset(0.0)
    , est_dist(0.0)
    , est_center_offset(0.0)  {
  (void)_alloc;
    }



   typedef float _pixel_center_offset_type;
  _pixel_center_offset_type pixel_center_offset;

   typedef float _est_dist_type;
  _est_dist_type est_dist;

   typedef float _est_center_offset_type;
  _est_center_offset_type est_center_offset;




  typedef boost::shared_ptr< ::barc::obj_offset_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::barc::obj_offset_<ContainerAllocator> const> ConstPtr;

}; // struct obj_offset_

typedef ::barc::obj_offset_<std::allocator<void> > obj_offset;

typedef boost::shared_ptr< ::barc::obj_offset > obj_offsetPtr;
typedef boost::shared_ptr< ::barc::obj_offset const> obj_offsetConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::barc::obj_offset_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::barc::obj_offset_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace barc

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'barc': ['/home/odroid/barc/workspace/src/barc/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::barc::obj_offset_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::barc::obj_offset_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::barc::obj_offset_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::barc::obj_offset_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::barc::obj_offset_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::barc::obj_offset_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::barc::obj_offset_<ContainerAllocator> >
{
  static const char* value()
  {
    return "906dd9894d5ba5a23af5c11934a5eac3";
  }

  static const char* value(const ::barc::obj_offset_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x906dd9894d5ba5a2ULL;
  static const uint64_t static_value2 = 0x3af5c11934a5eac3ULL;
};

template<class ContainerAllocator>
struct DataType< ::barc::obj_offset_<ContainerAllocator> >
{
  static const char* value()
  {
    return "barc/obj_offset";
  }

  static const char* value(const ::barc::obj_offset_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::barc::obj_offset_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 pixel_center_offset\n\
float32 est_dist\n\
float32 est_center_offset\n\
";
  }

  static const char* value(const ::barc::obj_offset_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::barc::obj_offset_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pixel_center_offset);
      stream.next(m.est_dist);
      stream.next(m.est_center_offset);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct obj_offset_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::barc::obj_offset_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::barc::obj_offset_<ContainerAllocator>& v)
  {
    s << indent << "pixel_center_offset: ";
    Printer<float>::stream(s, indent + "  ", v.pixel_center_offset);
    s << indent << "est_dist: ";
    Printer<float>::stream(s, indent + "  ", v.est_dist);
    s << indent << "est_center_offset: ";
    Printer<float>::stream(s, indent + "  ", v.est_center_offset);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BARC_MESSAGE_OBJ_OFFSET_H
