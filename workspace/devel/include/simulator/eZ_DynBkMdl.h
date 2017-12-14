// Generated by gencpp from file simulator/eZ_DynBkMdl.msg
// DO NOT EDIT!


#ifndef SIMULATOR_MESSAGE_EZ_DYNBKMDL_H
#define SIMULATOR_MESSAGE_EZ_DYNBKMDL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace simulator
{
template <class ContainerAllocator>
struct eZ_DynBkMdl_
{
  typedef eZ_DynBkMdl_<ContainerAllocator> Type;

  eZ_DynBkMdl_()
    : s(0.0)
    , ey(0.0)
    , epsi(0.0)
    , v_x(0.0)
    , v_y(0.0)
    , psi_dot(0.0)  {
    }
  eZ_DynBkMdl_(const ContainerAllocator& _alloc)
    : s(0.0)
    , ey(0.0)
    , epsi(0.0)
    , v_x(0.0)
    , v_y(0.0)
    , psi_dot(0.0)  {
  (void)_alloc;
    }



   typedef float _s_type;
  _s_type s;

   typedef float _ey_type;
  _ey_type ey;

   typedef float _epsi_type;
  _epsi_type epsi;

   typedef float _v_x_type;
  _v_x_type v_x;

   typedef float _v_y_type;
  _v_y_type v_y;

   typedef float _psi_dot_type;
  _psi_dot_type psi_dot;




  typedef boost::shared_ptr< ::simulator::eZ_DynBkMdl_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::simulator::eZ_DynBkMdl_<ContainerAllocator> const> ConstPtr;

}; // struct eZ_DynBkMdl_

typedef ::simulator::eZ_DynBkMdl_<std::allocator<void> > eZ_DynBkMdl;

typedef boost::shared_ptr< ::simulator::eZ_DynBkMdl > eZ_DynBkMdlPtr;
typedef boost::shared_ptr< ::simulator::eZ_DynBkMdl const> eZ_DynBkMdlConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::simulator::eZ_DynBkMdl_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace simulator

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
<<<<<<< HEAD
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'simulator': ['/home/odroid/barc/workspace/src/simulator/msg']}
=======
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'simulator': ['/home/heyin/barc/workspace/src/simulator/msg']}
>>>>>>> 7747ef15b6d1279a89cd10799f202a75fc3e3ab6

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::simulator::eZ_DynBkMdl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::simulator::eZ_DynBkMdl_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::simulator::eZ_DynBkMdl_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "19de479b90a68da73ee59a1fb6a50755";
  }

  static const char* value(const ::simulator::eZ_DynBkMdl_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x19de479b90a68da7ULL;
  static const uint64_t static_value2 = 0x3ee59a1fb6a50755ULL;
};

template<class ContainerAllocator>
struct DataType< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "simulator/eZ_DynBkMdl";
  }

  static const char* value(const ::simulator::eZ_DynBkMdl_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 s\n\
float32 ey\n\
float32 epsi\n\
float32 v_x\n\
float32 v_y\n\
float32 psi_dot\n\
";
  }

  static const char* value(const ::simulator::eZ_DynBkMdl_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.s);
      stream.next(m.ey);
      stream.next(m.epsi);
      stream.next(m.v_x);
      stream.next(m.v_y);
      stream.next(m.psi_dot);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct eZ_DynBkMdl_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::simulator::eZ_DynBkMdl_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::simulator::eZ_DynBkMdl_<ContainerAllocator>& v)
  {
    s << indent << "s: ";
    Printer<float>::stream(s, indent + "  ", v.s);
    s << indent << "ey: ";
    Printer<float>::stream(s, indent + "  ", v.ey);
    s << indent << "epsi: ";
    Printer<float>::stream(s, indent + "  ", v.epsi);
    s << indent << "v_x: ";
    Printer<float>::stream(s, indent + "  ", v.v_x);
    s << indent << "v_y: ";
    Printer<float>::stream(s, indent + "  ", v.v_y);
    s << indent << "psi_dot: ";
    Printer<float>::stream(s, indent + "  ", v.psi_dot);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SIMULATOR_MESSAGE_EZ_DYNBKMDL_H
