// Generated by gencpp from file pal_device_msgs/CancelEffectResponse.msg
// DO NOT EDIT!


#ifndef PAL_DEVICE_MSGS_MESSAGE_CANCELEFFECTRESPONSE_H
#define PAL_DEVICE_MSGS_MESSAGE_CANCELEFFECTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pal_device_msgs
{
template <class ContainerAllocator>
struct CancelEffectResponse_
{
  typedef CancelEffectResponse_<ContainerAllocator> Type;

  CancelEffectResponse_()
    {
    }
  CancelEffectResponse_(const ContainerAllocator& _alloc)
    {
    }






  typedef boost::shared_ptr< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> const> ConstPtr;

}; // struct CancelEffectResponse_

typedef ::pal_device_msgs::CancelEffectResponse_<std::allocator<void> > CancelEffectResponse;

typedef boost::shared_ptr< ::pal_device_msgs::CancelEffectResponse > CancelEffectResponsePtr;
typedef boost::shared_ptr< ::pal_device_msgs::CancelEffectResponse const> CancelEffectResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_device_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'pal_device_msgs': ['/root/tiago_public_ws/src/pal_msgs/pal_device_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_device_msgs/CancelEffectResponse";
  }

  static const char* value(const ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
";
  }

  static const char* value(const ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct CancelEffectResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::pal_device_msgs::CancelEffectResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // PAL_DEVICE_MSGS_MESSAGE_CANCELEFFECTRESPONSE_H
