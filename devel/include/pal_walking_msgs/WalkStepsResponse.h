// Generated by gencpp from file pal_walking_msgs/WalkStepsResponse.msg
// DO NOT EDIT!


#ifndef PAL_WALKING_MSGS_MESSAGE_WALKSTEPSRESPONSE_H
#define PAL_WALKING_MSGS_MESSAGE_WALKSTEPSRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pal_walking_msgs
{
template <class ContainerAllocator>
struct WalkStepsResponse_
{
  typedef WalkStepsResponse_<ContainerAllocator> Type;

  WalkStepsResponse_()
    {
    }
  WalkStepsResponse_(const ContainerAllocator& _alloc)
    {
    }






  typedef boost::shared_ptr< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> const> ConstPtr;

}; // struct WalkStepsResponse_

typedef ::pal_walking_msgs::WalkStepsResponse_<std::allocator<void> > WalkStepsResponse;

typedef boost::shared_ptr< ::pal_walking_msgs::WalkStepsResponse > WalkStepsResponsePtr;
typedef boost::shared_ptr< ::pal_walking_msgs::WalkStepsResponse const> WalkStepsResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_walking_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'pal_walking_msgs': ['/root/tiago_public_ws/src/pal_msgs/pal_walking_msgs/msg'], 'humanoid_nav_msgs': ['/opt/ros/indigo/share/humanoid_nav_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_walking_msgs/WalkStepsResponse";
  }

  static const char* value(const ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct WalkStepsResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::pal_walking_msgs::WalkStepsResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // PAL_WALKING_MSGS_MESSAGE_WALKSTEPSRESPONSE_H
