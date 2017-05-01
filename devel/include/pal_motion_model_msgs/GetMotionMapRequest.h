// Generated by gencpp from file pal_motion_model_msgs/GetMotionMapRequest.msg
// DO NOT EDIT!


#ifndef PAL_MOTION_MODEL_MSGS_MESSAGE_GETMOTIONMAPREQUEST_H
#define PAL_MOTION_MODEL_MSGS_MESSAGE_GETMOTIONMAPREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace pal_motion_model_msgs
{
template <class ContainerAllocator>
struct GetMotionMapRequest_
{
  typedef GetMotionMapRequest_<ContainerAllocator> Type;

  GetMotionMapRequest_()
    {
    }
  GetMotionMapRequest_(const ContainerAllocator& _alloc)
    {
    }






  typedef boost::shared_ptr< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetMotionMapRequest_

typedef ::pal_motion_model_msgs::GetMotionMapRequest_<std::allocator<void> > GetMotionMapRequest;

typedef boost::shared_ptr< ::pal_motion_model_msgs::GetMotionMapRequest > GetMotionMapRequestPtr;
typedef boost::shared_ptr< ::pal_motion_model_msgs::GetMotionMapRequest const> GetMotionMapRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_motion_model_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'pal_motion_model_msgs': ['/root/tiago_public_ws/src/pal_msgs/pal_motion_model_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_motion_model_msgs/GetMotionMapRequest";
  }

  static const char* value(const ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
";
  }

  static const char* value(const ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GetMotionMapRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::pal_motion_model_msgs::GetMotionMapRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // PAL_MOTION_MODEL_MSGS_MESSAGE_GETMOTIONMAPREQUEST_H
