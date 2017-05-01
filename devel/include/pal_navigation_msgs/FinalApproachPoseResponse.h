// Generated by gencpp from file pal_navigation_msgs/FinalApproachPoseResponse.msg
// DO NOT EDIT!


#ifndef PAL_NAVIGATION_MSGS_MESSAGE_FINALAPPROACHPOSERESPONSE_H
#define PAL_NAVIGATION_MSGS_MESSAGE_FINALAPPROACHPOSERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Bool.h>

namespace pal_navigation_msgs
{
template <class ContainerAllocator>
struct FinalApproachPoseResponse_
{
  typedef FinalApproachPoseResponse_<ContainerAllocator> Type;

  FinalApproachPoseResponse_()
    : done()  {
    }
  FinalApproachPoseResponse_(const ContainerAllocator& _alloc)
    : done(_alloc)  {
    }



   typedef  ::std_msgs::Bool_<ContainerAllocator>  _done_type;
  _done_type done;




  typedef boost::shared_ptr< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> const> ConstPtr;

}; // struct FinalApproachPoseResponse_

typedef ::pal_navigation_msgs::FinalApproachPoseResponse_<std::allocator<void> > FinalApproachPoseResponse;

typedef boost::shared_ptr< ::pal_navigation_msgs::FinalApproachPoseResponse > FinalApproachPoseResponsePtr;
typedef boost::shared_ptr< ::pal_navigation_msgs::FinalApproachPoseResponse const> FinalApproachPoseResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_navigation_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'pal_navigation_msgs': ['/root/tiago_public_ws/devel/share/pal_navigation_msgs/msg', '/root/tiago_public_ws/src/pal_msgs/pal_navigation_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "1c236c9790b7f5f3c3164ceb3563eae0";
  }

  static const char* value(const ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x1c236c9790b7f5f3ULL;
  static const uint64_t static_value2 = 0xc3164ceb3563eae0ULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_navigation_msgs/FinalApproachPoseResponse";
  }

  static const char* value(const ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
std_msgs/Bool done\n\
\n\
\n\
\n\
================================================================================\n\
MSG: std_msgs/Bool\n\
bool data\n\
";
  }

  static const char* value(const ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.done);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct FinalApproachPoseResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_navigation_msgs::FinalApproachPoseResponse_<ContainerAllocator>& v)
  {
    s << indent << "done: ";
    s << std::endl;
    Printer< ::std_msgs::Bool_<ContainerAllocator> >::stream(s, indent + "  ", v.done);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_NAVIGATION_MSGS_MESSAGE_FINALAPPROACHPOSERESPONSE_H
