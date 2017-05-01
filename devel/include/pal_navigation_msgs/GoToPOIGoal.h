// Generated by gencpp from file pal_navigation_msgs/GoToPOIGoal.msg
// DO NOT EDIT!


#ifndef PAL_NAVIGATION_MSGS_MESSAGE_GOTOPOIGOAL_H
#define PAL_NAVIGATION_MSGS_MESSAGE_GOTOPOIGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>

namespace pal_navigation_msgs
{
template <class ContainerAllocator>
struct GoToPOIGoal_
{
  typedef GoToPOIGoal_<ContainerAllocator> Type;

  GoToPOIGoal_()
    : poi()  {
    }
  GoToPOIGoal_(const ContainerAllocator& _alloc)
    : poi(_alloc)  {
    }



   typedef  ::std_msgs::String_<ContainerAllocator>  _poi_type;
  _poi_type poi;




  typedef boost::shared_ptr< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> const> ConstPtr;

}; // struct GoToPOIGoal_

typedef ::pal_navigation_msgs::GoToPOIGoal_<std::allocator<void> > GoToPOIGoal;

typedef boost::shared_ptr< ::pal_navigation_msgs::GoToPOIGoal > GoToPOIGoalPtr;
typedef boost::shared_ptr< ::pal_navigation_msgs::GoToPOIGoal const> GoToPOIGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_navigation_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'pal_navigation_msgs': ['/root/tiago_public_ws/devel/share/pal_navigation_msgs/msg', '/root/tiago_public_ws/src/pal_msgs/pal_navigation_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "538be845cb53d7d128987a020f2801b8";
  }

  static const char* value(const ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x538be845cb53d7d1ULL;
  static const uint64_t static_value2 = 0x28987a020f2801b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_navigation_msgs/GoToPOIGoal";
  }

  static const char* value(const ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
std_msgs/String poi\n\
\n\
================================================================================\n\
MSG: std_msgs/String\n\
string data\n\
";
  }

  static const char* value(const ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.poi);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GoToPOIGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_navigation_msgs::GoToPOIGoal_<ContainerAllocator>& v)
  {
    s << indent << "poi: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.poi);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_NAVIGATION_MSGS_MESSAGE_GOTOPOIGOAL_H
