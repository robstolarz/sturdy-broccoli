// Generated by gencpp from file pal_device_msgs/TimedFadeEffectRequest.msg
// DO NOT EDIT!


#ifndef PAL_DEVICE_MSGS_MESSAGE_TIMEDFADEEFFECTREQUEST_H
#define PAL_DEVICE_MSGS_MESSAGE_TIMEDFADEEFFECTREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pal_device_msgs/LedGroup.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/ColorRGBA.h>

namespace pal_device_msgs
{
template <class ContainerAllocator>
struct TimedFadeEffectRequest_
{
  typedef TimedFadeEffectRequest_<ContainerAllocator> Type;

  TimedFadeEffectRequest_()
    : leds()
    , firstColor()
    , secondColor()
    , colorChangeDuration()
    , reverseFade(false)
    , effectDuration()
    , priority(0)  {
    }
  TimedFadeEffectRequest_(const ContainerAllocator& _alloc)
    : leds(_alloc)
    , firstColor(_alloc)
    , secondColor(_alloc)
    , colorChangeDuration()
    , reverseFade(false)
    , effectDuration()
    , priority(0)  {
    }



   typedef  ::pal_device_msgs::LedGroup_<ContainerAllocator>  _leds_type;
  _leds_type leds;

   typedef  ::std_msgs::ColorRGBA_<ContainerAllocator>  _firstColor_type;
  _firstColor_type firstColor;

   typedef  ::std_msgs::ColorRGBA_<ContainerAllocator>  _secondColor_type;
  _secondColor_type secondColor;

   typedef ros::Duration _colorChangeDuration_type;
  _colorChangeDuration_type colorChangeDuration;

   typedef uint8_t _reverseFade_type;
  _reverseFade_type reverseFade;

   typedef ros::Duration _effectDuration_type;
  _effectDuration_type effectDuration;

   typedef uint8_t _priority_type;
  _priority_type priority;




  typedef boost::shared_ptr< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> const> ConstPtr;

}; // struct TimedFadeEffectRequest_

typedef ::pal_device_msgs::TimedFadeEffectRequest_<std::allocator<void> > TimedFadeEffectRequest;

typedef boost::shared_ptr< ::pal_device_msgs::TimedFadeEffectRequest > TimedFadeEffectRequestPtr;
typedef boost::shared_ptr< ::pal_device_msgs::TimedFadeEffectRequest const> TimedFadeEffectRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4b26fa459195deaf5bbcf5309bfe3b62";
  }

  static const char* value(const ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4b26fa459195deafULL;
  static const uint64_t static_value2 = 0x5bbcf5309bfe3b62ULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_device_msgs/TimedFadeEffectRequest";
  }

  static const char* value(const ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
LedGroup leds\n\
\n\
\n\
std_msgs/ColorRGBA firstColor\n\
std_msgs/ColorRGBA secondColor\n\
\n\
\n\
duration colorChangeDuration\n\
\n\
\n\
bool reverseFade\n\
\n\
\n\
duration effectDuration\n\
\n\
\n\
uint8 priority\n\
\n\
\n\
================================================================================\n\
MSG: pal_device_msgs/LedGroup\n\
byte LEFT_EAR = 1\n\
byte RIGHT_EAR = 2\n\
\n\
#OR-mask of the selected leds\n\
uint32 ledMask\n\
\n\
================================================================================\n\
MSG: std_msgs/ColorRGBA\n\
float32 r\n\
float32 g\n\
float32 b\n\
float32 a\n\
";
  }

  static const char* value(const ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.leds);
      stream.next(m.firstColor);
      stream.next(m.secondColor);
      stream.next(m.colorChangeDuration);
      stream.next(m.reverseFade);
      stream.next(m.effectDuration);
      stream.next(m.priority);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct TimedFadeEffectRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_device_msgs::TimedFadeEffectRequest_<ContainerAllocator>& v)
  {
    s << indent << "leds: ";
    s << std::endl;
    Printer< ::pal_device_msgs::LedGroup_<ContainerAllocator> >::stream(s, indent + "  ", v.leds);
    s << indent << "firstColor: ";
    s << std::endl;
    Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "  ", v.firstColor);
    s << indent << "secondColor: ";
    s << std::endl;
    Printer< ::std_msgs::ColorRGBA_<ContainerAllocator> >::stream(s, indent + "  ", v.secondColor);
    s << indent << "colorChangeDuration: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.colorChangeDuration);
    s << indent << "reverseFade: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reverseFade);
    s << indent << "effectDuration: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.effectDuration);
    s << indent << "priority: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.priority);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_DEVICE_MSGS_MESSAGE_TIMEDFADEEFFECTREQUEST_H
