// Generated by gencpp from file pal_interaction_msgs/recognizerServiceRequest.msg
// DO NOT EDIT!


#ifndef PAL_INTERACTION_MSGS_MESSAGE_RECOGNIZERSERVICEREQUEST_H
#define PAL_INTERACTION_MSGS_MESSAGE_RECOGNIZERSERVICEREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pal_interaction_msgs/asrupdate.h>

namespace pal_interaction_msgs
{
template <class ContainerAllocator>
struct recognizerServiceRequest_
{
  typedef recognizerServiceRequest_<ContainerAllocator> Type;

  recognizerServiceRequest_()
    : asrupdate()  {
    }
  recognizerServiceRequest_(const ContainerAllocator& _alloc)
    : asrupdate(_alloc)  {
    }



   typedef  ::pal_interaction_msgs::asrupdate_<ContainerAllocator>  _asrupdate_type;
  _asrupdate_type asrupdate;




  typedef boost::shared_ptr< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct recognizerServiceRequest_

typedef ::pal_interaction_msgs::recognizerServiceRequest_<std::allocator<void> > recognizerServiceRequest;

typedef boost::shared_ptr< ::pal_interaction_msgs::recognizerServiceRequest > recognizerServiceRequestPtr;
typedef boost::shared_ptr< ::pal_interaction_msgs::recognizerServiceRequest const> recognizerServiceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace pal_interaction_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'pal_interaction_msgs': ['/root/tiago_public_ws/src/pal_msgs/pal_interaction_msgs/msg', '/root/tiago_public_ws/devel/share/pal_interaction_msgs/msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cfe4314795c9fe304282fdbecf5be6ec";
  }

  static const char* value(const ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xcfe4314795c9fe30ULL;
  static const uint64_t static_value2 = 0x4282fdbecf5be6ecULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_interaction_msgs/recognizerServiceRequest";
  }

  static const char* value(const ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_interaction_msgs/asrupdate asrupdate\n\
\n\
================================================================================\n\
MSG: pal_interaction_msgs/asrupdate\n\
# V.1.0 of the ASR ROS API for the servie.\n\
# just has the language and grammar to enable, the acoustic models to be used,\n\
# and whether we want to activate or deactivate the recognizer.\n\
string language\n\
string enable_grammar\n\
string disable_grammar\n\
string acousticenv\n\
bool active\n\
";
  }

  static const char* value(const ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.asrupdate);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct recognizerServiceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_interaction_msgs::recognizerServiceRequest_<ContainerAllocator>& v)
  {
    s << indent << "asrupdate: ";
    s << std::endl;
    Printer< ::pal_interaction_msgs::asrupdate_<ContainerAllocator> >::stream(s, indent + "  ", v.asrupdate);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_INTERACTION_MSGS_MESSAGE_RECOGNIZERSERVICEREQUEST_H
