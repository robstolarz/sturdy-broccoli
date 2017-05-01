// Generated by gencpp from file pal_interaction_msgs/ASRFileResult.msg
// DO NOT EDIT!


#ifndef PAL_INTERACTION_MSGS_MESSAGE_ASRFILERESULT_H
#define PAL_INTERACTION_MSGS_MESSAGE_ASRFILERESULT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <pal_interaction_msgs/asrresult.h>

namespace pal_interaction_msgs
{
template <class ContainerAllocator>
struct ASRFileResult_
{
  typedef ASRFileResult_<ContainerAllocator> Type;

  ASRFileResult_()
    : file()
    , msg()
    , recognised_utterances()  {
    }
  ASRFileResult_(const ContainerAllocator& _alloc)
    : file(_alloc)
    , msg(_alloc)
    , recognised_utterances(_alloc)  {
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _file_type;
  _file_type file;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _msg_type;
  _msg_type msg;

   typedef std::vector< ::pal_interaction_msgs::asrresult_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::pal_interaction_msgs::asrresult_<ContainerAllocator> >::other >  _recognised_utterances_type;
  _recognised_utterances_type recognised_utterances;




  typedef boost::shared_ptr< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> const> ConstPtr;

}; // struct ASRFileResult_

typedef ::pal_interaction_msgs::ASRFileResult_<std::allocator<void> > ASRFileResult;

typedef boost::shared_ptr< ::pal_interaction_msgs::ASRFileResult > ASRFileResultPtr;
typedef boost::shared_ptr< ::pal_interaction_msgs::ASRFileResult const> ASRFileResultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f039521e38adb20a017037121dedb1d4";
  }

  static const char* value(const ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf039521e38adb20aULL;
  static const uint64_t static_value2 = 0x017037121dedb1d4ULL;
};

template<class ContainerAllocator>
struct DataType< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "pal_interaction_msgs/ASRFileResult";
  }

  static const char* value(const ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
##result definition\n\
# same path as specified in goal variable file\n\
string file\n\
# error/warning messages \n\
string msg\n\
# vector of results\n\
asrresult[] recognised_utterances\n\
\n\
================================================================================\n\
MSG: pal_interaction_msgs/asrresult\n\
## Message that containes the recognized utterance.\n\
## Confidence values\n\
int8 CONFIDENCE_UNDEFINED = -1\n\
int8 CONFIDENCE_POOR = 1\n\
int8 CONFIDENCE_LOW  = 2\n\
int8 CONFIDENCE_GOOD = 3\n\
int8 CONFIDENCE_MAX  = 4\n\
\n\
# ASR result messages used by RosRecognizerServer\n\
\n\
# text recognized\n\
string text\n\
\n\
# confidence with values from POOR to MAX\n\
int8 confidence\n\
\n\
# start and end of the recognizer uterance.\n\
time start\n\
time end\n\
\n\
# list of recognized tags\n\
# key value pairs of strings extracted from the text\n\
# given the action tags placed in the grammar.\n\
actiontag[] tags\n\
\n\
================================================================================\n\
MSG: pal_interaction_msgs/actiontag\n\
# Action tag contaings the key/value information genertated by parsing the recognised text with a JSGF grammar \n\
\n\
string key\n\
string value\n\
";
  }

  static const char* value(const ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.file);
      stream.next(m.msg);
      stream.next(m.recognised_utterances);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ASRFileResult_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::pal_interaction_msgs::ASRFileResult_<ContainerAllocator>& v)
  {
    s << indent << "file: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.file);
    s << indent << "msg: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.msg);
    s << indent << "recognised_utterances[]" << std::endl;
    for (size_t i = 0; i < v.recognised_utterances.size(); ++i)
    {
      s << indent << "  recognised_utterances[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::pal_interaction_msgs::asrresult_<ContainerAllocator> >::stream(s, indent + "    ", v.recognised_utterances[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PAL_INTERACTION_MSGS_MESSAGE_ASRFILERESULT_H
