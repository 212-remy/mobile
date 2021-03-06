// Generated by gencpp from file me212bot/WheelCmdVel.msg
// DO NOT EDIT!


#ifndef ME212BOT_MESSAGE_WHEELCMDVEL_H
#define ME212BOT_MESSAGE_WHEELCMDVEL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace me212bot
{
template <class ContainerAllocator>
struct WheelCmdVel_
{
  typedef WheelCmdVel_<ContainerAllocator> Type;

  WheelCmdVel_()
    : desiredWV_R(0.0)
    , desiredWV_L(0.0)  {
    }
  WheelCmdVel_(const ContainerAllocator& _alloc)
    : desiredWV_R(0.0)
    , desiredWV_L(0.0)  {
  (void)_alloc;
    }



   typedef float _desiredWV_R_type;
  _desiredWV_R_type desiredWV_R;

   typedef float _desiredWV_L_type;
  _desiredWV_L_type desiredWV_L;




  typedef boost::shared_ptr< ::me212bot::WheelCmdVel_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::me212bot::WheelCmdVel_<ContainerAllocator> const> ConstPtr;

}; // struct WheelCmdVel_

typedef ::me212bot::WheelCmdVel_<std::allocator<void> > WheelCmdVel;

typedef boost::shared_ptr< ::me212bot::WheelCmdVel > WheelCmdVelPtr;
typedef boost::shared_ptr< ::me212bot::WheelCmdVel const> WheelCmdVelConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::me212bot::WheelCmdVel_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::me212bot::WheelCmdVel_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace me212bot

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'me212bot': ['/home/robot/mobile/catkin_ws/src/me212bot/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::me212bot::WheelCmdVel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::me212bot::WheelCmdVel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::me212bot::WheelCmdVel_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::me212bot::WheelCmdVel_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::me212bot::WheelCmdVel_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::me212bot::WheelCmdVel_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::me212bot::WheelCmdVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "428fbbfd1f38717ca7baa73045b4efaa";
  }

  static const char* value(const ::me212bot::WheelCmdVel_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x428fbbfd1f38717cULL;
  static const uint64_t static_value2 = 0xa7baa73045b4efaaULL;
};

template<class ContainerAllocator>
struct DataType< ::me212bot::WheelCmdVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "me212bot/WheelCmdVel";
  }

  static const char* value(const ::me212bot::WheelCmdVel_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::me212bot::WheelCmdVel_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 desiredWV_R\n\
float32 desiredWV_L\n\
";
  }

  static const char* value(const ::me212bot::WheelCmdVel_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::me212bot::WheelCmdVel_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.desiredWV_R);
      stream.next(m.desiredWV_L);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct WheelCmdVel_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::me212bot::WheelCmdVel_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::me212bot::WheelCmdVel_<ContainerAllocator>& v)
  {
    s << indent << "desiredWV_R: ";
    Printer<float>::stream(s, indent + "  ", v.desiredWV_R);
    s << indent << "desiredWV_L: ";
    Printer<float>::stream(s, indent + "  ", v.desiredWV_L);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ME212BOT_MESSAGE_WHEELCMDVEL_H
