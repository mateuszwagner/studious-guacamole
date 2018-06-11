// Generated by gencpp from file marker_publisher/Marker.msg
// DO NOT EDIT!


#ifndef MARKER_PUBLISHER_MESSAGE_MARKER_H
#define MARKER_PUBLISHER_MESSAGE_MARKER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseWithCovariance.h>

namespace marker_publisher
{
template <class ContainerAllocator>
struct Marker_
{
  typedef Marker_<ContainerAllocator> Type;

  Marker_()
    : idx(0)
    , pose()  {
    }
  Marker_(const ContainerAllocator& _alloc)
    : idx(0)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef uint32_t _idx_type;
  _idx_type idx;

   typedef  ::geometry_msgs::PoseWithCovariance_<ContainerAllocator>  _pose_type;
  _pose_type pose;




  typedef boost::shared_ptr< ::marker_publisher::Marker_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::marker_publisher::Marker_<ContainerAllocator> const> ConstPtr;

}; // struct Marker_

typedef ::marker_publisher::Marker_<std::allocator<void> > Marker;

typedef boost::shared_ptr< ::marker_publisher::Marker > MarkerPtr;
typedef boost::shared_ptr< ::marker_publisher::Marker const> MarkerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::marker_publisher::Marker_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::marker_publisher::Marker_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace marker_publisher

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'marker_publisher': ['/home/tatulo/Documents/studious-guacamole/src/marker_publisher-master/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::marker_publisher::Marker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::marker_publisher::Marker_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::marker_publisher::Marker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::marker_publisher::Marker_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::marker_publisher::Marker_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::marker_publisher::Marker_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::marker_publisher::Marker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dbd1a350bd65d90cf1c2af9ba7d01fe6";
  }

  static const char* value(const ::marker_publisher::Marker_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdbd1a350bd65d90cULL;
  static const uint64_t static_value2 = 0xf1c2af9ba7d01fe6ULL;
};

template<class ContainerAllocator>
struct DataType< ::marker_publisher::Marker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "marker_publisher/Marker";
  }

  static const char* value(const ::marker_publisher::Marker_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::marker_publisher::Marker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 idx\n\
geometry_msgs/PoseWithCovariance pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseWithCovariance\n\
# This represents a pose in free space with uncertainty.\n\
\n\
Pose pose\n\
\n\
# Row-major representation of the 6x6 covariance matrix\n\
# The orientation parameters use a fixed-axis representation.\n\
# In order, the parameters are:\n\
# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)\n\
float64[36] covariance\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::marker_publisher::Marker_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::marker_publisher::Marker_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.idx);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Marker_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::marker_publisher::Marker_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::marker_publisher::Marker_<ContainerAllocator>& v)
  {
    s << indent << "idx: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.idx);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseWithCovariance_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MARKER_PUBLISHER_MESSAGE_MARKER_H