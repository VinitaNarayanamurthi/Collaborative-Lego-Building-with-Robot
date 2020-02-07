// Generated by gencpp from file baxter_maintenance_msgs/UpdateSources.msg
// DO NOT EDIT!


#ifndef BAXTER_MAINTENANCE_MSGS_MESSAGE_UPDATESOURCES_H
#define BAXTER_MAINTENANCE_MSGS_MESSAGE_UPDATESOURCES_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <baxter_maintenance_msgs/UpdateSource.h>

namespace baxter_maintenance_msgs
{
template <class ContainerAllocator>
struct UpdateSources_
{
  typedef UpdateSources_<ContainerAllocator> Type;

  UpdateSources_()
    : uuid()
    , sources()  {
    }
  UpdateSources_(const ContainerAllocator& _alloc)
    : uuid(_alloc)
    , sources(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _uuid_type;
  _uuid_type uuid;

   typedef std::vector< ::baxter_maintenance_msgs::UpdateSource_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::baxter_maintenance_msgs::UpdateSource_<ContainerAllocator> >::other >  _sources_type;
  _sources_type sources;





  typedef boost::shared_ptr< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> const> ConstPtr;

}; // struct UpdateSources_

typedef ::baxter_maintenance_msgs::UpdateSources_<std::allocator<void> > UpdateSources;

typedef boost::shared_ptr< ::baxter_maintenance_msgs::UpdateSources > UpdateSourcesPtr;
typedef boost::shared_ptr< ::baxter_maintenance_msgs::UpdateSources const> UpdateSourcesConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace baxter_maintenance_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'baxter_maintenance_msgs': ['/home/rashmi/baxter_ws/src/baxter_common/baxter_maintenance_msgs/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b3b428bf55e80e83d378830c33b3405b";
  }

  static const char* value(const ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb3b428bf55e80e83ULL;
  static const uint64_t static_value2 = 0xd378830c33b3405bULL;
};

template<class ContainerAllocator>
struct DataType< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >
{
  static const char* value()
  {
    return "baxter_maintenance_msgs/UpdateSources";
  }

  static const char* value(const ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string          uuid\n\
UpdateSource[]  sources\n\
\n\
================================================================================\n\
MSG: baxter_maintenance_msgs/UpdateSource\n\
string  devname\n\
string  filename\n\
string  version\n\
string  uuid\n\
";
  }

  static const char* value(const ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.uuid);
      stream.next(m.sources);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct UpdateSources_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::baxter_maintenance_msgs::UpdateSources_<ContainerAllocator>& v)
  {
    s << indent << "uuid: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.uuid);
    s << indent << "sources[]" << std::endl;
    for (size_t i = 0; i < v.sources.size(); ++i)
    {
      s << indent << "  sources[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::baxter_maintenance_msgs::UpdateSource_<ContainerAllocator> >::stream(s, indent + "    ", v.sources[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // BAXTER_MAINTENANCE_MSGS_MESSAGE_UPDATESOURCES_H
