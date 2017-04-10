#include "epos4_control/utils.h"
#include <boost/foreach.hpp>
#include <sstream>
#include <ros/ros.h>
#define MAX_STRING_SIZE 1000

bool SerialNumberFromHex(const std::string& str, uint64_t* serial_number) {
    std::stringstream ss;
    ss << std::hex << str;
    ss >> *serial_number;
    return true;
}

int GetErrorInfo(unsigned int error_code, std::string* error_string)
{
    char buffer[MAX_STRING_SIZE];
    int result;
    if(result = VCS_GetErrorInfo(error_code, buffer, MAX_STRING_SIZE))
    {
        *error_string = buffer;
    }
    return result;
}


int GetDeviceNameList(std::vector<std::string>* device_names, unsigned int* error_code) {
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetDeviceNameSelection(true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if(!result)
    return result;
  device_names->push_back(buffer);

  while(!end_of_selection) {
    result = VCS_GetDeviceNameSelection(false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
      return result;
    device_names->push_back(buffer);
  }

  return 1;
}

int GetProtocolStackNameList(const std::string device_name, std::vector<std::string>* protocol_stack_names, unsigned int* error_code) {
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetProtocolStackNameSelection((char*)device_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if(!result)
    return result;
  protocol_stack_names->push_back(buffer);

  while(!end_of_selection) {
    result = VCS_GetProtocolStackNameSelection((char*)device_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
      return result;
    protocol_stack_names->push_back(buffer);
  }

  return 1;
}


int GetInterfaceNameList(const std::string device_name, const std::string protocol_stack_name, std::vector<std::string>* interface_names, unsigned int* error_code) {
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetInterfaceNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if(!result)
    return result;
  interface_names->push_back(buffer);

  while(!end_of_selection) {
    result = VCS_GetInterfaceNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
      return result;
    interface_names->push_back(buffer);
  }

  return 1;
}

int GetPortNameList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name, std::vector<std::string>* port_names, unsigned int* error_code) {
  char buffer[MAX_STRING_SIZE];
  int end_of_selection; //BOOL
  int result;

  result = VCS_GetPortNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), true, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
  if(!result)
    return result;
  port_names->push_back(buffer);

  while(!end_of_selection) {
    result = VCS_GetPortNameSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), false, buffer, MAX_STRING_SIZE, &end_of_selection, error_code);
    if(!result)
      return result;
    port_names->push_back(buffer);
  }

  return 1;
}

int GetBaudrateList(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
            const std::string port_name, std::vector<unsigned int>* baudrates, unsigned int* error_code)
{
    unsigned int baudrate;
    int end_of_selection; //BOOL
    int result;

    result = VCS_GetBaudrateSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), (char*)port_name.c_str(), true, &baudrate, &end_of_selection, error_code);
    if(!result)
        return result;
    baudrates->push_back(baudrate);

    while(!end_of_selection)
    {
        result = VCS_GetBaudrateSelection((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), (char*)port_name.c_str(), false, &baudrate, &end_of_selection, error_code);
        if(!result)
            return result;
        baudrates->push_back(baudrate);
    }

    return 1;
}





EposFactory::EposFactory() {
}

DeviceHandlePtr EposFactory::CreateDeviceHandle(const std::string device_name,
						const std::string protocol_stack_name,
						const std::string interface_name,
						const std::string port_name,
                        unsigned int* error_code)
{
    const std::string key = device_name + '/' + protocol_stack_name + '/' + interface_name + '/' + port_name;

    DeviceHandlePtr handle;
    if(!(handle = existing_handles[key].lock()))
    { // Handle exists
        ROS_INFO_STREAM("CreateDeviceHandle: Handle exists");
        void* raw_handle = VCS_OpenDevice((char*)device_name.c_str(), (char*)protocol_stack_name.c_str(), (char*)interface_name.c_str(), (char*)port_name.c_str(), error_code);

        if(!raw_handle) // failed to open device
        {
            std::cout << "OPEN DEVICE FAILED: 0x" << std::hex << *error_code << std::endl;
            return DeviceHandlePtr();
        }
        handle = DeviceHandlePtr(new DeviceHandle(raw_handle));
        existing_handles[key] = handle;
    }

    return handle;
}

NodeHandlePtr EposFactory::CreateNodeHandle(const std::string device_name,
					    const std::string protocol_stack_name,
					    const std::string interface_name,
					    const uint64_t serial_number,
                        unsigned int* error_code)
{
    std::vector<EnumeratedNode> nodes;
    EnumerateNodes(device_name, protocol_stack_name, interface_name, &nodes, error_code);
    BOOST_FOREACH(const EnumeratedNode& node, nodes)
    {
        ROS_INFO_STREAM("CreateNodeHandle: node.serial_number=" << node.serial_number);
        ROS_INFO_STREAM("CreateNodeHandle: serial_number=" << serial_number);

        if(node.serial_number == serial_number)
        {
          return CreateNodeHandle(node, error_code);
        }
    }
    return NodeHandlePtr();
}

NodeHandlePtr EposFactory::CreateNodeHandle(const EnumeratedNode& node,
                        unsigned int* error_code)
{
    ROS_INFO_STREAM("CreateNodeHandle: 2");
    DeviceHandlePtr device_handle = CreateDeviceHandle(node.device_name, node.protocol_stack_name, node.interface_name, node.port_name, error_code);
    if(!device_handle)
        return NodeHandlePtr();
    return NodeHandlePtr(new NodeHandle(device_handle, node.node_id));
}



int EposFactory::EnumerateNodes(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                const std::string port_name, std::vector<EnumeratedNode>* nodes, unsigned int* error_code)
{
    ROS_INFO_STREAM("EnumerateNodes: device_name="<<device_name<<"protocol_stack_name="<<protocol_stack_name<<"interface_name="<<interface_name<< "port_name="<<port_name);
    DeviceHandlePtr handle;
    if(!(handle = CreateDeviceHandle(device_name, protocol_stack_name, interface_name, port_name, error_code)))
    {
        return 0;
    }

    ROS_INFO_STREAM("EnumerateNodes: start");
    for(unsigned short i = 1; i < 127; ++i)
    {
        EnumeratedNode node;
        node.device_name = device_name;
        node.protocol_stack_name = protocol_stack_name;
        node.interface_name = interface_name;
        node.port_name = port_name;
        node.node_id = i;
        if(!VCS_GetVersion(handle->ptr, i, &node.hardware_version, &node.software_version, &node.application_number, &node.application_version, error_code))
        {
            return 1;
        }
        ROS_INFO_STREAM("EnumerateNodes: VCS_GetVersion");
        unsigned int bytes_read1;
        int bitrate = -1;
        if(!VCS_GetObject(handle->ptr, i, 0x2001, 0x00, &bitrate, 4, &bytes_read1, error_code))
        {
            ROS_INFO_STREAM("EnumerateNodes: Fail to read CAN Bit Rate " << bytes_read1);
        }
        else
            ROS_INFO_STREAM("EnumerateNodes: bitrate=" << bitrate);
        unsigned int bytes_written;
        int value = 1;
        if(!VCS_SetObject(handle->ptr, i, 0x2001, 0x00, &value, 4, &bytes_written, error_code))
        {
            ROS_INFO_STREAM("EnumerateNodes: Fail to set CAN Bit Rate " << bytes_written);
        }
        if(!VCS_GetObject(handle->ptr, i, 0x2001, 0x00, &bitrate, 4, &bytes_read1, error_code))
        {
            ROS_INFO_STREAM("EnumerateNodes: Fail to read CAN Bit Rate " << bytes_read1);
        }
        else
            ROS_INFO_STREAM("EnumerateNodes: bitrate=" << bitrate);

        if(!VCS_Store(handle->ptr, i, error_code))
        {
            ROS_INFO_STREAM("EnumerateNodes: Fail to store settings, error_code=" << error_code);
        }

        unsigned int bytes_read;
        if(!VCS_GetObject(handle->ptr, i, 0x2100, 0x01, &node.serial_number, 8, &bytes_read, error_code))
        {
            node.serial_number = 0;
        }
        ROS_INFO_STREAM("EnumerateNodes: VCS_GetObject " << bytes_read << ", error_code=" << error_code);
        nodes->push_back(node);
    }
    return 1;
}


int EposFactory::EnumerateNodes(const std::string device_name, const std::string protocol_stack_name, const std::string interface_name,
                std::vector<EnumeratedNode>* nodes, unsigned int* error_code)
{
    std::vector<std::string> port_names;
    if(GetPortNameList(device_name, protocol_stack_name, interface_name, &port_names, error_code))
    {
        BOOST_FOREACH(const std::string& port_name, port_names)
        {
            if(!EnumerateNodes(device_name, protocol_stack_name, interface_name, port_name, nodes, error_code))
            {
                return 0;
            }
        }
        return 1;
    }
    else
        return 0;
}
