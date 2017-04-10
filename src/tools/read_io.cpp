#include <ros/ros.h>
#include "epos4_control/utils.h"
#include "epos4_control/Definitions.h"
#include "std_msgs/Int16.h"
#include <iostream>

#include "boost/foreach.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "io_reader");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int16>("/voltage/gui", 10);

    uint64_t serial_number;
    if(argc == 2)
    {
      if(!SerialNumberFromHex(argv[1], &serial_number))
      {
        std::cerr << "Expected a serial number" << std::endl;
        return 1;
      }
    }
    else
    {
      std::cerr << "Expected exactly one argument that is a serial number" << std::endl;
      return 1;
    }

    std::string error_string;
    unsigned int error_code = 0;

    std::cout << "Searching for USB EPOS4: 0x" << std::hex << serial_number << std::endl;

    std::string port_name;

    EposFactory epos_factory;

    std_msgs::Int16 v_read;

    ros::Rate ros_sleep(1);

    int reading = 0;


    NodeHandlePtr handle;
    if(handle = epos_factory.CreateNodeHandle("EPOS4", "MAXON SERIAL V2", "USB", serial_number, &error_code))
    {
        int x = 0;
        while(ros::ok())
        {
            u_int16_t analog_value;

            if(VCS_GetAnalogInput(handle->device_handle->ptr, handle->node_id, 1, &analog_value, &error_code))
            {
                v_read.data = analog_value;
                pub.publish(v_read);
            }
            else
            {
                if(GetErrorInfo(error_code, &error_string))
                    std::cerr << "Could not get Voltage Value: " << error_string << std::endl;
                else
                    std::cerr << "Could not get Voltage Value" << std::endl;
            }

            u_int16_t digital_inputs[10];

            for(int i = 0; i < 10; i++)
            {
                digital_inputs[i] = 50;
            }

            if(VCS_GetAllDigitalInputs(handle->device_handle->ptr, handle->node_id, digital_inputs, &error_code))
            {
                for(int i = 0; i < 10; i++)
                {
                    std::cout <<"[ " << reading << " ]" << " Pin[ " << i << " ] = " << std::dec << digital_inputs[i] << std::endl;
                }

                std::cout <<"NEXT" << std::endl;
            }
            else
            {
                if(GetErrorInfo(error_code, &error_string))
                    std::cerr << "Could not get input Value: " << error_string << std::endl;
                else
                    std::cerr << "Could not get input Value" << std::endl;
            }

            ros_sleep.sleep();
            reading++;
        }
    }
    else
    {
        if(GetErrorInfo(error_code, &error_string))
        {
          std::cerr << "Could not open device: " << error_string << std::endl;
        }
        else
        {
          std::cerr << "Could not open device" << std::endl;
        }
        return 1;
    }
}
