//This node operates the gripper, which is actuated by a Maxon EC16 motor. This code communicates with the motor controller, which is an EPOS 24/2.

#include <ros/ros.h>
#include "gripper/GripperClose.h"
#include <string.h>
#include "Definitions.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include <string.h>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>
#include <sstream>
#include <rosbag/bag.h>
#include <unistd.h>


typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0;
unsigned short g_usNodeId = 1;
string g_deviceName;
string g_protocolStackName;
string g_interfaceName;
string g_portName;
int g_baudrate = 0;

const string g_programName = "HelloEposCmd";

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif

void  LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode);
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ForwardHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   Close(unsigned int* p_pErrorCode);



void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
    ROS_ERROR_STREAM(g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")");
}


void SeparatorLine()
{
	const int lineLength = 60;
	for(int i=0; i<lineLength; i++)
	{
		cout << "-";
	}
	cout << endl;
}

void PrintSettings()
{
    ROS_INFO_STREAM("default settings:");
    ROS_INFO_STREAM("node id             = " << g_usNodeId);
    ROS_INFO_STREAM("device name         = '" << g_deviceName);
    ROS_INFO_STREAM("protocol stack name = '" << g_protocolStackName);
    ROS_INFO_STREAM("interface name      = '" << g_interfaceName << "'");
    ROS_INFO_STREAM("port name           = '" << g_portName << "'");
    ROS_INFO_STREAM("baudrate            = " << g_baudrate);
}

void SetDefaultParameters()
{
	g_usNodeId = 1;
	g_deviceName = "EPOS2"; //EPOS
	g_protocolStackName = "MAXON SERIAL V2"; //MAXON_RS232
	g_interfaceName = "USB"; //RS232
	g_portName = "USB0"; // /dev/ttyS1
	g_baudrate = 1000000; //115200
}


int OpenDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	char* pDeviceName = new char[255];
	char* pProtocolStackName = new char[255];
	char* pInterfaceName = new char[255];
	char* pPortName = new char[255];

	strcpy(pDeviceName, g_deviceName.c_str());
	strcpy(pProtocolStackName, g_protocolStackName.c_str());
	strcpy(pInterfaceName, g_interfaceName.c_str());
	strcpy(pPortName, g_portName.c_str());

    ROS_INFO_STREAM("Open device...");

	g_pKeyHandle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

	if(g_pKeyHandle!=0 && *p_pErrorCode == 0)
	{
		unsigned int lBaudrate = 0;
		unsigned int lTimeout = 0;

		if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
		{
			if(VCS_SetProtocolStackSettings(g_pKeyHandle, g_baudrate, lTimeout, p_pErrorCode)!=0)
			{
				if(VCS_GetProtocolStackSettings(g_pKeyHandle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
				{
					if(g_baudrate==(int)lBaudrate)
					{
						lResult = MMC_SUCCESS;
					}
				}
			}
		}
	}
	else
	{
		g_pKeyHandle = 0;
	}

	delete []pDeviceName;
	delete []pProtocolStackName;
	delete []pInterfaceName;
	delete []pPortName;

	return lResult;
}



int CloseDevice(unsigned int* p_pErrorCode)
{
	int lResult = MMC_FAILED;

	*p_pErrorCode = 0;

    ROS_INFO_STREAM("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}



int Prepare(unsigned int* p_pErrorCode)
{
    int lResult = MMC_SUCCESS;
    BOOL oIsFault = 0;

    if(VCS_GetFaultState(g_pKeyHandle, g_usNodeId, &oIsFault, p_pErrorCode ) == 0)
    {
        LogError("VCS_GetFaultState", lResult, *p_pErrorCode);
        lResult = MMC_FAILED;
    }

    if(lResult==0)
    {
        if(oIsFault)
        {
            ROS_INFO_STREAM("clear fault, node = '" << g_usNodeId << "'");

            if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
            {
                LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }

        if(lResult==0)
        {
            BOOL oIsEnabled = 0;

            if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
            {
                LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }

            if(lResult==0)
            {
                if(!oIsEnabled)
                {
                    if(VCS_SetEnableState(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
                    {
                        LogError("VCS_SetEnableState", lResult, *p_pErrorCode);
                        lResult = MMC_FAILED;
                    }
                }
            }
        }
    }
    return lResult;
}



int ForwardHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    int long closePosition = -22000;
    int vel = 10000;
    int accel = 10000;
    int decel = 10000;


    ROS_INFO_STREAM("set gripper close mode, node = " << p_usNodeId);

    ROS_INFO_STREAM("activating gripper close");

    //if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        ROS_INFO_STREAM("ERROR!");
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }

    ROS_INFO("ABOUT TO CLOSE THE GRIPPER");
    VCS_SetPositionProfile(p_DeviceHandle, p_usNodeId, vel, accel, decel, &p_rlErrorCode);

    VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, closePosition, false, false, &p_rlErrorCode);

    ROS_INFO("GRIPPER CLOSED");

    return lResult;
}


int Close(unsigned int* p_pErrorCode)
{

	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

    lResult = ForwardHoming(g_pKeyHandle, g_usNodeId, lErrorCode);

    ROS_INFO_STREAM("lResult is:" << lResult);

    if(lResult != MMC_SUCCESS)
    {
        LogError("ForwardHoming", lResult, lErrorCode);
    }

	return lResult;
}



bool grip(gripper::GripperClose::Request  &req,
          gripper::GripperClose::Response &res)
{

    //ros::init(argc, argv, "gripper_node4");

    ros::NodeHandle nh;

    //ROS_INFO_STREAM("About to poke some plants");

    res.gripper_close_resp = req.gripper_close_req;  //this is a placeholder for now

    ROS_INFO_STREAM("Gripper actuation request: " << req.gripper_close_req);

	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

	SetDefaultParameters();

	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}

    if((lResult = Prepare(&ulErrorCode))!=MMC_SUCCESS)
	{
        LogError("Prepare", lResult, ulErrorCode);
		return lResult;
	}

    //**********************************
    //   ACTUATE GRIPPER
    //**********************************

    if((lResult = Close(&ulErrorCode))!=MMC_SUCCESS)
	{
        LogError("Close", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}

    //return lResult;

    ROS_INFO_STREAM("sending back response: " << res.gripper_close_resp);

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_close_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("gripper_close", grip);
  ROS_INFO("Ready to grip!");
  ros::spin();

  return 0;
}
