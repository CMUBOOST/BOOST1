//This node operates the gripper, which is actuated by a Maxon EC16 motor. This code communicates with the motor controller, which is an EPOS 24/2.

#include <ros/ros.h>
#include "gripper/GripperOpen.h"
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
int   RearHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   Open(unsigned int* p_pErrorCode);



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




int RearHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    int lHomingAttained = 0; //Homing status zero until homed
    int lHomingError = 0;
    int HomingAccel = 20000; //This is a pretty high accel/decel
    int SpeedSwitch = 5000; //Speed during search for switch
    int SpeedIndex = 7000; //Speed during search for index (not used)
    int long HomeOffset = -9000; //Offset after homing  //I CHANGED THIS AS OF 9/21!!!
    int CurrentThresh = 800; //Current threshold in mA
    int long HomePosition = 0; //-8000;  //I CHANGED THIS AS OF 9/21!!!!
    int HomingTimeout = 3000;
    int pPositionIs;
    //int stalk_dia_mm;

    ROS_INFO_STREAM("set rear homing mode, node = " << p_usNodeId);

    ROS_INFO_STREAM("activating homing mode");


    //Activate the homing mode
    if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }

    //Set the homing parameter, perform homing, get the position of homing, and assign that to stalk_dia_msg->data, then clear the error.
    else
    {
        VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, HomingAccel, SpeedSwitch, SpeedIndex, HomeOffset, CurrentThresh, HomePosition, &p_rlErrorCode);

        if(VCS_FindHome(p_DeviceHandle, p_usNodeId, -3, &p_rlErrorCode) == 0)
        {
            LogError("VCS_FindHome", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }

        VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, HomingTimeout, &p_rlErrorCode);
        //VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &pPositionIs, &p_rlErrorCode);
        //ROS_INFO_STREAM("Encoder Position3 is:" << pPositionIs);
        //stalk_dia_mm = ((91644 - pPositionIs)/1382)+22;  //91644 is the average position when there is no stalk. 1382 is the number of encoder ticks per mm. 22mm is the smallest stalk that can be detected (ie the measurement offset)
        //stalk_dia_msg->data = pPositionIs;
        //stalk_dia_msg->data = stalk_dia_mm;

        if((lResult = Prepare(&p_rlErrorCode))!=MMC_SUCCESS)
        {
            LogError("Prepare", lResult, p_rlErrorCode);
            return lResult;
        }
        else
            ROS_INFO_STREAM("Completed Clearing Error");
            VCS_DefinePosition(p_DeviceHandle, p_usNodeId, HomePosition, &p_rlErrorCode);
            ROS_INFO_STREAM("Front set to zero: " << HomePosition);
        }

    //ROS_INFO_STREAM("Stalk Diameter is:" << stalk_dia_mm);

    if(lResult == MMC_SUCCESS)
    {
        ROS_INFO_STREAM("halt position movement");

        if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
        {
            LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }
    }

    return lResult;
}


int Open(unsigned int* p_pErrorCode)
{

	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

    lResult = RearHoming(g_pKeyHandle, g_usNodeId, lErrorCode);

    if(lResult != MMC_SUCCESS)
    {
        LogError("RearHoming", lResult, lErrorCode);
    }
    else
    {
        if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
        {
            LogError("VCS_SetDisableState", lResult, lErrorCode);
            lResult = MMC_FAILED;
        }
    }

    //lResult = Offset(g_pKeyHandle, g_usNodeId, lErrorCode);

	return lResult;
}



bool grip(gripper::GripperOpen::Request  &req,
          gripper::GripperOpen::Response &res)
{

    //ros::init(argc, argv, "gripper_node4");

    ros::NodeHandle nh;

    //ROS_INFO_STREAM("About to poke some plants");

    res.gripper_open_resp = req.gripper_open_req;  //this is a placeholder for now

    ROS_INFO_STREAM("Gripper actuation request: " << req.gripper_open_req);

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

    if((lResult = Open(&ulErrorCode))!=MMC_SUCCESS)
	{
        LogError("Open", lResult, ulErrorCode);
		return lResult;
	}

    if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
    {
        LogError("CloseDevice", lResult, ulErrorCode);
        return lResult;
    }

    //return lResult;

    ROS_INFO_STREAM("sending back response: " << res.gripper_open_resp);

    return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripper_open_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("gripper_open", grip);
  ROS_INFO("Ready to grip!");
  ros::spin();

  return 0;
}
