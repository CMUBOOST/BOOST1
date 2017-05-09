//This node operates the gripper, which is actuated by a Maxon EC16 motor. This code communicates with the motor controller, which is an EPOS 24/2.

#include <ros/ros.h>
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
int   RearHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, std_msgs::Int32* msg);
int   ForwardHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, std_msgs::Int16* msg);
int   Actuate(unsigned int* p_pErrorCode, std_msgs::Int16* stalk_presence_msg, std_msgs::Int32* stalk_dia_msg);



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




int RearHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, std_msgs::Int32* stalk_dia_msg)
{
    int lResult = MMC_SUCCESS;
    int lHomingAttained = 0; //Homing status zero until homed
    int lHomingError = 0;
    int HomingAccel = 20000; //This is a pretty high accel/decel
    int SpeedSwitch = 5000; //Speed during search for switch
    int SpeedIndex = 7000; //Speed during search for index (not used)
    int long HomeOffset = 0; //Offset after homing
    int CurrentThresh = 50; //Current threshold in mA
    int long HomePosition = 0;
    int HomingTimeout = 3000;
    int pPositionIs;
    int stalk_dia_mm;

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

        if(VCS_FindHome(p_DeviceHandle, p_usNodeId, 18, &p_rlErrorCode) == 0)
        {
            LogError("VCS_FindHome", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }

        VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, HomingTimeout, &p_rlErrorCode);
        VCS_GetPositionIs(p_DeviceHandle, p_usNodeId, &pPositionIs, &p_rlErrorCode);
        ROS_INFO_STREAM("Encoder Position3 is:" << pPositionIs);
        stalk_dia_mm = ((91644 - pPositionIs)/1382)+22;  //91644 is the average position when there is no stalk. 1382 is the number of encoder ticks per mm. 22mm is the smallest stalk that can be detected (ie the measurement offset)
        //stalk_dia_msg->data = pPositionIs;
        stalk_dia_msg->data = stalk_dia_mm;

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

    ROS_INFO_STREAM("Stalk Diameter is:" << stalk_dia_mm);

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




int ForwardHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode, std_msgs::Int16* stalk_presence_msg)
{
    int lResult = MMC_SUCCESS;
    int lHomingAttained = 0; //Homing status zero until homed
    int lHomingError = 0;
    int HomingAccel = 20000; //This is a pretty high accel/decel
    int SpeedSwitch = 5000; //Speed during search for switch
    int SpeedIndex = 7000; //Speed during search for index (not used)
    int long HomeOffset = 0; //Offset after homing
    int CurrentThresh = 50; //Current threshold in mA
    int long HomePosition = 0;
    int HomingTimeout = 3500;  //3.5 seconds before a timeout
    int postPenetrateDelay = 100000; //delay in microseconds after full penetration

    ROS_INFO_STREAM("set forward homing mode, node = " << p_usNodeId);

    ROS_INFO_STREAM("activating homing mode");

    if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, HomingAccel, SpeedSwitch, SpeedIndex, HomeOffset, CurrentThresh, HomePosition, &p_rlErrorCode);
        if(VCS_FindHome(p_DeviceHandle, p_usNodeId, 17, &p_rlErrorCode) == 0)
        {
            LogError("VCS_FindHome", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }

        if(VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, HomingTimeout, &p_rlErrorCode) == 0)
        {
            ROS_INFO_STREAM("No Stalk!!");

            stalk_presence_msg->data = 0;

            if((lResult = Prepare(&p_rlErrorCode))!=MMC_SUCCESS)
            {
                LogError("Prepare", lResult, p_rlErrorCode);
                return lResult;
            }
            else
                ROS_INFO_STREAM("Completed Clearing Error");

        }
        else
        {
            VCS_DefinePosition(p_DeviceHandle, p_usNodeId, HomePosition, &p_rlErrorCode);
            ROS_INFO_STREAM("Front set to zero: " << HomePosition);
            stalk_presence_msg->data = 1;
            usleep(postPenetrateDelay);
        }
    }



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




int Actuate(unsigned int* p_pErrorCode, std_msgs::Int16* stalk_presence_msg, std_msgs::Int32* stalk_dia_msg)
{

	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;

    lResult = ForwardHoming(g_pKeyHandle, g_usNodeId, lErrorCode, stalk_presence_msg);

    ROS_INFO_STREAM("lResult is:" << lResult);

    if(lResult != MMC_SUCCESS)
    {
        LogError("ForwardHoming", lResult, lErrorCode);
    }

    else
    {
        lResult = RearHoming(g_pKeyHandle, g_usNodeId, lErrorCode, stalk_dia_msg);

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
	}
	return lResult;
}



int main(int argc, char** argv)
{

    ros::init(argc, argv, "gripper_node4");

    ros::NodeHandle nh;

    ROS_INFO_STREAM("About to poke some plants");

    ros::Publisher stalk_presence_pub=nh.advertise<std_msgs::Int16>("stalk_presence", 1000);
    ros::Publisher stalk_dia_pub=nh.advertise<std_msgs::Int32>("stalk_dia", 1000);

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

    std_msgs::Int16 stalk_presence_msg;
    std_msgs::Int32 stalk_dia_msg;

    //***********************************
    //      CALL THE SERVICE HERE
    //***********************************
    /*
    ros::ServiceClient client = nh.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
    beginner_tutorials::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if (client.call(srv))
    {
      ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
    }
    */

    //**********************************
    //   ACTUATE GRIPPER AND PUBLISH
    //**********************************

    if((lResult = Actuate(&ulErrorCode, &stalk_presence_msg, &stalk_dia_msg))!=MMC_SUCCESS)
	{
        LogError("Actuate", lResult, ulErrorCode);
		return lResult;
	}

    stalk_presence_pub.publish(stalk_presence_msg);
    ros::spinOnce();
    stalk_dia_pub.publish(stalk_dia_msg);

	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}

	return lResult;
}
