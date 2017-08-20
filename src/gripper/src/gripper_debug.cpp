//============================================================================
// Name        : HelloEposCmd.cpp
// Author      : Dawid Sienkiewicz
// Version     :
// Copyright   : maxon motor ag 2014
// Description : Hello Epos in C++, Ansi-style
//============================================================================

#include <ros/ros.h>
#include <string.h>
#include <iostream>
//#include "home/cmuboost/test/src/gripper/include/Definitions.h"
#include "../include/Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <list>
#include <math.h>

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
void  LogInfo(string message);
void  PrintUsage();
void  PrintHeader();
void  PrintSettings();
int   OpenDevice(unsigned int* p_pErrorCode);
int   CloseDevice(unsigned int* p_pErrorCode);
void  SetDefaultParameters();
int   ParseArguments(int argc, char** argv);
int   DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   DemoRearHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   DemoForwardHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode);
int   Demo(unsigned int* p_pErrorCode);

void PrintUsage()
{
	cout << "Usage: HelloEposCmd -h -n 1 -d deviceName -s protocolStackName -i interfaceName -p portName -b baudrate" << endl;
}

void LogError(string functionName, int p_lResult, unsigned int p_ulErrorCode)
{
	cerr << g_programName << ": " << functionName << " failed (result=" << p_lResult << ", errorCode=0x" << std::hex << p_ulErrorCode << ")"<< endl;
}

void LogInfo(string message)
{
	cout << message << endl;
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
	stringstream msg;

	msg << "default settings:" << endl;
	msg << "node id             = " << g_usNodeId << endl;
	msg << "device name         = '" << g_deviceName << "'" << endl;
    msg << "protocol stack name = '" << g_protocolStackName << "'" << endl;
	msg << "interface name      = '" << g_interfaceName << "'" << endl;
	msg << "port name           = '" << g_portName << "'"<< endl;
	msg << "baudrate            = " << g_baudrate;

	LogInfo(msg.str());

	SeparatorLine();
}

void SetDefaultParameters()
{
	//USB
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

	LogInfo("Open device...");

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

	LogInfo("Close device");

	if(VCS_CloseDevice(g_pKeyHandle, p_pErrorCode)!=0 && *p_pErrorCode == 0)
	{
		lResult = MMC_SUCCESS;
	}

	return lResult;
}

int ParseArguments(int argc, char** argv)
{
	int lOption;
	int lResult = MMC_SUCCESS;

	// Shut GetOpt error messages down (return '?'):
	opterr = 0;
	// Retrieve the options:
	while ( (lOption = getopt(argc, argv, ":hd:s:i:p:b:n:")) != -1 )
	{
		switch ( lOption ) {
			case 'h':
				PrintUsage();
				lResult = 1;
				break;
			case 'd':
				g_deviceName = optarg;
				break;
			case 's':
				g_protocolStackName = optarg;
				break;
			case 'i':
				g_interfaceName = optarg;
				break;
			case 'p':
				g_portName = optarg;
				break;
			case 'b':
				g_baudrate = atoi(optarg);
				break;
			case 'n':
				g_usNodeId = (unsigned short)atoi(optarg);
				break;
			case '?':  // unknown option...
				stringstream msg;
				msg << "Unknown option: '" << char(optopt) << "'!";
				LogInfo(msg.str());
				PrintUsage();
				lResult = MMC_FAILED;
				break;
		}
	}

	return lResult;
}

int DemoProfilePositionMode(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
	int lResult = MMC_SUCCESS;
	stringstream msg;

	msg << "set profile position mode, node = " << p_usNodeId;
	LogInfo(msg.str());



    //VCS_FindHome(p_DeviceHandle, p_usNodeId, -3, &p_rlErrorCode);

	if(VCS_ActivateProfilePositionMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
	{
		LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
		lResult = MMC_FAILED;
	}
	else
	{
		list<long> positionList;

        positionList.push_back(-45000);

		for(list<long>::iterator it = positionList.begin(); it !=positionList.end(); it++)
		{
			long targetPosition = (*it);
			stringstream msg;
			msg << "move to position = " << targetPosition << ", node = " << p_usNodeId;
			LogInfo(msg.str());

			if(VCS_MoveToPosition(p_DeviceHandle, p_usNodeId, targetPosition, 0, 1, &p_rlErrorCode) == 0)
			{
				LogError("VCS_MoveToPosition", lResult, p_rlErrorCode);
				lResult = MMC_FAILED;
				break;
			}

            sleep(5);
		}


        if(lResult == MMC_SUCCESS)
        {
            LogInfo("halt position movement");

            if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
            {
                LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
                lResult = MMC_FAILED;
            }
        }

	}


	return lResult;
}


int PrepareDemo(unsigned int* p_pErrorCode)
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
            stringstream msg;
            msg << "clear fault, node = '" << g_usNodeId << "'";
            LogInfo(msg.str());

            if(VCS_ClearFault(g_pKeyHandle, g_usNodeId, p_pErrorCode) == 0)
            {
                LogError("VCS_ClearFault", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }
        }

        if(lResult==0)
        {
            BOOL oIsEnabled = 0;
            cout << "I'm in the lResult=0 statement!!" << endl;

            if(VCS_GetEnableState(g_pKeyHandle, g_usNodeId, &oIsEnabled, p_pErrorCode) == 0)
            {
                LogError("VCS_GetEnableState", lResult, *p_pErrorCode);
                lResult = MMC_FAILED;
            }

            if(lResult==0)
            {
                cout << "lResult == 0" << endl;
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

int DemoRearHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    int lHomingAttained = 0; //Homing status zero until homed
    int lHomingError = 0;
    int HomingAccel = 100000; //This is a pretty high accel/decel
    int SpeedSwitch = 10000; //Speed during search for switch
    int SpeedIndex = 7000; //Speed during search for index (not used)
    int long HomeOffset = 0; //Offset after homing
    int CurrentThresh = 1600; //Current threshold in mA
    int long HomePosition = 0;
    int HomingTimeout = 5000;
    stringstream msg;

    msg << "set rear homing mode, node = " << p_usNodeId;
    LogInfo(msg.str());

    cout << "activating homing mode" << endl;

    if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, HomingAccel, SpeedSwitch, SpeedIndex, HomeOffset, CurrentThresh, HomePosition, &p_rlErrorCode);
        if(VCS_FindHome(p_DeviceHandle, p_usNodeId, -3, &p_rlErrorCode) == 0)
        {
            LogError("VCS_FindHome", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }

        VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, HomingTimeout, &p_rlErrorCode);
        sleep(1);
    }



    if(lResult == MMC_SUCCESS)
    {
        LogInfo("halt position movement");

        if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
        {
            LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }
    }

    return lResult;
}


int DemoForwardHoming(HANDLE p_DeviceHandle, unsigned short p_usNodeId, unsigned int & p_rlErrorCode)
{
    int lResult = MMC_SUCCESS;
    int lHomingAttained = 0; //Homing status zero until homed
    int lHomingError = 0;
    int HomingAccel = 20000; //This is a pretty high accel/decel
    int SpeedSwitch = 10000; //Speed during search for switch
    int SpeedIndex = 7000; //Speed during search for index (not used)
    int long HomeOffset = 0; //Offset after homing
    int CurrentThresh = 50; //Current threshold in mA
    int long HomePosition = 0;
    int HomingTimeout = 5000;
    int postPenetrateDelay = 1; //delay in seconds after full penetration


    stringstream msg;

    msg << "set forward homing mode, node = " << p_usNodeId;
    LogInfo(msg.str());

    cout << "activating homing mode" << endl;

    if(VCS_ActivateHomingMode(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
    {
        LogError("VCS_ActivateProfilePositionMode", lResult, p_rlErrorCode);
        lResult = MMC_FAILED;
    }
    else
    {
        cout << "In the ActivateHomingMode else-statement" << endl;
        VCS_SetHomingParameter(p_DeviceHandle, p_usNodeId, HomingAccel, SpeedSwitch, SpeedIndex, HomeOffset, CurrentThresh, HomePosition, &p_rlErrorCode);
        cout << "Set homing parameters. Now " << endl;
        if(VCS_FindHome(p_DeviceHandle, p_usNodeId, 17, &p_rlErrorCode) == 0)
        {
            cout << "VCS_FINDHOME IS ZERO" <<endl;
            LogError("VCS_FindHome", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }

        if(VCS_WaitForHomingAttained(p_DeviceHandle, p_usNodeId, HomingTimeout, &p_rlErrorCode) == 0)
        {
            cout << "No Stalk!!" << endl;

            if((lResult = PrepareDemo(&p_rlErrorCode))!=MMC_SUCCESS)
            {
                cout << "PrepareDemo does not equal MMC_SUCCESS" << endl;
                LogError("PrepareDemo", lResult, p_rlErrorCode);
                return lResult;
            }
            else
                cout << "Completed Clearing Error" << endl;

            /*
            cout << "clearing fault" << endl;
            VCS_ClearFault(p_DeviceHandle, p_usNodeId, &p_rlErrorCode);
            cout << "setting enable state" << endl;
            VCS_SetEnableState(p_DeviceHandle, p_usNodeId, &p_rlErrorCode);
            */

        }
        else
            {
            cout << "found a stalk!!" << endl;
            sleep(postPenetrateDelay);
            }
    }



    if(lResult == MMC_SUCCESS)
    {
        LogInfo("halt position movement");

        if(VCS_HaltPositionMovement(p_DeviceHandle, p_usNodeId, &p_rlErrorCode) == 0)
        {
            LogError("VCS_HaltPositionMovement", lResult, p_rlErrorCode);
            lResult = MMC_FAILED;
        }
    }

    return lResult;
}




int Demo(unsigned int* p_pErrorCode)
{
    stringstream msg;

	int lResult = MMC_SUCCESS;
	unsigned int lErrorCode = 0;


    /*
    lResult = DemoProfilePositionMode(g_pKeyHandle, g_usNodeId, lErrorCode);

    if(lResult != MMC_SUCCESS)
    {
        LogError("DemoProfilePositionMode", lResult, lErrorCode);
    }
    else
    {
        if(VCS_SetDisableState(g_pKeyHandle, g_usNodeId, &lErrorCode) == 0)
        {
            LogError("VCS_SetDisableState", lResult, lErrorCode);
            lResult = MMC_FAILED;
        }
    }

    */

    lResult = DemoForwardHoming(g_pKeyHandle, g_usNodeId, lErrorCode);

    if(lResult != MMC_SUCCESS)
    {
        LogError("DemoForwardHoming", lResult, lErrorCode);
    }
    /*
    else
    {
        lResult = DemoRearHoming(g_pKeyHandle, g_usNodeId, lErrorCode);

		if(lResult != MMC_SUCCESS)
		{
            LogError("DemoRearHoming", lResult, lErrorCode);
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
    */
	return lResult;
}



int main(int argc, char** argv)
{
	int lResult = MMC_FAILED;
	unsigned int ulErrorCode = 0;

    //PrintHeader();

	SetDefaultParameters();

	if((lResult = ParseArguments(argc, argv))!=MMC_SUCCESS)
	{
		return lResult;
	}

	PrintSettings();

	if((lResult = OpenDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("OpenDevice", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = PrepareDemo(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("PrepareDemo", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = Demo(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("Demo", lResult, ulErrorCode);
		return lResult;
	}

	if((lResult = CloseDevice(&ulErrorCode))!=MMC_SUCCESS)
	{
		LogError("CloseDevice", lResult, ulErrorCode);
		return lResult;
	}

	return lResult;
}
