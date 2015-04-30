//=============================================================================
// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: GrabCallbackEx.cpp,v 1.1 2010-05-20 18:01:36 soowei Exp $
//=============================================================================

#include "GrabCallbackClass.h"
#include <pluginlib/class_list_macros.h>// NODELET_INFO

using namespace FlyCapture2;
using namespace ptgrey_simple;


extern "C" { 
void C_OnImageGrabbed(Image* pImage, const void* pCallbackData)
  {
    printf( "Grabbed image \n");
    return;
  }
}


void GrabCallbackClass::PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf( 
        version, 
        "FlyCapture2 library version: %d.%d.%d.%d\n", 
        fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

    printf("%s", version );

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    printf("%s", timeStamp );
}

void GrabCallbackClass::PrintCameraInfo( )
{
    //if (!pCamInfo) { printf( "Can not print Camera Info, pCamInfo is NULL!" ); return; }
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        camInfo.serialNumber,
        camInfo.modelName,
        camInfo.vendorName,
        camInfo.sensorInfo,
        camInfo.sensorResolution,
        camInfo.firmwareVersion,
        camInfo.firmwareBuildTime );
}//PrintCameraInfo

  
  int GrabCallbackClass::RunSingleCamera( 
    void (*callback)(Image* pImage, const void* pCallbackData) 
    )
  {
    const int k_numImages = 10;

    //Error error;
    //Camera cam;

    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError( );
	OnError();
    }

    // Get the camera information
    //CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError();
        OnError();
    }

    PrintCameraInfo();        

    // Start capturing images
    error = cam.StartCapture(callback);
    if (error != PGRERROR_OK)
    {
        PrintError( );
        OnError();
    }

    Property frameRateProp(FRAME_RATE);
    error = cam.GetProperty(&frameRateProp);
    if (error != PGRERROR_OK)
    {
        PrintError( );
	OnError();
        
    }
    return 0;
}//RunSingleCamera
    
    
int GrabCallbackClass::StopSingleCamera(  )  {  
    
    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError();
	OnError();
        
    }      

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError();
	OnError();
        
    }

	// Reset counter for next iteration
	imageCnt = 0;

    return 0;
}//StopSingleCamera

//int main(int /*argc*/, char** /*argv*/)
int GrabCallbackClass::InitPointGrey( )
{    
    PrintBuildInfo();

    //Error error;

    //BusManager busMgr;
    //unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( );
        return -1;
    }

    printf( "Number of cameras detected: %u\n", numCameras );

    for (unsigned int i=0; i < 1; i++)//numCameras; i++) // by now do it only once
    {
        //PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK)
        {
            PrintError(  );
            return -1;
        }
    }
    return 0;
}// Init PointGray

void GrabCallbackClass::PrintError( )
  {
    printf ("\nERROR: %s ",error.GetDescription() );
    printf ("\nfile %s (%ud)",error.GetFilename(),error.GetLine() );
    error.PrintErrorTrace(); // maybe it is not needed, the previous lines do the same and we can send into ROSERROR
  }

void GrabCallbackClass::OnError( )
  {
    printf(" On Error called. kill");
    StopSingleCamera(  );
    exit(0); // by now, kill the process. TODO improve this.
  }//onerror
  
  
  