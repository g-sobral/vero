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


#ifndef GRABCALLBACKCLASS_H
#define GRABCALLBACKCLASS_H


#include <stdio.h>
#include <unistd.h>
#include <boost/shared_ptr.hpp>
#include "FlyCapture2.h"
using namespace FlyCapture2;

namespace ptgrey_simple {


class GrabCallbackClass  
{
public:
  GrabCallbackClass() {
     numCameras=0 ; imageCnt=0;
  };
  void PrintBuildInfo();
  void PrintCameraInfo(  );
  void PrintError( );
  
  void OnError( );
  
  void OnImageGrabbed(Image* pImage, const void* pCallbackData)
  {
    printf( "Grabbed image %lu \n", imageCnt++ );
    return;
  }

  
  int RunSingleCamera(
    void (*callback)(Image* pImage, const void* pCallbackData) 
     );

  //int RunSingleCamera( ); // run and stop tinham um parametro guid
  int StopSingleCamera();
  int InitPointGrey( );
  
private: 

  BusManager busMgr;
  PGRGuid guid;
  unsigned int numCameras;
  unsigned long imageCnt;
  CameraInfo camInfo;
  Error error;
  Camera cam;

  
};//class

}//namespace

// // It is not an exec anymore. 
//int main(int /*argc*/, char** /*argv*/)
/*
{    
    PrintBuildInfo();

    Error error;
 
    // Since this application saves images in the current folder
    // we must ensure that we have permission to write to this folder.
    // If we do not have permission, fail right away.
    FILE* tempFile = fopen("test.txt", "w+");
    if (tempFile == NULL)
    {
        printf("Failed to create file in current folder.  Please check permissions.\n");
        return -1;
    }
    fclose(tempFile);
    remove("test.txt");

    BusManager busMgr;
    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    printf( "Number of cameras detected: %u\n", numCameras );

    for (unsigned int i=0; i < numCameras; i++)
    {
        PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        RunSingleCamera( guid );
    }

    printf( "Done! Press Enter to exit...\n" );
    getchar();

    return 0;
}
*/

#endif