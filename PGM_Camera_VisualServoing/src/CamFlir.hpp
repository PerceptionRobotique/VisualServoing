// Guillaume Caron
// JRL, AIST, Tsukuba, Japan
// Nov. 2019
#ifndef __CamFlir_H__
#define  __CamFlir_H__

#include <visp/vpImage.h>

#include "FlyCapture2.h"

using namespace FlyCapture2;
using namespace std;

template<typename T>
class CamFlir
{
public:

  CamFlir(int wdth, int heigh, int dpth, int camID = 0, void *filename=NULL): depth(dpth)
  {
    error = busMgr.GetCameraFromIndex(camID, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        //return -1;
    }

    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        //return -1;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        //return -1;
    }

    PrintCameraInfo(&camInfo);

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration(&config);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        //return -1;
    }

    // Set the number of driver buffers used to 10.
    config.numBuffers = 10;

    // Set the camera configuration
    error = cam.SetConfiguration(&config);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        //return -1;
    }

    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        //return -1;
    }

  }
  
  ~CamFlir()
  {
    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        //return -1;
    }

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        //return -1;
    }
  }

  PGRGuid getPGRGuid()
  {
          return guid;
  }


  vpImage<T> getFrame()
  {
          getFrame(Ie);
          return Ie;
  }

  void getFrame(vpImage<T>& I)
  {
    // Retrieve an image
    error = cam.RetrieveBuffer(&rawImage);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }

    switch(depth)
    {
      case 8:
      {
        // Convert the raw image
        error = rawImage.Convert(PIXEL_FORMAT_MONO8, &convertedImage);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return;
        }

        I.resize(convertedImage.GetRows(), convertedImage.GetCols());

        memcpy(I.bitmap, convertedImage.GetData(), convertedImage.GetDataSize());
        break;
      }
      default:
      {
        cout << "Image bit depth not implemented" << endl;
        return;
      }
    }
  }
  
  void setAutoWhiteBalance(bool set=true)
  {
    cout << "CamFlir::setAutoWhiteBalance not implemented" << endl;
  /*        double empty;
          double on = set ? 1 : 0;
          int retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &on, &empty);
          if (retInt != IS_SUCCESS) {
                  throw CamuEyeException(hCam, retInt);
          }
  */
  }
  
  void setAutoGain(bool set=true)
  {
    cout << "CamFlir::setAutoGain not implemented" << endl;
  /*
          double empty;
          double on = set ? 1 : 0;
          int retInt = is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &on, &empty);
          if (retInt != IS_SUCCESS) {
                  throw CamuEyeException(hCam, retInt);
          }
  */
  }

private:
  void PrintCameraInfo(CameraInfo *pCamInfo)
  {
      cout << endl;
      cout << "*** CAMERA INFORMATION ***" << endl;
      cout << "Serial number - " << pCamInfo->serialNumber << endl;
      cout << "Camera model - " << pCamInfo->modelName << endl;
      cout << "Camera vendor - " << pCamInfo->vendorName << endl;
      cout << "Sensor - " << pCamInfo->sensorInfo << endl;
      cout << "Resolution - " << pCamInfo->sensorResolution << endl;
      cout << "Firmware version - " << pCamInfo->firmwareVersion << endl;
      cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << endl
           << endl;
  }

  void PrintError(Error error)
  { 
    error.PrintErrorTrace(); 
  }

        BusManager busMgr;
        PGRGuid guid;
      	Camera cam;

        vpImage<T> Ie;
        Image rawImage;
        Image convertedImage;
        
        int depth;

        Error error;
};

#endif // __CamFlir_H__
