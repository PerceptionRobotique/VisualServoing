// Guillaume Caron
// JRL, AIST, Tsukuba, Japan
// Nov. 2019

#include "CamFlir.h"
#include <iostream>

using namespace FlyCapture2;
using namespace std;

template<typename T>
CamFlir<T>::CamFlir(int wdth, int heigh, int dpth, int camID, void *filename) 
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

template<typename T>
CamFlir<T>::~CamFlir()
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
 
template<typename T>
void
CamFlir<T>::PrintCameraInfo(CameraInfo *pCamInfo)
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

template<typename T> 
void
CamFlir<T>::PrintError(Error error)
{ 
  error.PrintErrorTrace(); 
}

template<typename T>
vpImage<T> 
CamFlir<T>::getFrame() 
{
        getFrame(Ie);
        return Ie;
}

template<typename T>
void 
CamFlir<T>::getFrame(vpImage<T>& I) 
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
    }
    default:
    {
      cout << "Image bit depth not implemented" << endl;
      return;
    }
  }
}

template<typename T>
PGRGuid 
CamFlir<T>::getPGRGuid() 
{
        return guid;
}

template<typename T>
void 
CamFlir<T>::setAutoWhiteBalance(bool set) 
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
 
template<typename T>
void
CamFlir<T>::setAutoGain(bool set) 
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
