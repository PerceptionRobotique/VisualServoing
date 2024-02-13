// a mix between libuvc-theta-sample and Crombez' insta360 ros package as a C++ class compatible with VS programs
// Guillaume Caron
// JRL, AIST, Tsukuba, Japan
// Feb. 2024
#ifndef __CamUVCtheta_H__
#define  __CamUVCtheta_H__

#include "libuvc/libuvc.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <stdio.h>
#include <unistd.h>

using namespace cv;

#include <visp/vpImage.h>
#include <visp/vpImageConvert.h>

#include "thirdparty/libuvc-theta-sample/thetauvc.h"

#include "thirdparty/insta360-crombez/videoDecoder.h"

using namespace std;

template<typename T>
class CamUVCtheta
{
public:

  CamUVCtheta(int wdth, int heigh, int dpth, int camID = 0, void *filename=NULL): depth(dpth)
  {
    firstTimeIn = true;
		/* Initialize a UVC service context. Libuvc will set up its own libusb
   * context. Replace NULL with a libusb_context pointer to run libuvc
   * from an existing libusb context. */
    res = uvc_init(&ctx, NULL);

    if (res < 0)
    {
        ctx_OK = false;
        uvc_perror(res, "uvc_init");
    }
    else
    {
        ctx_OK = true;
        puts("UVC initialized");

        /* Locates the first attached UVC device, stores in dev */
        res = thetauvc_find_device(
            ctx, &dev,
            0);//, 0, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

        if (res < 0)
        {
            dev_OK = false;
            uvc_perror(res, "uvc_find_device"); /* no devices found */
        }
        else
        {
            dev_OK = true;
            puts("Device found");

            /* Try to open the device: requires exclusive access */
            res = uvc_open(dev, &devh);

            if (res < 0)
            {
                devh_OK = false;
                uvc_perror(res, "uvc_open"); /* unable to open device */
            }
            else
            {
                devh_OK = true;
                puts("Device opened");

                /* Print out a message containing all the information that libuvc
        * knows about the device */
                uvc_print_diag(devh, stderr);

                res = thetauvc_get_stream_ctrl_format_size(devh,
                        THETAUVC_MODE_FHD_2997, &ctrl); // for 1920x960 resolution
            //			THETAUVC_MODE_UHD_2997, &ctrl); // for 3840x1920 resolution

                /* Print out the result */
                uvc_print_stream_ctrl(&ctrl, stderr);

                if (res < 0)
                {
                    uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
                }
                else
                {
                    /* Start the video stream. The library will call user function cb:
            *   cb(frame, (void *) 12345)
            */
                    res = uvc_start_streaming(devh, &ctrl, cb, (void *)12345, 0);

                    if (res < 0)
                    {
                        uvc_perror(res, "start_streaming"); /* unable to start stream */
                    }
                    else
                    {
                        puts("Streaming...");
                    }
                }
            }
        }
    }
    
  }
  
  ~CamUVCtheta()
  {
    /* End the stream. Blocks until last callback is serviced */
    if(devh_OK)
    {
        uvc_stop_streaming(devh);
        puts("Done streaming.");

        /* Release our handle on the device */
        uvc_close(devh);
        puts("Device closed");
    }

    if(dev_OK)
    {
        /* Release the device descriptor */
        uvc_unref_device(dev);
    }

    if(ctx_OK)
    {
        /* Close the UVC context. This closes and cleans up any existing device handles,
   * and it closes the libusb context if one was not provided. */
        uvc_exit(ctx);
        puts("UVC exited");
    }
  }

  vpImage<T> getFrame()
  {
          getFrame(Ie);
          return Ie;
  }

  void getFrame(vpImage<T>& I)
  {
    if(firstTimeIn)
        return;

    switch(depth)
    {
        case 8:
        {

            //
            // Convert image to mono 8
            //
            //std::cout << "getFrame tries to lock" << std::endl;
            image_mutex.lock();
            I.resize(img.rows, img.cols, false);

            vpImageConvert::convert(img,I);
            //std::cout << "getFrame unlocks" << std::endl;
            image_mutex.unlock();

            break;
        }
        default:
        {
            cout << "Image bit depth not implemented" << endl;
            //return;
        }
    }
  }

    /* This callback function runs once per frame. Use it to perform any
    * quick processing you need, or have it put the frame into your application's
    * input queue. If this function takes too long, you'll start losing frames. */
    static void cb(uvc_frame_t *frame, void *ptr)
    {
        if(firstTimeIn)
        {
            firstTimeIn = false;
        }

        //printf("Format: %d / %d, Width: %d, Height: %d, Bytes: %d\n", frame->frame_format, UVC_FRAME_FORMAT_H264, frame->width, frame->height, frame->data_bytes);
        int rret = vd.parse((const uint8_t* ) frame->data, frame->data_bytes);
        if (vd.pkt->data)
        {
            //std::cout << "cb tries to lock" << std::endl;
            image_mutex.lock();
            img = vd.decode((const uint8_t* ) frame->data, frame->data_bytes); 
            //std::cout << "cb unlocks" << std::endl;
            image_mutex.unlock();
            //std::cout << "img: " << img.rows << " "  << rret << std::endl;
        }
        else
        {
            //std::cerr << "parse failed: " << rret << std::endl;
        }
    }
  
  void setAutoWhiteBalance(bool set=true)
  {
    cout << "CamUVCtheta::setAutoWhiteBalance not implemented" << endl;
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
    cout << "CamUVCtheta::setAutoGain not implemented" << endl;
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

	uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    uvc_error_t res;

    bool devh_OK;
    bool dev_OK;
    bool ctx_OK;

	vpImage<T> Ie;
	int depth;
    static cv::Mat img;

    static std::mutex image_mutex;
    static bool firstTimeIn;

    static videoDecoder vd;

};

template<typename T>
bool CamUVCtheta<T>::firstTimeIn = true;
template<typename T>
std::mutex CamUVCtheta<T>::image_mutex;
template<typename T>
cv::Mat CamUVCtheta<T>::img;

template<typename T>
videoDecoder CamUVCtheta<T>::vd;

#endif // __CamUVCtheta_H__
