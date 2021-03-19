// #ifndef KINECT_H
// #define KINECT_H
//
// #include <k4a/k4a.h>
//
// #include "frame.h"
// #include "kexception.h"
// #include "logger.h"
// #include "timer.h"
//
//
//
// struct settings {
//     k4a_device_t m_device = nullptr;
//     static const int32_t TIMEOUT = 1000;
//     k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
//
//     settings(){
//         m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
//         m_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
//         m_config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
//         m_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
//         m_config.synchronized_images_only = true;
//     }
// };
//
//
//
// /**
//  * @file kinect.h
//  *    Kinect device.
//  *
//  *    Initializes the kinect device before image capturing.
//  *
//  * @b Reference
//  *    https://github.com/microsoft//tree/develop/examples/fastpointcloud */
// class Kinect {
// private:
//     /**
//      * xyLookupTable
//      *   Pre-computes a lookup table by storing x and y depth sc-
//      *   ale factors for every pixel.
//      *
//      * @param t_calibration
//      *   Calibration for the kinect device in question.
//      *
//      * @param t_depth
//      *   Depth map captured by the kinect device.
//      */
//     static void xyLookupTable(
//         const k4a_calibration_t* t_calibration, k4a_image_t t_depth);
//
//     /**
//      * resolveDepth
//      *   Calibrates point-cloud image resolution based on depth
//      *   image resolution
//      */
//     void resolveDepth();
//
// public:
//     k4a_device_t m_device;
//     k4a_calibration_t m_calibration {};
//
//     k4a_image_t m_xyTable = nullptr;
//     k4a_image_t m_depth = nullptr;
//     k4a_image_t m_pointcloud = nullptr;
//
//     /**
//      * Kinect
//      *
//      * @param t_device
//      *   Id of kinect device.
//      *
//      * @param t_config
//      *   Configuration settings for the kinect device. */
//     Kinect();
//
//     /**
//      * getImage
//      *   Uses kinect to capture an image.
//      *
//      * @param t_timeout
//      *   Timeout window for capturing an image.
//      *
//      * @retval
//      *    Return an Image.
//      */
//     Frame getImage();
//
//     /**
//      * close
//      *   Closes connection to kinect device
//      */
//     void close() const;
// };
// #endif /* KINECT_H */
