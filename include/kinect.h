#ifndef KINECT_H
#define KINECT_H

#include <atomic>
#include <cfloat>
#include <k4a/k4a.h>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include "logger.h"
#include "point.h"
#include "timer.h"

/**
 * @struct DEVICE_CONF
 *    Class-like single container for all kinect config
 * @b Reference
 *    https://docs.microsoft.com/en-us/azure/kinect-dk/hardware-specification#depth-camera-supported-operating-modes
 */
struct DEVICE_CONF {

    const int32_t TIMEOUT = 1000;
    k4a_device_t m_device = nullptr;
    k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    DEVICE_CONF()
    {
        // m_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
        // m_config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
        m_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        m_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        m_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        m_config.synchronized_images_only = true;
    }
};

/**
 * @file kinect.h
 *    Kinect device.
 * @b Reference
 *    https://github.com/microsoft//tree/develop/examples/fastpointcloud
 */
class Kinect {

public:
    /**
     * capture
     *   Capture depth and color images using the kinect device.
     */
    void getCapture();

    /**
     * getPclImage
     *   Calibrates point-cloud image resolution based on depth
     *   image resolution.
     *
     * @param sptr_points
     *   "Safe global" share pointer to point cloud points.
     */
    void getPclImage();

    std::mutex m_mutex;
    std::shared_mutex s_mutex;
    int32_t m_timeout = 0;
    k4a_device_t m_device;
    k4a_image_t m_rgbImage = nullptr;
    k4a_capture_t m_capture = nullptr;
    k4a_image_t m_depthImage = nullptr;
    k4a_image_t m_pclImage = nullptr;
    k4a_calibration_t m_calibration {};
    k4a_transformation_t m_transformation {};

    /** tabletop interaction context boundary */
    Point pclLowerBoundary;
    Point pclUpperBoundary;

    int numPoints = 640 * 576;

    /** segmented context */
    std::shared_ptr<std::vector<float>> sptr_context
        = std::make_shared<std::vector<float>>(numPoints * 3);

    /** tabletop environment */
    std::shared_ptr<std::vector<float>> sptr_pcl
        = std::make_shared<std::vector<float>>(numPoints * 3);

    /**
     * close
     *   Closes connection to kinect device.
     */
    void close() const;

    /**
     * close
     *   Releases kinect's  capture resources.
     */
    void release() const;

    /**
     * Kinect
     *   Kinect device constructor.
     */
    Kinect();

    ~Kinect();

    void setContext(std::pair<Point, Point> threshold);

    std::shared_ptr<std::vector<float>> getContextPcl();

    std::shared_ptr<std::vector<float>> getPcl();

    int getNumPoints();
};
#endif /* KINECT_H */
