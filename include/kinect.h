#ifndef KINECT_H
#define KINECT_H

#include <atomic>
#include <cfloat>
#include <k4a/k4a.h>
#include <mutex>
#include <shared_mutex>

#include "point.h"

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
 */
class Kinect {

public:
    /** thread guards */
    std::mutex m_mutex;
    std::shared_mutex s_mutex;

    /** image resolution */
    int numPoints = 640 * 576;

    /** initialize device config */
    int32_t m_timeout = 0;
    k4a_device_t m_device;
    k4a_image_t m_rgbImage = nullptr;
    k4a_image_t m_pclImage = nullptr;
    k4a_capture_t m_capture = nullptr;
    k4a_image_t m_depthImage = nullptr;
    k4a_calibration_t m_calibration {};
    k4a_image_t m_rgbImageScaled = nullptr;
    k4a_calibration_t m_scaledRgbCalibration;
    k4a_transformation_t m_transform = nullptr;
    k4a_transformation_t m_transformScaled = nullptr;

    /** interaction context boundary */
    Point pclLowerBoundary;
    Point pclUpperBoundary;

    /** context pcl */
    std::shared_ptr<std::vector<float>> sptr_context
        = std::make_shared<std::vector<float>>(numPoints * 3);

    /** tabletop environment pcl */
    std::shared_ptr<std::vector<float>> sptr_pcl
        = std::make_shared<std::vector<float>>(numPoints * 3);

    /**
     * capture
     *   Capture depth and color images.
     */
    void getCapture();

    /**
     * transform
     *   Calibrates point cloud image resolution.
     *
     * @param sptr_points
     *   "Safe global" share pointer to point cloud points.
     */
    void transform();

    /**
     * getNumPoints
     *   Retrieve point cloud image resolution.
     */
    int getNumPoints();

    /**
     * getPcl
     *   Retrieve raw/unprocessed point cloud points.
     *
     *  @retval
     *     Point cloud corresponding to interaction context.
     */
    std::shared_ptr<std::vector<float>> getPcl();

    /**
     * getContext
     *   Retrieve interaction context point cloud.
     *
     *  @retval
     *     Point cloud corresponding to interaction context.
     */
    std::shared_ptr<std::vector<float>> getContext();

    /**
     * setContextBounds
     *   Define interaction context boundary.
     *
     *  @param threshold
     *    Pair of points { min, max } corresponding to the
     *    lower-bound and lower-bound of interaction context.
     */
    void setContextBounds(std::pair<Point, Point> threshold);

    /**
     * capturePcl
     *   Capture and transform next point cloud frame
     */
    void capturePcl();

    /**
     * close
     *   Closes connection to device.
     */
    void close() const;

    /**
     * close
     *   Releases device resources.
     */
    void release() const;

    /**
     * Kinect
     *   Initialize kinect device.
     */
    Kinect();

    /**
     * Kinect
     *   De-initialize kinect device.
     */
    ~Kinect();

    void defineContext();
};
#endif /* KINECT_H */
