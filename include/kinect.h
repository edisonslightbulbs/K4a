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
 *    Single container for all kinect config
 *
 * @b Reference
 *    https://docs.microsoft.com/en-us/azure/kinect-dk/hardware-specification#depth-camera-supported-operating-modes
 */
struct t_config {
    const int32_t TIMEOUT = 1000;
    k4a_device_t m_device = nullptr;
    k4a_device_configuration_t m_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    t_config()
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

extern const int RGB_TO_DEPTH;
extern const int DEPTH_TO_RGB;

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
    int m_numPoints = 640 * 576;

    /** device config */
    int32_t m_timeout = 0;
    k4a_device_t m_device;

    /** k4a images */
    k4a_image_t m_rgbImage = nullptr;
    k4a_image_t m_pclImage = nullptr;
    k4a_image_t m_depthImage = nullptr;
    k4a_image_t m_rgb2depthImage = nullptr;
    k4a_image_t m_depth2rgbImage = nullptr;

    /** k4a capture, calibration, and transformation */
    k4a_capture_t m_capture = nullptr;
    k4a_calibration_t m_calibration {};
    k4a_transformation_t m_transform = nullptr;

    /** context boundary */
    Point m_contextLower;
    Point m_contextUpper;

    /** pcl: unsegmented */
    std::shared_ptr<std::vector<float>> sptr_pcl
        = std::make_shared<std::vector<float>>(m_numPoints * 3);

    /** pcl: segmented */
    std::shared_ptr<std::vector<float>> sptr_context
        = std::make_shared<std::vector<float>>(m_numPoints * 3);

    /** pcl color */
    std::shared_ptr<std::vector<uint8_t>> sptr_color
        = std::make_shared<std::vector<uint8_t>>(m_numPoints * 3);

    /** output PLY file */
    std::string m_file;

    /**
     * capture
     *   Capture depth and color images.
     */
    void capture();

    /**
     * transform
     *   Calibrates point cloud image resolution.
     *
     * @param sptr_points
     *   "Safe global" share pointer to point cloud points.
     */
    void transform(const int& transformType);

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
     *     Raw point cloud.
     */
    std::shared_ptr<std::vector<float>> getPcl();

    /**
     * getColor
     *   Retrieves point cloud color.
     *
     *  @retval
     *     Point cloud color.
     */
    std::shared_ptr<std::vector<uint8_t>> getColor();

    /**
     * getContext
     *   Retrieve interaction context point cloud.
     *
     *  @retval
     *     Point cloud corresponding to context segment.
     */
    std::shared_ptr<std::vector<float>> getContext();

    /**
     * setContextBounds
     *   Sets min and max points of context boundary.
     *
     *  @param threshold
     *    Pair of points { min, max } corresponding to the
     *    lower-bound and lower-bound of interaction context.
     */
    void setContextBounds(const std::pair<Point, Point>& threshold);

    /**
     * record
     *   Records a frame using the kinect.
     *
     *  @param transformType
     *    Specification of transformation type:
     *      option 1: RGB_TO_DEPTH
     *      option 2: DEPTH_TO_RGB
     */
    void record(const int& transformType);

    /**
     * constructPcl
     *    Constructs a point cloud using  a transformation type.
     */
    void constructPcl();

    /**
     * close
     *   Closes connection to kinect.
     */
    void close() const;

    /**
     * release
     *   Releases kinect resources.
     */
    void release() const;

    /**
     * Kinect
     *   Initialize kinect device.
     */
    Kinect();

    /**
     * Kinect
     *   Destroy kinect object
     */
    ~Kinect();
};
#endif /* KINECT_H */
