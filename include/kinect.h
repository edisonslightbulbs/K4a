#ifndef KINECT_H
#define KINECT_H

#if __linux__
#include <k4a/k4a.h>
#include <mutex>

/** @struct t_config
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
        m_config.color_resolution = K4A_COLOR_RESOLUTION_720P;
        m_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // 640 * 576
        m_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
        m_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        m_config.synchronized_images_only = true;
    }
};

extern const int RGB_TO_DEPTH;
extern const int DEPTH_TO_RGB;

/** @file kinect.h
 *    Kinect device.
 */
class Kinect {

public:
    std::mutex m_mutex;

    int32_t m_timeout = 0;
    k4a_device_t m_device;
    k4a_capture_t m_capture = nullptr;
    k4a_calibration_t m_calibration {};
    k4a_transformation_t m_transform = nullptr;

    k4a_image_t m_pcl = nullptr;   // point cloud
    k4a_image_t m_img = nullptr;   // color image
    k4a_image_t m_c2d = nullptr;   // color to depth image
    k4a_image_t m_d2c = nullptr;   // depth to color image
    k4a_image_t m_xyT = nullptr;   // xy table
    k4a_image_t m_xyPcl = nullptr; // point cloud from xy table
    k4a_image_t m_depth = nullptr; // depth image

    /** capture
     *   Captures images using k4a device.
     */
    void capture();

    /** transform
     *   Carries out requested transformation.
     *
     * @param transformFlag
     *   Flag for transformation types, i.e.,
     *   RGB_TO_DEPTH & DEPTH_TO_RGB
     */
    void transform(const int& transformFlag);

    /**
     * xyLookupTable
     *   Pre-computes a lookup table by storing x and y depth
     *   scale factors for every pixel.
     *
     * @param ptr_calibration
     *   Calibration for the kinect device in question.
     *
     * @param depthImg
     *   Depth map captured by the kinect device.
     */
    static void xyLookupTable(
        const k4a_calibration_t* ptr_calibration, k4a_image_t depthImg);

    /** close
     *   Closes k4a device.
     */
    void close() const;

    void releaseK4aImages() const;
    void releaseK4aCapture() const;
    __attribute__((unused)) void releaseK4aTransform();

    void xyTable();
    void imgCapture();
    void pclCapture();
    void c2dCapture();
    void depthCapture();

    Kinect();
    ~Kinect();
};
#endif
#endif /* KINECT_H */
