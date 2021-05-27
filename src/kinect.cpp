#include <cmath>

#include "kinect.h"
#include "logger.h"

extern const int RGB_TO_DEPTH = 1;
extern const int DEPTH_TO_RGB = 2;

void Kinect::capture()
{
    switch (k4a_device_get_capture(m_device, &m_capture, m_timeout)) {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        LOG(FATAL) << "-- capture timed out!";
    case K4A_WAIT_RESULT_FAILED:
        LOG(FATAL) << "-- failed to capture!";
    }
}

void Kinect::depthCapture()
{
    m_depth = k4a_capture_get_depth_image(m_capture);
    if (m_depth == nullptr) {
        LOG(FATAL) << "-- failed to get depth image!";
    }
}

void Kinect::imgCapture()
{
    m_img = k4a_capture_get_color_image(m_capture);
    if (m_img == nullptr) {
        LOG(FATAL) << "-- failed to get color image!";
    }
}

void Kinect::pclCapture()
{
    int depthWidth = k4a_image_get_width_pixels(m_depth);
    int depthHeight = k4a_image_get_height_pixels(m_depth);

    m_pcl = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depthWidth, depthHeight,
            depthWidth * 3 * (int)sizeof(int16_t), &m_pcl)) {
        LOG(FATAL) << "-- failed to create point cloud";
    }
}

void Kinect::c2dCapture()
{
    int depthWidth = k4a_image_get_width_pixels(m_depth);
    int depthHeight = k4a_image_get_height_pixels(m_depth);

    m_c2d = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, depthWidth,
            depthHeight, depthWidth * 4 * (int)sizeof(uint8_t), &m_c2d)) {
        LOG(FATAL) << "-- failed to create c2d image!";
    }
}

void Kinect::transform(const int& transformFlag)
{
    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(
            m_transform, m_depth, K4A_CALIBRATION_TYPE_DEPTH, m_pcl)) {
        LOG(FATAL) << "-- failed to transform depth image to point cloud!";
    }

    switch (transformFlag) {

    case RGB_TO_DEPTH: {
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_color_image_to_depth_camera(
                m_transform, m_depth, m_img, m_c2d)) {
            LOG(FATAL) << "-- failed to transform rgb to depth!";
        }
        break;
    }

    case DEPTH_TO_RGB: {
        int colorWidth = k4a_image_get_width_pixels(m_img);
        int colorHeight = k4a_image_get_height_pixels(m_img);

        m_d2c = nullptr;
        if (K4A_RESULT_SUCCEEDED
            != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth,
                colorHeight, colorWidth * (int)sizeof(uint16_t), &m_d2c)) {
            LOG(FATAL) << "-- failed to initialize depth 2 rgb transformation!";
        }

        m_pcl = nullptr;
        if (K4A_RESULT_SUCCEEDED
            != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, colorWidth,
                colorHeight, colorWidth * 3 * (int)sizeof(int16_t), &m_pcl)) {
            LOG(FATAL) << "-- failed to initialize depth 2 rgb transformation!";
        }

        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_depth_image_to_color_camera(
                m_transform, m_depth, m_d2c)) {
            LOG(FATAL) << "-- failed to transform depth to rgb!";
        }
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_depth_image_to_point_cloud(
                m_transform, m_d2c, K4A_CALIBRATION_TYPE_COLOR, m_pcl)) {
            LOG(FATAL) << "-- failed to create DEPTH_TO_RGB point cloud!";
        }
        break;
    }
    default: {
        break;
    }
    }
}

void Kinect::xyLookupTable(
    const k4a_calibration_t* ptr_calibration, k4a_image_t depthImg)
{
    auto* xyTable = (k4a_float2_t*)(void*)k4a_image_get_buffer(depthImg);
    int depthImgWidth
        = ptr_calibration->depth_camera_calibration.resolution_width;
    int depthImgHeight
        = ptr_calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t point;
    k4a_float3_t ray;

    int valid;
    for (int y = 0, idx = 0; y < depthImgHeight; y++) {
        point.xy.y = (float)y;

        for (int x = 0; x < depthImgWidth; x++, idx++) {
            point.xy.x = (float)x;
            k4a_calibration_2d_to_3d(ptr_calibration, &point, 1.0F,
                K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray,
                &valid);

            if (valid != 0) {
                xyTable[idx].xy.x = ray.xyz.x;
                xyTable[idx].xy.y = ray.xyz.y;
            } else {
                xyTable[idx].xy.x = nanf("");
                xyTable[idx].xy.y = nanf("");
            }
        }
    }
}

void Kinect::xyTable()
{

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        m_calibration.depth_camera_calibration.resolution_width,
        m_calibration.depth_camera_calibration.resolution_height,
        m_calibration.depth_camera_calibration.resolution_width
            * (int)sizeof(k4a_float2_t),
        &m_xyT);

    xyLookupTable(&m_calibration, m_xyT);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        m_calibration.depth_camera_calibration.resolution_width,
        m_calibration.depth_camera_calibration.resolution_height,
        m_calibration.depth_camera_calibration.resolution_width
            * (int)sizeof(k4a_float3_t),
        &m_xyPcl);
}

void Kinect::releaseK4aImages() const
{
    if (m_img != nullptr) {
        k4a_image_release(m_img);
    }
    if (m_depth != nullptr) {
        k4a_image_release(m_depth);
    }
    if (m_pcl != nullptr) {
        k4a_image_release(m_pcl);
    }
    if (m_c2d != nullptr) {
        k4a_image_release(m_c2d);
    }
    // if (m_xyT != nullptr) {
    //     k4a_image_release(m_xyT);
    // }
    // if (m_d2cImg != nullptr) {
    //     k4a_image_release(m_d2cImg);
    // }
}

void Kinect::releaseK4aCapture()
{
    if (m_capture != nullptr) {
        k4a_capture_release(m_capture);
    }
}

void Kinect::releaseK4aTransform()
{
    std::lock_guard<std::mutex> lck(m_mutex);
    if (m_transform != nullptr) {
        k4a_transformation_destroy(m_transform);
    }
}

void Kinect::close() const
{
    if (m_device != nullptr) {
        k4a_device_close(m_device);
    }
}

Kinect::~Kinect() { close(); }

Kinect::Kinect()
{
    // setup device config
    t_config deviceConf;
    m_device = deviceConf.m_device;
    m_timeout = deviceConf.TIMEOUT;

    // check for device
    uint32_t deviceCount = k4a_device_get_installed_count();
    if (deviceCount == 0) {
        LOG(FATAL) << "-- no device found!";
    }

    // connect to device
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_open(K4A_DEVICE_DEFAULT, &m_device)) {
        LOG(FATAL) << "-- unable to open device!";
    }

    // initialize device
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_get_calibration(m_device, deviceConf.m_config.depth_mode,
            deviceConf.m_config.color_resolution, &m_calibration)) {
        LOG(FATAL) << "-- unable to get device calibration!";
    }

    // start cameras
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_start_cameras(m_device, &deviceConf.m_config)) {
        LOG(FATAL) << "-- failed to start cameras!";
    }

    // retrieve transform
    m_transform = k4a_transformation_create(&m_calibration);

    // get fast point cloud */
    xyTable();

    // capture();
}
