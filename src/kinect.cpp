#include "kinect.h"

void Kinect::getCapture()
{
    std::lock_guard<std::mutex> lck(m_mutex);
    /** capture */
    switch (k4a_device_get_capture(m_device, &m_capture, m_timeout)) {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        throw std::runtime_error("Timed out waiting for a kinect capture.");
    case K4A_WAIT_RESULT_FAILED:
        throw std::runtime_error("Failed to get a capture using kinect.");
    }

    /** capture depth image */
    m_depthImage = k4a_capture_get_depth_image(m_capture);
    if (m_depthImage == nullptr) {
        throw std::runtime_error("Failed to get capture depth image.");
    }

    /** capture colour image */
    m_rgbImage = k4a_capture_get_color_image(m_capture);
    if (m_rgbImage == nullptr) {
        throw std::runtime_error("Failed to capture color image.");
    }
}

void Kinect::getPclImage()
{
    // std::lock_guard<std::mutex> lck (m_mutex);
    /** transform depth image to point cloud */
    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(m_transformation,
            m_depthImage, K4A_CALIBRATION_TYPE_DEPTH, m_pclImage)) {
        throw std::runtime_error("Failed to compute point cloud.");
    }
    auto* point_cloud_image_data
        = (int16_t*)(void*)k4a_image_get_buffer(m_pclImage);

    if (m_mutex.try_lock()) {
        for (int i = 0; i < getNumPoints(); i++) {
            if (point_cloud_image_data[3 * i + 2] == 0) {
                (*sptr_points)[3 * i + 0] = 0.f;
                (*sptr_points)[3 * i + 1] = 0.f;
                (*sptr_points)[3 * i + 2] = 0.f;
                continue;
            }
            (*sptr_points)[3 * i + 0]
                = (float)point_cloud_image_data[3 * i + 0];
            (*sptr_points)[3 * i + 1]
                = (float)point_cloud_image_data[3 * i + 1];
            (*sptr_points)[3 * i + 2]
                = (float)point_cloud_image_data[3 * i + 2];
        }
        m_mutex.unlock();
    }
}

Kinect::Kinect()
{
    DEVICE_CONF deviceConf;
    m_device = deviceConf.m_device;
    m_timeout = deviceConf.TIMEOUT;

    Point maxConstraint;
    Point minConstraint;
    std::pair<Point, Point> threshold(minConstraint, maxConstraint);
    sptr_threshold = std::make_shared<std::pair<Point, Point>>(threshold);

    /** open kinect, calibrate device, start cameras, and get transform */
    uint32_t deviceCount = k4a_device_get_installed_count();
    if (deviceCount == 0) {
        throw std::runtime_error("No device found.");
    }
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_open(K4A_DEVICE_DEFAULT, &m_device)) {
        throw std::runtime_error("Unable to open device.");
    }
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_get_calibration(m_device, deviceConf.m_config.depth_mode,
            deviceConf.m_config.color_resolution, &m_calibration)) {
        throw std::runtime_error("Unable to get calibration.");
    }
    m_transformation = k4a_transformation_create(&m_calibration);

    if (K4A_RESULT_SUCCEEDED
        != k4a_device_start_cameras(m_device, &deviceConf.m_config)) {
        throw std::runtime_error("Failed to start cameras.");
    }

    /** capture images */
    getCapture();

    int depthWidth = k4a_image_get_width_pixels(m_depthImage);
    int depthHeight = k4a_image_get_height_pixels(m_depthImage);

    /** create point cloud image */
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depthWidth, depthHeight,
            depthWidth * 3 * (int)sizeof(int16_t), &m_pclImage)) {
        throw std::runtime_error("Failed to create point cloud image.");
    }
}

void Kinect::setContext(std::pair<Point, Point> threshold)
{
    if (m_mutex.try_lock()) {
        sptr_threshold->first = threshold.first;
        sptr_threshold->second = threshold.second;
        m_mutex.unlock();
    }
}

std::vector<float> Kinect::getPcl()
{
    std::lock_guard<std::mutex> lck(m_mutex);
    return *sptr_points;
}

int Kinect::getNumPoints() const { return NUM_POINTS; }

void Kinect::updateContext()
{
    Point min;
    Point max;
    // std::lock_guard<std::mutex> lck (m_mutex);
    if (m_mutex.try_lock()) {
        min = sptr_threshold->first;
        max = sptr_threshold->second;
        m_mutex.unlock();
    }
    if (m_mutex.try_lock()) {
        for (int i = 0; i < getNumPoints(); i++) {
            if ((*sptr_points)[3 * i + 0] > max.m_x
                || (*sptr_points)[3 * i + 0] < min.m_x
                || (*sptr_points)[3 * i + 1] > max.m_y
                || (*sptr_points)[3 * i + 1] < min.m_y
                || (*sptr_points)[3 * i + 2] > max.m_z
                || (*sptr_points)[3 * i + 2] < min.m_z) {
                (*sptr_points)[3 * i + 0] = 0.f;
                (*sptr_points)[3 * i + 1] = 0.f;
                (*sptr_points)[3 * i + 2] = 0.f;
                continue;
            }
        }
        m_mutex.unlock();
    }
}

void Kinect::release() const
{
    if (m_depthImage != nullptr) {
        k4a_image_release(m_depthImage);
    }
    if (m_capture != nullptr) {
        k4a_capture_release(m_capture);
    }
    if (m_transformation != nullptr) {
        k4a_transformation_destroy(m_transformation);
    }
}

void Kinect::close() const
{
    if (m_device != nullptr) {
        k4a_device_close(m_device);
    }
}

Kinect::~Kinect()
{
    release();
    close();
}
