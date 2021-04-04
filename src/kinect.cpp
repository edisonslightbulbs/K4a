#include "kinect.h"

extern std::mutex SYNCHRONIZE;
extern std::shared_ptr<bool> RUN_SYSTEM;

void Kinect::getCapture()
{
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

void Kinect::transformDepthImageToPcl()
{
    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(m_transformation,
            m_depthImage, K4A_CALIBRATION_TYPE_DEPTH, m_pclImage)) {
        throw std::runtime_error("Failed to compute point cloud.");
    }
    auto* point_cloud_image_data
        = (int16_t*)(void*)k4a_image_get_buffer(m_pclImage);

    /** iff context boundaries undefined */
    std::vector<float> runningPcl(numPoints * 3);
    for (int i = 0; i < getNumPoints(); i++) {
        if (point_cloud_image_data[3 * i + 0] == 0
            || point_cloud_image_data[3 * i + 1] == 0
            || point_cloud_image_data[3 * i + 2] == 0) {
            (*sptr_pcl)[3 * i + 0] = 0.0f;
            (*sptr_pcl)[3 * i + 1] = 0.0f;
            (*sptr_pcl)[3 * i + 2] = 0.0f;
            continue;
        }
        (*sptr_pcl)[3 * i + 0] = (float)point_cloud_image_data[3 * i + 0];
        (*sptr_pcl)[3 * i + 1] = (float)point_cloud_image_data[3 * i + 1];
        (*sptr_pcl)[3 * i + 2] = (float)point_cloud_image_data[3 * i + 2];

        if ((float)point_cloud_image_data[3 * i + 0] > pclUpperBoundary.m_x
            || (float)point_cloud_image_data[3 * i + 0] < pclLowerBoundary.m_x
            || (float)point_cloud_image_data[3 * i + 1] > pclUpperBoundary.m_y
            || (float)point_cloud_image_data[3 * i + 1] < pclLowerBoundary.m_y
            || (float)point_cloud_image_data[3 * i + 2] > pclUpperBoundary.m_z
            || (float)point_cloud_image_data[3 * i + 2]
                < pclLowerBoundary.m_z) {
            continue;
        }
        runningPcl[3 * i + 0] = (float)point_cloud_image_data[3 * i + 0];
        runningPcl[3 * i + 1] = (float)point_cloud_image_data[3 * i + 1];
        runningPcl[3 * i + 2] = (float)point_cloud_image_data[3 * i + 2];
    }

    /** iff context boundaries are undefined,
     *  let context = unfiltered point cloud  */
    if (pclUpperBoundary.m_z == __FLT_MAX__
        || pclLowerBoundary.m_z == __FLT_MIN__) {
        *sptr_context = *sptr_pcl;
    }
    /** ... else, default to filtered point cloud */
    *sptr_context = runningPcl;
}

void Kinect::getFrame()
{
    /** block threads from accessing resources
     *  during Capture and transformation */
    std::lock_guard<std::mutex> lck(m_mutex);
    getCapture();
    transformDepthImageToPcl();
}

std::shared_ptr<std::vector<float>> Kinect::getPcl()
{
    /** allow multiple threads to read unfiltered point cloud */
    std::shared_lock lock(s_mutex);
    return sptr_pcl;
}

std::shared_ptr<std::vector<float>> Kinect::getContext()
{
    /** allow multiple threads to read
     *  filtered interaction context point cloud */
    std::shared_lock lock(s_mutex);
    return sptr_context;
}

void Kinect::defineContext(std::pair<Point, Point> threshold)
{
    /** dis-allow threads from accessing resources
     *  during interaction context definition */
    std::unique_lock lock(s_mutex);
    pclLowerBoundary = threshold.first;
    pclUpperBoundary = threshold.second;
}

int Kinect::getNumPoints()
{
    /** allow multiple threads to read image resolution */
    std::shared_lock lock(s_mutex);
    return numPoints;
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
Kinect::Kinect()
{
    /** setup kinect  */
    DEVICE_CONF deviceConf;
    m_device = deviceConf.m_device;
    m_timeout = deviceConf.TIMEOUT;

    /** initialize boundless interaction context */
    pclLowerBoundary = Point(__FLT_MIN__, __FLT_MIN__, __FLT_MIN__);
    pclUpperBoundary = Point(__FLT_MAX__, __FLT_MAX__, __FLT_MAX__);

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

    /** create point cloud image */
    int depthWidth = k4a_image_get_width_pixels(m_depthImage);
    int depthHeight = k4a_image_get_height_pixels(m_depthImage);
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depthWidth, depthHeight,
            depthWidth * 3 * (int)sizeof(int16_t), &m_pclImage)) {
        throw std::runtime_error("Failed to create point cloud image.");
    }
}
