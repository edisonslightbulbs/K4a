#include "kinect.h"
#include "io.h"
#include "ply.h"
#define path io::pwd()

extern std::shared_ptr<bool> RUN_SYSTEM;

void Kinect::construct()
{
    auto* pclData = (int16_t*)(void*)k4a_image_get_buffer(m_pclImage);
    uint8_t* colorData = k4a_image_get_buffer(m_rgb2depthImage);

    std::vector<float> segment(m_numPoints * 3);
    for (int i = 0; i < m_numPoints; i++) {
        if (pclData[3 * i + 2] == 0) {
            (*sptr_pcl)[3 * i + 0] = 0.0f;
            (*sptr_pcl)[3 * i + 1] = 0.0f;
            (*sptr_pcl)[3 * i + 2] = 0.0f;

            /** kinect color reversed! */
            (*sptr_color)[3 * i + 2] = colorData[4 * i + 0];
            (*sptr_color)[3 * i + 1] = colorData[4 * i + 1];
            (*sptr_color)[3 * i + 0] = colorData[4 * i + 2];
            continue;
        }
        (*sptr_pcl)[3 * i + 0] = (float)pclData[3 * i + 0];
        (*sptr_pcl)[3 * i + 1] = (float)pclData[3 * i + 1];
        (*sptr_pcl)[3 * i + 2] = (float)pclData[3 * i + 2];
        (*sptr_color)[3 * i + 2] = colorData[4 * i + 0];
        (*sptr_color)[3 * i + 1] = colorData[4 * i + 1];
        (*sptr_color)[3 * i + 0] = colorData[4 * i + 2];

        if (m_contextUpper.m_z == __FLT_MAX__
            || m_contextLower.m_z == __FLT_MIN__) {
            continue;
        }
        /** filter interaction context */
        if ((float)pclData[3 * i + 0] > m_contextUpper.m_x
            || (float)pclData[3 * i + 0] < m_contextLower.m_x
            || (float)pclData[3 * i + 1] > m_contextUpper.m_y
            || (float)pclData[3 * i + 1] < m_contextLower.m_y
            || (float)pclData[3 * i + 2] > m_contextUpper.m_z
            || (float)pclData[3 * i + 2] < m_contextLower.m_z) {
            continue;
        }
        segment[3 * i + 0] = (float)pclData[3 * i + 0];
        segment[3 * i + 1] = (float)pclData[3 * i + 1];
        segment[3 * i + 2] = (float)pclData[3 * i + 2];
    }
    /** iff context boundaries are not set default to full point cloud */
    if (m_contextUpper.m_z == __FLT_MAX__
        || m_contextLower.m_z == __FLT_MIN__) {
        *sptr_context = *sptr_pcl;
    } else {
        *sptr_context = segment;
    }
    // todo: tidy up
}

void Kinect::capture()
{
    /** initiate capture sequence */
    switch (k4a_device_get_capture(m_device, &m_capture, m_timeout)) {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;
    case K4A_WAIT_RESULT_TIMEOUT:
        throw std::runtime_error("Capture timed out!");
    case K4A_WAIT_RESULT_FAILED:
        throw std::runtime_error("Failed to capture!");
    }

    /** capture colour image */
    m_rgbImage = k4a_capture_get_color_image(m_capture);
    if (m_rgbImage == nullptr) {
        throw std::runtime_error("Failed to get color image!");
    }

    /** capture depth image */
    m_depthImage = k4a_capture_get_depth_image(m_capture);
    if (m_depthImage == nullptr) {
        throw std::runtime_error("Failed to get depth image!");
    }

    int depthWidth = k4a_image_get_width_pixels(m_depthImage);
    int depthHeight = k4a_image_get_height_pixels(m_depthImage);

    /** initialize rgb image using depth image dimensions */
    m_rgb2depthImage = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, depthWidth,
            depthHeight, depthWidth * 4 * (int)sizeof(uint8_t),
            &m_rgb2depthImage)) {
        throw std::runtime_error(
            "Failed to initialize rgb image using depth image dimensions!");
    }

    /** initialize pcl image using depth image dimensions */
    m_pclImage = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depthWidth, depthHeight,
            depthWidth * 3 * (int)sizeof(int16_t), &m_pclImage)) {
        throw std::runtime_error("Failed to initialize point cloud image using "
                                 "depth image dimensions!");
    }
}
void Kinect::transform(const int& transformType)
{
    /** DEPTH -> POINT CLOUD */
    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(m_transform,
            m_depthImage, K4A_CALIBRATION_TYPE_DEPTH, m_pclImage)) {
        throw std::runtime_error(
            "Failed to transform depth image to point cloud image!");
    }

    switch (transformType) {

    /** RGB -> DEPTH */
    case 1: {
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_color_image_to_depth_camera(
                m_transform, m_depthImage, m_rgbImage, m_rgb2depthImage)) {
            throw std::runtime_error(
                "Failed to create rgb2depth point cloud image!");
        }
        /** dev option: */
        // m_file = path + "/output/rgb2depth.ply";
        // ply::write(m_pclImage, m_rgb2depthImage, m_file);
        break;
    }

    /** DEPTH -> RGB */
    case 2: {
        int colorWidth = k4a_image_get_width_pixels(m_rgbImage);
        int colorHeight = k4a_image_get_height_pixels(m_rgbImage);

        /** initialize depth2rgb image using rgb image dimensions */
        m_depth2rgbImage = nullptr;
        if (K4A_RESULT_SUCCEEDED
            != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth,
                colorHeight, colorWidth * (int)sizeof(uint16_t),
                &m_depth2rgbImage)) {
            throw std::runtime_error(
                "Failed to initialize depth2rgb point cloud image!");
        }

        /** re-initialize pcl image using rgb image dimensions */
        m_pclImage = nullptr;
        if (K4A_RESULT_SUCCEEDED
            != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, colorWidth,
                colorHeight, colorWidth * 3 * (int)sizeof(int16_t),
                &m_pclImage)) {
            throw std::runtime_error("Failed to initialize point cloud image "
                                     "using rgb image dimensions!");
        }

        /** transform: depth -> rgb */
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_depth_image_to_color_camera(
                m_transform, m_depthImage, m_depth2rgbImage)) {
            throw std::runtime_error(
                "Failed to create depth2rgb point cloud image!");
        }

        /** transform: depth -> point cloud */
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_depth_image_to_point_cloud(m_transform,
                m_depth2rgbImage, K4A_CALIBRATION_TYPE_COLOR, m_pclImage)) {
            throw std::runtime_error("Failed to compute point cloud!");
        }
        /** dev option: */
        // m_file = path + "/output/depth2rgb.ply";
        // ply::write(m_pclImage, m_depth2rgbImage, m_file);
        break;
    }
    default: {
        break;
    }
    }
}

/** transformation options for kinect */

void Kinect::record(const int& transformType)
{
    /** block threads from accessing
     *  resources during recording */
    std::lock_guard<std::mutex> lck(m_mutex);
    capture();
    transform(transformType);
    construct();
}

std::shared_ptr<std::vector<float>> Kinect::getPcl()
{
    /** allow multiple threads to read unfiltered point cloud */
    std::shared_lock lock(s_mutex);
    return sptr_pcl;
}

std::shared_ptr<std::vector<uint8_t>> Kinect::getColor()
{
    /** allow multiple threads to read pcl color */
    std::shared_lock lock(s_mutex);
    return sptr_color;
}

std::shared_ptr<std::vector<float>> Kinect::getContext()
{
    /** allow multiple threads to read interaction context */
    std::shared_lock lock(s_mutex);
    return sptr_context;
}

void Kinect::setContextBounds(const std::pair<Point, Point>& threshold)
{
    /** dis-allow threads from accessing
     *  resources during context definition */
    std::unique_lock lock(s_mutex);
    m_contextLower = threshold.first;
    m_contextUpper = threshold.second;
}

int Kinect::getNumPoints()
{
    /** allow multiple threads to read image resolution */
    std::shared_lock lock(s_mutex);
    return m_numPoints;
}

void Kinect::release() const
{
    if (m_capture != nullptr) {
        k4a_capture_release(m_capture);
    }
    if (m_rgbImage != nullptr) {
        k4a_image_release(m_rgbImage);
    }
    if (m_depthImage != nullptr) {
        k4a_image_release(m_depthImage);
    }
    if (m_pclImage != nullptr) {
        k4a_image_release(m_pclImage);
    }
    if (m_rgb2depthImage != nullptr) {
        k4a_image_release(m_rgb2depthImage);
    }
    if (m_depth2rgbImage != nullptr) {
        k4a_image_release(m_depth2rgbImage);
    }
}

void Kinect::close() const
{
    // todo: destroying this nonchalant yields undesired results
    // if (m_transform != nullptr) {
    //     k4a_transformation_destroy(m_transform);
    // }
    if (m_device != nullptr) {
        k4a_device_close(m_device);
    }
}

Kinect::~Kinect() { close(); }

Kinect::Kinect()
{
    /** setup kinect  */
    DEVICE_CONF deviceConf;
    m_device = deviceConf.m_device;
    m_timeout = deviceConf.TIMEOUT;

    /** initialize boundless interaction context */
    m_contextLower = Point(__FLT_MIN__, __FLT_MIN__, __FLT_MIN__);
    m_contextUpper = Point(__FLT_MAX__, __FLT_MAX__, __FLT_MAX__);

    /** check for kinect */
    uint32_t deviceCount = k4a_device_get_installed_count();
    if (deviceCount == 0) {
        throw std::runtime_error("No device found!");
    }
    /** open kinect */
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_open(K4A_DEVICE_DEFAULT, &m_device)) {
        throw std::runtime_error("Unable to open device!");
    }
    /** calibrate */
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_get_calibration(m_device, deviceConf.m_config.depth_mode,
            deviceConf.m_config.color_resolution, &m_calibration)) {
        throw std::runtime_error("Unable to calibrate!");
    }
    /** start cameras */
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_start_cameras(m_device, &deviceConf.m_config)) {
        throw std::runtime_error("Failed to start cameras!");
    }
    /** get transform */
    m_transform = k4a_transformation_create(&m_calibration);

    /** capture dry run */
    capture();
}
