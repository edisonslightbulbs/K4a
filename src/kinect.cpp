#include "kinect.h"
#include "io.h"
#include "ply.h"

extern std::shared_ptr<bool> RUN_SYSTEM;

void Kinect::defineContext()
{
    int width = k4a_image_get_width_pixels(m_pclImage);
    int height = k4a_image_get_height_pixels(m_transformedRgbImage);

    auto* point_cloud_image_data
        = (int16_t*)(void*)k4a_image_get_buffer(m_pclImage);

    uint8_t* color_image_data = k4a_image_get_buffer(m_transformedRgbImage);

    /** iff context boundaries undefined */
    std::vector<float> segment(m_numPoints * 3);
    for (int i = 0; i < m_numPoints; i++) {
        if (point_cloud_image_data[3 * i + 0] == 0
            || point_cloud_image_data[3 * i + 1] == 0
            || point_cloud_image_data[3 * i + 2] == 0) {
            (*sptr_pcl)[3 * i + 0] = 0.0f;
            (*sptr_pcl)[3 * i + 1] = 0.0f;
            (*sptr_pcl)[3 * i + 2] = 0.0f;

            /** colors from the kinect are in reverse
             *  order thus, assign in reverse order */
            (*sptr_color)[3 * i + 2] = color_image_data[4 * i + 0];
            (*sptr_color)[3 * i + 1] = color_image_data[4 * i + 1];
            (*sptr_color)[3 * i + 0] = color_image_data[4 * i + 2];
            continue;
        }
        (*sptr_pcl)[3 * i + 0] = (float)point_cloud_image_data[3 * i + 0];
        (*sptr_pcl)[3 * i + 1] = (float)point_cloud_image_data[3 * i + 1];
        (*sptr_pcl)[3 * i + 2] = (float)point_cloud_image_data[3 * i + 2];

        /** colors from the kinect are in reverse
         *  order thus, assign in reverse order */
        (*sptr_color)[3 * i + 2] = color_image_data[4 * i + 0];
        (*sptr_color)[3 * i + 1] = color_image_data[4 * i + 1];
        (*sptr_color)[3 * i + 0] = color_image_data[4 * i + 2];

        // if ((*sptr_color)[4 * i + 0] == 0 &&(*sptr_color)[4 * i + 1] == 0 &&
        // (*sptr_color)[4 * i + 2] == 0
        //     && (*sptr_color)[4 * i + 3] == 0) {
        //     continue;
        // }

        if ((float)point_cloud_image_data[3 * i + 0] > m_pclUpperBoundary.m_x
            || (float)point_cloud_image_data[3 * i + 0] < m_pclLowerBoundary.m_x
            || (float)point_cloud_image_data[3 * i + 1] > m_pclUpperBoundary.m_y
            || (float)point_cloud_image_data[3 * i + 1] < m_pclLowerBoundary.m_y
            || (float)point_cloud_image_data[3 * i + 2] > m_pclUpperBoundary.m_z
            || (float)point_cloud_image_data[3 * i + 2]
                < m_pclLowerBoundary.m_z) {
            continue;
        }
        segment[3 * i + 0] = (float)point_cloud_image_data[3 * i + 0];
        segment[3 * i + 1] = (float)point_cloud_image_data[3 * i + 1];
        segment[3 * i + 2] = (float)point_cloud_image_data[3 * i + 2];
    }

    /**   iff context boundaries undefined: context = unfiltered point cloud
     *  else, boundary defined segment as interaction context */
    if (m_pclUpperBoundary.m_z == __FLT_MAX__
        || m_pclLowerBoundary.m_z == __FLT_MIN__) {
        *sptr_context = *sptr_pcl;
    } else {
        *sptr_context = segment;
    }
}

void Kinect::getCapture()
{
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

    /** create rgb image */
    m_transformedRgbImage = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32, depthWidth,
            depthHeight, depthWidth * 4 * (int)sizeof(uint8_t),
            &m_transformedRgbImage)) {
        throw std::runtime_error("Failed to create transformed color image!");
    }

    /** create pcl image */
    m_pclImage = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depthWidth, depthHeight,
            depthWidth * 3 * (int)sizeof(int16_t), &m_pclImage)) {
        throw std::runtime_error("Failed to create point cloud image!");
    }
}
void Kinect::transform(const int& TRANSFORMATION_TYPE)
{
    /** transform depth image to pcl */
    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(m_transform,
            m_depthImage, K4A_CALIBRATION_TYPE_DEPTH, m_pclImage)) {
        throw std::runtime_error("Failed to compute point cloud.");
    }

    switch (TRANSFORMATION_TYPE) {
    case 1: {

        /** transform rgb image to depth image: RGB_TO_DEPTH */
        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_color_image_to_depth_camera(
                m_transform, m_depthImage, m_rgbImage, m_transformedRgbImage)) {
            throw std::runtime_error("Failed to create point cloud image!");
        }
        // /** dev option: write rgb-depth ply file */
        // const std::string c2d = io::pwd() + "/output/color2depth.ply";
        // ply::write(m_pclLowerBoundary, m_pclUpperBoundary, m_pclImage,
        //     m_transformedRgbImage, c2d);
        break;
    }

    case 2: {

        /** transform depth image into color image: DEPTH_TO_RGB */
        int colorWidth = k4a_image_get_width_pixels(m_rgbImage);
        int colorHeight = k4a_image_get_height_pixels(m_rgbImage);

        m_transformedDepthImage = nullptr;
        if (K4A_RESULT_SUCCEEDED
            != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, colorWidth,
                colorHeight, colorWidth * (int)sizeof(uint16_t),
                &m_transformedDepthImage)) {
            throw std::runtime_error(
                "Failed to create transformed dept image!");
        }

        m_pclImage = nullptr;
        if (K4A_RESULT_SUCCEEDED
            != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, colorWidth,
                colorHeight, colorWidth * 3 * (int)sizeof(int16_t),
                &m_pclImage)) {
            throw std::runtime_error("Failed to create point cloud image!");
        }

        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_depth_image_to_color_camera(
                m_transform, m_depthImage, m_transformedDepthImage)) {
            throw std::runtime_error(
                "Failed to create transformed depth image!");
        }

        if (K4A_RESULT_SUCCEEDED
            != k4a_transformation_depth_image_to_point_cloud(m_transform,
                m_transformedDepthImage, K4A_CALIBRATION_TYPE_COLOR,
                m_pclImage)) {
            throw std::runtime_error("Failed to compute point cloud!");
        }
        // /** dev option write depth-rgb ply file */
        // const std::string d2c = io::pwd() + "/output/depth2color.ply";
        // ply::write(m_pclImage, m_transformedRgbImage, d2c);
        break;
    }
    default: {
        break;
    }
    }
}

/** transformation options for kinect */

void Kinect::capturePcl(const int& TYPE_OF_TRANSFORMATION)
{
    /** block threads from accessing resources
     *  during capture and transformation */
    std::lock_guard<std::mutex> lck(m_mutex);
    getCapture();
    transform(TYPE_OF_TRANSFORMATION);
    defineContext();
    // release();
}

std::shared_ptr<std::vector<float>> Kinect::getPcl()
{
    /** allow multiple threads to read unfiltered point cloud */
    std::shared_lock lock(s_mutex);
    return sptr_pcl;
}

k4a_image_t Kinect::getTransformedRgb()
{
    /** allow multiple threads to read transformed rgb image*/
    std::shared_lock lock(s_mutex);
    return m_transformedRgbImage;
}

std::shared_ptr<std::vector<uint8_t>> Kinect::getColor()
{
    /** allow multiple threads to read pcl color */
    std::shared_lock lock(s_mutex);
    return sptr_color;
}

k4a_image_t Kinect::getTransformedDepth()
{
    /** allow multiple threads to read transformed rgb image*/
    std::shared_lock lock(s_mutex);
    return m_transformedDepthImage;
}

std::shared_ptr<std::vector<float>> Kinect::getContext()
{
    /** allow multiple threads to read interaction context */
    std::shared_lock lock(s_mutex);
    return sptr_context;
}

void Kinect::setContextBounds(std::pair<Point, Point> threshold)
{
    /** dis-allow threads from accessing
     *  resources during context definition */
    std::unique_lock lock(s_mutex);
    m_pclLowerBoundary = threshold.first;
    m_pclUpperBoundary = threshold.second;
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
    if (m_transformedRgbImage != nullptr) {
        k4a_image_release(m_transformedRgbImage);
    }
    if (m_transformedDepthImage != nullptr) {
        k4a_image_release(m_transformedDepthImage);
    }
}

void Kinect::close() const
{
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
    m_pclLowerBoundary = Point(__FLT_MIN__, __FLT_MIN__, __FLT_MIN__);
    m_pclUpperBoundary = Point(__FLT_MAX__, __FLT_MAX__, __FLT_MAX__);

    /** open kinect, calibrate device, start cameras, and get transform */
    uint32_t deviceCount = k4a_device_get_installed_count();
    if (deviceCount == 0) {
        throw std::runtime_error("No device found!");
    }
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_open(K4A_DEVICE_DEFAULT, &m_device)) {
        throw std::runtime_error("Unable to open device!");
    }
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_get_calibration(m_device, deviceConf.m_config.depth_mode,
            deviceConf.m_config.color_resolution, &m_calibration)) {
        throw std::runtime_error("Unable to calibrate!");
    }
    if (K4A_RESULT_SUCCEEDED
        != k4a_device_start_cameras(m_device, &deviceConf.m_config)) {
        throw std::runtime_error("Failed to start cameras!");
    }
    m_transform = k4a_transformation_create(&m_calibration);

    /** capture dry run */
    getCapture();
}
