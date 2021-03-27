#include "kinect.h"

Kinect::Kinect()
{
    settings t_kinect;
    m_device = t_kinect.m_device;
    m_timeout = t_kinect.TIMEOUT;

    uint32_t deviceCount = k4a_device_get_installed_count();

        if (deviceCount == 0) {
            throw std::runtime_error("No device found.\n");
        }

    /* open kinect */
        if (K4A_RESULT_SUCCEEDED
            != k4a_device_open(K4A_DEVICE_DEFAULT, &m_device)) {
            throw std::runtime_error("Unable to open device.\n");
        }

        if (K4A_RESULT_SUCCEEDED
            != k4a_device_get_calibration(m_device, t_kinect.m_config.depth_mode,
                                          t_kinect.m_config.color_resolution, &m_calibration)) {
            throw std::runtime_error("Unable to get calibration.\n");
        }

    createImages();

        if (K4A_RESULT_SUCCEEDED
            != k4a_device_start_cameras(m_device, &t_kinect.m_config)) {
            throw std::runtime_error("Failed to start cameras.\n");
        }

    /** capture surface view */
    switch (k4a_device_get_capture(m_device, &m_capture,m_timeout)) {
        case K4A_WAIT_RESULT_SUCCEEDED:
            break;
        case K4A_WAIT_RESULT_TIMEOUT:
            throw std::runtime_error("Time out exceeded.\n");
        case K4A_WAIT_RESULT_FAILED:
            throw std::runtime_error("Failed to get capture.\n");
    }

    /** get depth image */
    m_depth = k4a_capture_get_depth_image(m_capture);
    if (m_depth == nullptr) {
        throw std::runtime_error("Failed to get depth image.\n");
    }

    /* get rgb image */
    m_rgb = k4a_capture_get_color_image(m_capture);
    if (m_rgb == nullptr) {
        throw std::runtime_error("Failed to get color image.\n");
    }

    /* compute point cloud points using xy table */
    int depthWidth = k4a_image_get_width_pixels(m_depth);
    int depthHeight = k4a_image_get_height_pixels(m_depth);

    auto* depthData = (uint16_t*)(void*)k4a_image_get_buffer(m_depth);
    auto* k4aImageData = (k4a_float2_t*)(void*)k4a_image_get_buffer(m_xyTable);
    auto* pclData = (k4a_float3_t*)(void*)k4a_image_get_buffer(m_pointcloud);

    int numPoints = 0;
    for (int i = 0; i < depthWidth * depthHeight; i++) {
        if (depthData[i] != 0 && !std::isnan(k4aImageData[i].xy.x)
            && !std::isnan(k4aImageData[i].xy.y)) {
            pclData[i].xyz.x = k4aImageData[i].xy.x * (float)depthData[i];
            pclData[i].xyz.y = k4aImageData[i].xy.y * (float)depthData[i];
            pclData[i].xyz.z = (float)depthData[i];
            (numPoints)++;
        } else {
            pclData[i].xyz.x = nanf("");
            pclData[i].xyz.y = nanf("");
            pclData[i].xyz.z = nanf("");
        }
    }

    int pclWidth = k4a_image_get_width_pixels(m_pointcloud);
    int pclHeight = k4a_image_get_height_pixels(m_pointcloud);

    auto* points = (k4a_float3_t*)(void*)k4a_image_get_buffer(m_pointcloud);

    for (int i = 0; i < pclWidth * pclHeight; i++) {

        /* filter valid points */
        if (std::isnan(points[i].xyz.x) || std::isnan(points[i].xyz.y)
            || std::isnan(points[i].xyz.z)) {
            continue;
        }
        auto x = (float)(points[i].xyz.x);
        auto y = (float)(points[i].xyz.y);
        auto z = (float)(points[i].xyz.z);
        Point point(x, y, z);
        m_points.push_back(point);
    }
}
void Kinect::xyLookupTable(
    const k4a_calibration_t* t_calibration, k4a_image_t t_depth)
{
    auto* table_data = (k4a_float2_t*)(void*)k4a_image_get_buffer(t_depth);

    int width = t_calibration->depth_camera_calibration.resolution_width;
    int height = t_calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++) {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++) {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(t_calibration, &p, 1.0F,
                K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray,
                &valid);

            if (valid != 0) {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            } else {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

void Kinect::createImages()
{
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        m_calibration.depth_camera_calibration.resolution_width,
        m_calibration.depth_camera_calibration.resolution_height,
        m_calibration.depth_camera_calibration.resolution_width
            * (int)sizeof(k4a_float2_t),
        &m_xyTable);

    xyLookupTable(&m_calibration, m_xyTable);

    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
        m_calibration.depth_camera_calibration.resolution_width,
        m_calibration.depth_camera_calibration.resolution_height,
        m_calibration.depth_camera_calibration.resolution_width
            * (int)sizeof(k4a_float3_t),
        &m_pointcloud);
}

void Kinect::release() const
{
    k4a_image_release(m_depth);
    k4a_capture_release(m_capture);
    k4a_image_release(m_xyTable);
    k4a_image_release(m_pointcloud);
}

void Kinect::close() const
{
    if (m_device != nullptr) {
        k4a_device_close(m_device);
    }
}
