#include "frame.h"

#include "kexception.h"
#include "logger.h"
#include "point.h"
#include "timer.h"

Frame::Frame(k4a_device_t& t_device, const uint32_t& t_timeout,
    const k4a_image_t& t_xyTable, const k4a_image_t& t_depth,
    const k4a_image_t& t_pointcloud)
    : m_xyTable(t_xyTable)
    , m_depth(t_depth)
    , m_pointcloud(t_pointcloud)
{
    Timer timer;

    /** capture surface view */
    switch (k4a_device_get_capture(t_device, &m_capture, t_timeout)) {
    case K4A_WAIT_RESULT_SUCCEEDED:
        break;

    case K4A_WAIT_RESULT_TIMEOUT:
        try {
            throw kexception();
        } catch (kexception& e) {
            LOG(WARNING) << kexception::captureTimeout();
        }

    case K4A_WAIT_RESULT_FAILED:
        try {
            throw kexception();
        } catch (kexception& e) {
            LOG(WARNING) << kexception::captureRead();
        }
    }

    /** get depth image */
    m_depth = k4a_capture_get_depth_image(m_capture);
    try {
        if (m_depth == nullptr) {
            throw kexception();
        }
    } catch (kexception& e) {
        LOG(WARNING) << kexception::captureDepth();
    }

    /* get rgb image */
    m_rgb = k4a_capture_get_color_image(m_capture);
    try {
        if (m_rgb == nullptr) {
            throw kexception();
        }
    } catch (kexception& e) {
        LOG(WARNING) << kexception::captureDepth();
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
    LOG(INFO) << "kinect image frame captured in: " << timer.getDuration();
}

void Frame::release() const
{
    k4a_image_release(m_depth);
    k4a_capture_release(m_capture);
    k4a_image_release(m_xyTable);
    k4a_image_release(m_pointcloud);
}
