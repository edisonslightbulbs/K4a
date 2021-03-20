#if __linux__
#include "kinect.h"

Kinect::Kinect()
{
    Timer timer;

    settings kinect;
    m_device = kinect.m_device;
    uint32_t deviceCount = k4a_device_get_installed_count();
    try {
        if (deviceCount == 0) {
            throw kexception();
        }
    } catch (kexception& e) {
        LOG(WARNING) << kexception::findDevice();
    }

    /* open kinect */
    try {
        if (K4A_RESULT_SUCCEEDED
            != k4a_device_open(K4A_DEVICE_DEFAULT, &m_device)) {
            throw kexception();
        }
    } catch (kexception& e) {
        LOG(WARNING) << kexception::openDevice();
    }

    try {
        if (K4A_RESULT_SUCCEEDED
            != k4a_device_get_calibration(m_device, kinect.m_config.depth_mode,
                kinect.m_config.color_resolution, &m_calibration)) {
            throw kexception();
        }
    } catch (kexception& e) {
        LOG(WARNING) << kexception::calibrateDevice();
    }

    resolveDepth();

    try {
        if (K4A_RESULT_SUCCEEDED
            != k4a_device_start_cameras(m_device, &kinect.m_config)) {
            throw kexception();
        }
    } catch (kexception& e) {
        LOG(WARNING) << kexception::startCamera();
    }
    LOG(INFO) << "kinect initialized in: " << timer.getDuration();
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

void Kinect::resolveDepth()
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

Frame Kinect::getImage()
{
    return Frame { m_device, settings::TIMEOUT, m_xyTable, m_depth,
        m_pointcloud };
}

void Kinect::close() const
{
    if (m_device != nullptr) {
        k4a_device_close(m_device);
    }
}
#endif
