#ifndef FRAME_H
#define FRAME_H

#include <opencv2/opencv.hpp> // remove this
#include <k4a/k4a.h>
#include <vector>

#include "point.h"

class Frame {
public:
    k4a_image_t m_rgb;
    k4a_image_t m_depth;
    k4a_image_t m_xyTable;
    k4a_image_t m_pointcloud;
    k4a_capture_t m_capture = nullptr;

    std::vector<Point> m_points;

    /**
     * @param t_device
     *   Id of kinect device.
     *
     * @param t_config
     *   Configuration settings for the kinect device. */
    Frame(k4a_device_t& t_device, const uint32_t& t_timeout,
        const k4a_image_t& t_xyTable, const k4a_image_t& t_depth,
        const k4a_image_t& t_pointcloud);

    void release() const;
};
#endif /* FRAME_H */
