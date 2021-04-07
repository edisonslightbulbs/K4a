#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <k4a/k4a.h>

#include "io.h"
#include "kinect.h"
#include "ply.h"

namespace transformer {

    // todo: pass kinect
bool pclColorToDepth(k4a_transformation_t transformation_handle,
    const k4a_image_t& depth_image, const k4a_image_t& color_image,
    const std::string& file_name)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);

    // todo: add as class member
    k4a_image_t transformed_color_image = nullptr;

    // todo: move to bottom of capture method
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
            depth_image_width_pixels, depth_image_height_pixels,
            depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
            &transformed_color_image)) {
        throw std::runtime_error("Failed to create transformed color image!");
    }

    // todo: remove
    // todo: currently all don in capture method !!!
    k4a_image_t point_cloud_image = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depth_image_width_pixels,
            depth_image_height_pixels,
            depth_image_width_pixels * 3 * (int)sizeof(int16_t),
            &point_cloud_image)) {
        throw std::runtime_error("Failed to create point cloud image!");
    }
    // todo: currently all don in capture method !!!

    // todo: move to  top of transform method
    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_color_image_to_depth_camera(transformation_handle,
            depth_image, color_image, transformed_color_image)) {
        printf("Failed to compute transformed color image\n");
        throw std::runtime_error("Failed to create point cloud image!");
    }

    // todo: currently all done in transform method !!!
    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
            depth_image, K4A_CALIBRATION_TYPE_DEPTH, point_cloud_image)) {
        throw std::runtime_error("Failed to create point cloud image!");
    }
    // todo: currently all done in transform method !!!

    ply::write(point_cloud_image, transformed_color_image, file_name.c_str());

    // todo: handled in kinect
    k4a_image_release(transformed_color_image);
    k4a_image_release(point_cloud_image);
    return true;
}

bool pclDepthToColor(k4a_transformation_t transformation_handle,
    const k4a_image_t& depth_image, const k4a_image_t& color_image,
    const std::string& file_name)
{
    // transform color image into depth camera geometry
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    k4a_image_t transformed_depth_image = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16, color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * (int)sizeof(uint16_t),
            &transformed_depth_image)) {
        //throw std::runtime_error("Failed to create transformed dept image!");
        return false;
    }

    k4a_image_t point_cloud_image = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, color_image_width_pixels,
            color_image_height_pixels,
            color_image_width_pixels * 3 * (int)sizeof(int16_t),
            &point_cloud_image)) {
        //throw std::runtime_error("Failed to create point cloud image!");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_color_camera(
            transformation_handle, depth_image, transformed_depth_image)) {
       // throw std::runtime_error("Failed to create transformed depth image!");
        return false;
    }

    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
            transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR,
            point_cloud_image)) {
        //throw std::runtime_error("Failed to compute point cloud!");
        return false;
    }

    ply::write(point_cloud_image, color_image, file_name.c_str());

    // todo: handled in kinect
    k4a_image_release(transformed_depth_image);
    k4a_image_release(point_cloud_image);
    return true;
}

    void transform(std::shared_ptr<Kinect>& sptr_kinect)
{
    const std::string C2D = io::pwd() + "/output/color2depth.ply";
    if (!pclColorToDepth(sptr_kinect->m_transform, sptr_kinect->m_depthImage,
            sptr_kinect->m_rgbImage, C2D)) {
        sptr_kinect->release();
        //sptr_kinect->close();
        //throw std::runtime_error("Failed to create point cloud image!");
    }

    // Compute color point cloud by warping depth image into color camera
    // geometry
    const std::string D2C = io::pwd() + "/output/depth2color.ply";
    if (!pclDepthToColor(sptr_kinect->m_transform, sptr_kinect->m_depthImage,
            sptr_kinect->m_rgbImage, D2C)) {
        sptr_kinect->release();
        //sptr_kinect->close();
        //throw std::runtime_error("Failed to create point cloud image!");
    }
}
}

#endif /* TRANSFORMER */
