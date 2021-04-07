#ifndef TRANSFORMER_H
#define TRANSFORMER_H

#include <k4a/k4a.h>

#include "io.h"
#include "kinect.h"
#include "ply.h"

namespace transformer {

bool pclColorToDepth(k4a_transformation_t transformation_handle,
    const k4a_image_t& depth_image, const k4a_image_t& color_image,
    const std::string& file_name)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    k4a_image_t transformed_color_image = nullptr;

    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
            depth_image_width_pixels, depth_image_height_pixels,
            depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
            &transformed_color_image)) {
        throw std::runtime_error("Failed to create transformed color image!");
    }

    k4a_image_t point_cloud_image = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM, depth_image_width_pixels,
            depth_image_height_pixels,
            depth_image_width_pixels * 3 * (int)sizeof(int16_t),
            &point_cloud_image)) {
        throw std::runtime_error("Failed to create point cloud image!");
    }

    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_color_image_to_depth_camera(transformation_handle,
            depth_image, color_image, transformed_color_image)) {
        printf("Failed to compute transformed color image\n");
        throw std::runtime_error("Failed to create point cloud image!");
    }

    if (K4A_RESULT_SUCCEEDED
        != k4a_transformation_depth_image_to_point_cloud(transformation_handle,
            depth_image, K4A_CALIBRATION_TYPE_DEPTH, point_cloud_image)) {
        throw std::runtime_error("Failed to create point cloud image!");
    }

    ply::write(point_cloud_image, transformed_color_image, file_name.c_str());

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

    // Compute color point cloud by warping depth image into color camera
    // geometry with downscaled color image and downscaled calibration. This
    // example's goal is to show how to configure the calibration and use the
    // transformation API as it is when the user does not need a point cloud
    // from high resolution transformed depth image. The downscaling method here
    // is naively to average binning 2x2 pixels, user should choose their own
    // appropriate downscale method on the color image, this example is only
    // demonstrating the idea. However, no matter what scale you choose to
    // downscale the color image, please keep the aspect ratio unchanged (to
    // ensure the distortion parameters from original calibration can still be
    // used for the downscaled image).
    memcpy(&sptr_kinect->m_scaledRgbCalibration, &sptr_kinect->m_calibration, sizeof(k4a_calibration_t));

    sptr_kinect->m_scaledRgbCalibration.color_camera_calibration .resolution_width /= 2;
    sptr_kinect->m_scaledRgbCalibration.color_camera_calibration .resolution_height /= 2;
    sptr_kinect->m_scaledRgbCalibration.color_camera_calibration.intrinsics .parameters.param.cx /= 2;
    sptr_kinect->m_scaledRgbCalibration.color_camera_calibration.intrinsics .parameters.param.cy /= 2;
    sptr_kinect->m_scaledRgbCalibration.color_camera_calibration.intrinsics .parameters.param.fx /= 2;
    sptr_kinect->m_scaledRgbCalibration.color_camera_calibration.intrinsics .parameters.param.fy /= 2;

    sptr_kinect->m_transformScaled = k4a_transformation_create(&sptr_kinect->m_scaledRgbCalibration);
    sptr_kinect->m_rgbImageScaled = ply::downscale(sptr_kinect->m_rgbImage);

    if (sptr_kinect->m_rgbImageScaled == nullptr) {
        sptr_kinect->release();
        //sptr_kinect->close();
        //throw std::runtime_error("Failed to downscaled color image!");
    }

    const std::string SCALED = io::pwd() + "/output/downscaled.ply";
    if (!pclDepthToColor(sptr_kinect->m_transformScaled,
            sptr_kinect->m_depthImage, sptr_kinect->m_rgbImageScaled, SCALED)) {
        sptr_kinect->release();
        //sptr_kinect->close();
        //throw std::runtime_error("Failed to transform depth to color!");
    }
}
}

#endif /* TRANSFORMER */
