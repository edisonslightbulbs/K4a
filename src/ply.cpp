// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "ply.h"

struct color_point_t {
    int16_t xyz[3];
    uint8_t rgb[3];
};

void ply::write(const k4a_image_t& point_cloud_image,
    const k4a_image_t& color_image, const char* file_name)
{
    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(point_cloud_image);
    int height = k4a_image_get_height_pixels(color_image);

    auto* point_cloud_image_data
        = (int16_t*)(void*)k4a_image_get_buffer(point_cloud_image);
    uint8_t* color_image_data = k4a_image_get_buffer(color_image);

    for (int i = 0; i < width * height; i++) {
        color_point_t point {};
        point.xyz[0] = point_cloud_image_data[3 * i + 0];
        point.xyz[1] = point_cloud_image_data[3 * i + 1];
        point.xyz[2] = point_cloud_image_data[3 * i + 2];
        if (point.xyz[2] == 0) {
            continue;
        }

        point.rgb[0] = color_image_data[4 * i + 0];
        point.rgb[1] = color_image_data[4 * i + 1];
        point.rgb[2] = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];

        if (point.rgb[0] == 0 && point.rgb[1] == 0 && point.rgb[2] == 0
            && alpha == 0) {
            continue;
        }

        points.push_back(point);
    }

    std::ofstream ofs(file_name); // text mode first
    ofs << "ply" << std::endl;
    ofs << "format ascii 1.0" << std::endl;
    ofs << "element vertex"
        << " " << points.size() << std::endl;
    ofs << "property float x" << std::endl;
    ofs << "property float y" << std::endl;
    ofs << "property float z" << std::endl;
    ofs << "property uchar red" << std::endl;
    ofs << "property uchar green" << std::endl;
    ofs << "property uchar blue" << std::endl;
    ofs << "end_header" << std::endl;
    ofs.close();

    std::stringstream ss;
    for (auto& point : points) {
        // image data is BGR
        ss << (float)point.xyz[0] << " " << (float)point.xyz[1] << " "
           << (float)point.xyz[2];
        ss << " " << (float)point.rgb[2] << " " << (float)point.rgb[1] << " "
           << (float)point.rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(file_name, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

k4a_image_t ply::downscale(const k4a_image_t& color_image)
{
    int color_image_width_pixels = k4a_image_get_width_pixels(color_image);
    int color_image_height_pixels = k4a_image_get_height_pixels(color_image);
    int color_image_downscaled_width_pixels = color_image_width_pixels / 2;
    int color_image_downscaled_height_pixels = color_image_height_pixels / 2;
    k4a_image_t color_image_downscaled = nullptr;
    if (K4A_RESULT_SUCCEEDED
        != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
            color_image_downscaled_width_pixels,
            color_image_downscaled_height_pixels,
            color_image_downscaled_width_pixels * 4 * (int)sizeof(uint8_t),
            &color_image_downscaled)) {
        printf("Failed to create downscaled color image\n");
        return color_image_downscaled;
    }

    uint8_t* color_image_data = k4a_image_get_buffer(color_image);
    uint8_t* color_image_downscaled_data
        = k4a_image_get_buffer(color_image_downscaled);
    for (int j = 0; j < color_image_downscaled_height_pixels; j++) {
        for (int i = 0; i < color_image_downscaled_width_pixels; i++) {
            int index_downscaled = j * color_image_downscaled_width_pixels + i;
            int index_tl = (j * 2 + 0) * color_image_width_pixels + i * 2 + 0;
            int index_tr = (j * 2 + 0) * color_image_width_pixels + i * 2 + 1;
            int index_bl = (j * 2 + 1) * color_image_width_pixels + i * 2 + 0;
            int index_br = (j * 2 + 1) * color_image_width_pixels + i * 2 + 1;

            color_image_downscaled_data[4 * index_downscaled + 0]
                = (uint8_t)((float)(color_image_data[4 * index_tl + 0]
                                + color_image_data[4 * index_tr + 0]
                                + color_image_data[4 * index_bl + 0]
                                + color_image_data[4 * index_br + 0])
                    / 4.0f);
            color_image_downscaled_data[4 * index_downscaled + 1]
                = (uint8_t)((float)(color_image_data[4 * index_tl + 1]
                                + color_image_data[4 * index_tr + 1]
                                + color_image_data[4 * index_bl + 1]
                                + color_image_data[4 * index_br + 1])
                    / 4.0f);
            color_image_downscaled_data[4 * index_downscaled + 2]
                = (uint8_t)((float)(color_image_data[4 * index_tl + 2]
                                + color_image_data[4 * index_tr + 2]
                                + color_image_data[4 * index_bl + 2]
                                + color_image_data[4 * index_br + 2])
                    / 4.0f);
            color_image_downscaled_data[4 * index_downscaled + 3]
                = (uint8_t)((float)(color_image_data[4 * index_tl + 3]
                                + color_image_data[4 * index_tr + 3]
                                + color_image_data[4 * index_bl + 3]
                                + color_image_data[4 * index_br + 3])
                    / 4.0f);
        }
    }

    return color_image_downscaled;
}
