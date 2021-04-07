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

void ply::write(const k4a_image_t& pclImage, const k4a_image_t& rgbImage,
    const std::string& file)
{
    std::vector<color_point_t> points;

    int width = k4a_image_get_width_pixels(pclImage);
    int height = k4a_image_get_height_pixels(rgbImage);

    auto* point_cloud_image_data
        = (int16_t*)(void*)k4a_image_get_buffer(pclImage);
    uint8_t* color_image_data = k4a_image_get_buffer(rgbImage);

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

    std::ofstream ofs(file); // text mode first
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
    std::ofstream ofs_text(file, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}
