#include <fstream>
#include <iostream>
#include <set>
#include <vector>

#include "io.h"
#include "ply.h"
#include "point.h"

#define PLY_HEADER                                                             \
    std::ofstream ofs(FILE);                                                   \
    ofs << "ply" << std::endl;                                                 \
    ofs << "format ascii 1.0" << std::endl;                                    \
    ofs << "element vertex" << " " << points.size() << std::endl;              \
    ofs << "property float x" << std::endl;                                    \
    ofs << "property float y" << std::endl;                                    \
    ofs << "property float z" << std::endl;                                    \
    ofs << "property uchar red" << std::endl;                                  \
    ofs << "property uchar green" << std::endl;                                \
    ofs << "property uchar blue" << std::endl;                                 \
    ofs << "end_header" << std::endl;                                          \
    ofs.close()

#define red " 215 48 39"
#define orange " 244 109 67"
#define gold " 253 173 97"
#define brown " 254 224 144"
#define yellow " 255 255 191"
#define sky " 224 243 248"
#define ocean " 171 217 233"
#define blue " 116 173 209"
#define deep " 69 117 180"
#define black " 0 0 0"

struct t_rgbPoint {
    int16_t xyz[3];
    uint8_t rgb[3];
};

void ply::write(const k4a_image_t& pclImage, const k4a_image_t& rgbImage,
    const std::string& FILE)
{
    std::vector<t_rgbPoint> points;
    int width = k4a_image_get_width_pixels(pclImage);
    int height = k4a_image_get_height_pixels(rgbImage);

    auto* point_cloud_image_data
        = (int16_t*)(void*)k4a_image_get_buffer(pclImage);
    uint8_t* color_image_data = k4a_image_get_buffer(rgbImage);

    for (int i = 0; i < width * height; i++) {
        t_rgbPoint point {};
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
    /** write to file */
    PLY_HEADER;
    std::stringstream ss;
    for (auto& point : points) {
        // image data is BGR
        ss << (float)point.xyz[0] << " " << (float)point.xyz[1] << " "
           << (float)point.xyz[2];
        ss << " " << (float)point.rgb[2] << " " << (float)point.rgb[1] << " "
           << (float)point.rgb[0];
        ss << std::endl;
    }
    std::ofstream ofs_text(FILE, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

std::vector<Point> colorize(
    std::vector<Point>& pcl, std::vector<Point>& context)
{
    Point centroid = Point::centroid(context);

    for (auto& point : pcl) {
        point.m_distance.second = point.distance(centroid);
    }
    for (auto& point : context) {
        point.m_distance.second = point.distance(centroid);
    }

    const int CONTEXT_CLUSTER = 100; // <- random value

    std::set<Point> colorizedContext;
    for (auto& point : context) {
        point.m_cluster = CONTEXT_CLUSTER;
        colorizedContext.insert(point);
    }
    for (auto& point : pcl) {
        colorizedContext.insert(point);
    }
    std::vector<Point> colorized;
    colorized.assign(colorizedContext.begin(), colorizedContext.end());
    return colorized;
}

void ply::write(std::vector<Point>& raw, std::vector<Point>& context)
{
    std::vector<Point> points = colorize(raw, context);
    const std::string FILE = io::pwd() + "/output/context.ply";

    /** write to file */
    PLY_HEADER;
    std::stringstream ss;
    for (const auto& point : points) {
        if (point.m_cluster == 100) {
            ss << point.m_x << " " << point.m_y << " " << point.m_z
               << " 174 1 126" << std::endl;
            continue;
        }
        ss << point.m_x << " " << point.m_y << " " << point.m_z;
        ss << " " << 0 << " " << 0 << " " << 0 << std::endl;
    }
    std::ofstream ofs_text(FILE, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}

void ply::write(std::vector<Point>& points)
{
    const std::string FILE = io::pwd() + "/output/context.ply";

    /** write to file */
    PLY_HEADER;
    std::stringstream ss;
    for (const auto& point : points) {
        switch(point.m_cluster){
            case 0:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << red << std::endl;
                break;
            case 1:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << blue << std::endl;
                break;
            case 2:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << brown << std::endl;
                break;
            case 3:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << deep << std::endl;
                break;
            case 4:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << orange << std::endl;
                break;
            case 5:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << gold << std::endl;
                break;
            case 6:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << yellow << std::endl;
                break;
            case 7:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << sky << std::endl;
                break;
            case 8:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << ocean << std::endl;
                break;
            default:
                ss << point.m_x << " " << point.m_y << " " << point.m_z;
                ss << black << std::endl;
                break;
        }
    }
    std::ofstream ofs_text(FILE, std::ios::out | std::ios::app);
    ofs_text.write(ss.str().c_str(), (std::streamsize)ss.str().length());
}