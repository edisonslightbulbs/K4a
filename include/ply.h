#ifndef PLY_H
#define PLY_H

#include <k4a/k4a.h>

namespace ply {
/** write color write file */
void write(const k4a_image_t& point_cloud_image, const k4a_image_t& color_image,
    const char* file_name);

/** downscale images 2x2  binning */
k4a_image_t downscale(const k4a_image_t& color_image);

}
#endif /* PLY_H */
