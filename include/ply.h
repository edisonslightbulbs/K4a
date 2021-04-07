#ifndef PLY_H
#define PLY_H

#include <k4a/k4a.h>

namespace ply {
void write(const k4a_image_t& pclImage, const k4a_image_t& rgbImage,
    const std::string& file);
}
#endif /* PLY_H */
