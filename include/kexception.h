#ifndef KINECT_EXCEPTION_H
#define KINECT_EXCEPTION_H

#include <exception>

struct kexception : public std::exception {
public:
    static const char* findDevice() noexcept(false)
    {
        return "Failed to find Kinect device ";
    }

    static const char* openDevice() noexcept(false)
    {
        return "Failed to setup Kinect";
    }

    static const char* calibrateDevice() noexcept(false)
    {
        return "Failed to calibrate Kinect";
    }

    static const char* startCamera() noexcept(false)
    {
        return "Failed to start cameras";
    }

    static const char* captureTimeout() noexcept(false)
    {
        return "Capture timeout exceeded";
    }

    static const char* captureRead() noexcept(false)
    {
        return "Failed to read capture images";
    }

    static const char* captureDepth() noexcept(false)
    {
        return "Failed to get depth image from capture";
    }
};
#endif /* KINECT_EXCEPTION_H */
