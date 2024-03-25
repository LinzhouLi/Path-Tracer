/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <pt/color.h>
#include <pt/vector.h>

namespace pt {

/**
 * \brief Stores a RGB high dynamic-range bitmap
 *
 * The bitmap class provides I/O support using the OpenEXR file format
 */
class Bitmap : public Eigen::Array<Color3f, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> {
public:
    typedef Eigen::Array<Color3f, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Base;

    /**
     * \brief Allocate a new bitmap of the specified size
     *
     * The contents will initially be undefined, so make sure
     * to call \ref clear() if necessary
     */
    Bitmap(const std::string& name = "") : m_name(name) { }

    Bitmap(const Vector2i &size = Vector2i(0, 0), const std::string& name = "")
        : Base(size.y(), size.x()), m_name(name) { }

    /// Load an OpenEXR file with the specified filename
    void loadEXR(const std::string& filename);

    /// Load an PNG/JPG file (with sRGB tonemapping) with the specified filename
    void load(const std::string& filename);

    /// Save the bitmap as an EXR file with the specified filename
    void saveEXR(const std::string& filename);

    /// Save the bitmap as a PNG file (with sRGB tonemapping) with the specified filename
    void savePNG(const std::string& filename);

    Color3f sample(const Vector2f& uv) const;

    std::string toString() const {
        return tfm::format(
            "Bitmap[\n"
            "  name = %s,\n"
            "  size = [ %i, %i ]\n"
            "]",
            m_name, cols(), rows()
        );
    }

private:
    std::string m_name;
};

}
