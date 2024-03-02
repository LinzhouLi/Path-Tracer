#pragma once

#include <pt/common.h>

#define PT_FILTER_RESOLUTION 32

namespace pt {

class Filter {
public:
    Filter(float radius): m_radius(radius) { }

    /// Return the filter radius in fractional pixels
    float getRadius() const { return m_radius; }

    /// Evaluate the filter function
    virtual float eval(float x) const = 0;

protected:
    float m_radius;
};


class GaussianFilter : public Filter {
public:
    GaussianFilter(float radius = 2.0f, float stddev = 0.5f): Filter(radius), m_stddev(stddev) { }

    float eval(float x) const {
        float alpha = -1.0f / (2.0f * m_stddev * m_stddev);
        return std::max(0.0f,
            std::exp(alpha * x * x) -
            std::exp(alpha * m_radius * m_radius));
    }

protected:
    float m_stddev;
};

}