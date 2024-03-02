#pragma once

#include <pt/vector.h>
#include <pcg32.h>

namespace sobol {

static constexpr float FloatOneMinusEpsilon = 0x1.fffffep-1;

static constexpr int NSobolDimensions = 1024;
static constexpr int SobolMatrixSize = 52;
extern const uint32_t SobolMatrices32[NSobolDimensions * SobolMatrixSize];

extern const uint64_t VdCSobolMatrices[][SobolMatrixSize];
extern const uint64_t VdCSobolMatricesInv[][SobolMatrixSize];

extern inline float sobolSample(int64_t a, int dimension);

extern inline uint64_t sobolIntervalToIndex(uint32_t m, uint64_t frame, pt::Vector2i p);

}

namespace pt {

class Sampler {
public:
    Sampler(uint32_t spp = 1) : m_spp(spp) { }

    uint32_t getSPP() const { return m_spp; }

    virtual std::unique_ptr<Sampler> clone() const = 0;

    virtual void startBlockSample(const Vector2i& offset) = 0;

    virtual void startPixelSample(const Vector2i& p, int sampleIndex) = 0;

	virtual inline float sample1D() = 0;

	virtual inline Vector2f sample2D() = 0;

    virtual inline Vector2f samplePixel2D() = 0;

protected:
    uint32_t m_spp;
};


class IndependentSampler : public Sampler {
public:
    IndependentSampler(uint32_t spp = 1) : Sampler(spp) { }

    std::unique_ptr<Sampler> clone() const {
        std::unique_ptr<IndependentSampler> cloned(new IndependentSampler());
        cloned->m_spp = m_spp;
        cloned->m_random = m_random;
        return std::move(cloned);
    }

    void startBlockSample(const Vector2i& offset) {
        m_random.seed(offset.x(), offset.y());
    }

    void startPixelSample(const Vector2i& p, int sampleIndex) { }

    inline float sample1D() { 
        return m_random.nextFloat(); 
    }

    inline Vector2f sample2D() {
        return Vector2f(
            m_random.nextFloat(), 
            m_random.nextFloat()
        );
    }

    inline Vector2f samplePixel2D() { return sample2D(); }

private:
    pcg32 m_random;
};


class SobolSampler : public Sampler {
public:
    SobolSampler(uint32_t spp, Vector2i resolution);

    std::unique_ptr<Sampler> clone() const;

    void startBlockSample(const Vector2i& offset) { }

    void startPixelSample(const Vector2i& p, int sampleIndex);

    inline float sample1D();

    inline Vector2f sample2D();

    inline Vector2f samplePixel2D();

private:
    float sampleDimension(int dimension) const { return sobol::sobolSample(m_sobolIndex, dimension); }

    int64_t m_sobolIndex;
    int m_dimension, m_scale;
    Vector2i m_pixel;
};

}