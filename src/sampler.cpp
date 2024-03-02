#include <pt/sampler.h>


namespace sobol {

inline float sobolSample(int64_t a, int dimension) {
    uint32_t v = 0;
    // Compute initial Sobol\+$'$ sample _v_ using generator matrices
    for (int i = dimension * SobolMatrixSize; a != 0; a >>= 1, i++)
        if (a & 1)
            v ^= SobolMatrices32[i];
    // No randomize
    return std::min(v * 0x1p-32f, FloatOneMinusEpsilon);
}

inline uint64_t sobolIntervalToIndex(uint32_t m, uint64_t frame, pt::Vector2i p) {
    if (m == 0)
        return frame;

    const uint32_t m2 = m << 1;
    uint64_t index = uint64_t(frame) << m2;

    uint64_t delta = 0;
    for (int c = 0; frame; frame >>= 1, ++c)
        if (frame & 1)  // Add flipped column m + c + 1.
            delta ^= VdCSobolMatrices[m - 1][c];

    // flipped b
    uint64_t b = (((uint64_t)((uint32_t)p.x()) << m) | ((uint32_t)p.y())) ^ delta;

    for (int c = 0; b; b >>= 1, ++c)
        if (b & 1)  // Add column 2 * m - c.
            index ^= VdCSobolMatricesInv[m - 1][c];

    return index;
}

}


namespace pt {

inline int exponent(float v) {
    uint32_t v_;
    std::memcpy(&v_, &v, sizeof(uint32_t));
    return (v_ >> 23) - 127;
}

inline int significand(float v) {
    uint32_t v_;
    std::memcpy(&v_, &v, sizeof(uint32_t));
    return v_ & ((1 << 23) - 1);
}

inline int log2int(float v) {
    if (v < 1)
        return -log2int(1 / v);
    // https://graphics.stanford.edu/~seander/bithacks.html#IntegerLog
    // (With an additional check of the significant to get round-to-nearest
    // rather than round down.)
    // midsignif = Significand(std::pow(2., 1.5))
    // i.e. grab the significand of a value halfway between two exponents,
    // in log space.
    const uint32_t midsignif = 0b00000000001101010000010011110011;
    return exponent(v) + ((significand(v) >= midsignif) ? 1 : 0);
}

inline constexpr int32_t roundUpPow2(int32_t v) {
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1;
}

inline constexpr int64_t roundUpPow2(int64_t v) {
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    return v + 1;
}

SobolSampler::SobolSampler(uint32_t spp, Vector2i resolution) : Sampler(spp) {
    m_sobolIndex = 0;
    m_dimension = 0;
    m_scale = roundUpPow2(std::max(resolution.x(), resolution.y()));
    m_pixel = Vector2i();
}

void SobolSampler::startPixelSample(const Vector2i& p, int sampleIndex) {
    m_pixel = p;
    m_dimension = 2;
    m_sobolIndex = sobol::sobolIntervalToIndex(log2int(m_scale), sampleIndex, m_pixel);
}

inline float SobolSampler::sample1D() {
    if (m_dimension >= sobol::NSobolDimensions)
        m_dimension = 2;
    return sampleDimension(m_dimension++);
}

inline Vector2f SobolSampler::sample2D() {
    if (m_dimension + 1 >= sobol::NSobolDimensions)
        m_dimension = 2;
    Vector2f u(sampleDimension(m_dimension), sampleDimension(m_dimension + 1));
    m_dimension += 2;
    return u;
}

inline Vector2f SobolSampler::samplePixel2D() {
    Vector2f u(sampleDimension(0), sampleDimension(1));
    // Remap Sobol\+$'$ dimensions used for pixel samples
    for (int dim = 0; dim < 2; ++dim) {
        u[dim] = clamp(u[dim] * m_scale - m_pixel[dim], 0.0f, sobol::FloatOneMinusEpsilon);
    }
    return u;
}

std::unique_ptr<Sampler> SobolSampler::clone() const {
    std::unique_ptr<SobolSampler> cloned(new SobolSampler(uint32_t(), Vector2i()));
    cloned->m_spp = m_spp;
    cloned->m_sobolIndex = m_sobolIndex;
    cloned->m_dimension = m_dimension;
    cloned->m_scale = m_scale;
    cloned->m_pixel = m_pixel;
    return std::move(cloned);
}

}