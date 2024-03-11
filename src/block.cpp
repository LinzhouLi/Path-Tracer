/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <pt/block.h>
#include <pt/bitmap.h>
#include <pt/filter.h>
#include <tbb/tbb.h>

namespace pt {

ImageBlock::ImageBlock(const Vector2i &size, const Filter* filter) : m_offset(0, 0), m_size(size) {
    if (filter) {
        /* Tabulate the image reconstruction filter for performance reasons */
        m_filterRadius = filter->getRadius();
        m_borderSize = (int)std::ceil(m_filterRadius - 0.5f);
        m_filter = new float[PT_FILTER_RESOLUTION + 1];
        for (int i = 0; i < PT_FILTER_RESOLUTION; ++i) {
            float pos = (m_filterRadius * i) / PT_FILTER_RESOLUTION;
            m_filter[i] = filter->eval(pos);
        }
        m_filter[PT_FILTER_RESOLUTION] = 0.0f;
        m_lookupFactor = PT_FILTER_RESOLUTION / m_filterRadius;
        int weightSize = (int)std::ceil(2 * m_filterRadius) + 1;
        m_weightsX = new float[weightSize];
        m_weightsY = new float[weightSize];
        memset(m_weightsX, 0, sizeof(float) * weightSize);
        memset(m_weightsY, 0, sizeof(float) * weightSize);
    }

    /* Allocate space for pixels and border regions */
    resize(size.y() + 2 * m_borderSize, size.x() + 2 * m_borderSize);
}

ImageBlock::~ImageBlock() {
    delete[] m_filter;
    delete[] m_weightsX;
    delete[] m_weightsY;
}

Bitmap *ImageBlock::toBitmap() const {
    Bitmap *result = new Bitmap(m_size);
    for (int y=0; y<m_size.y(); ++y)
        for (int x=0; x<m_size.x(); ++x)
            result->coeffRef(y, x) = coeff(y + m_borderSize, x + m_borderSize).divideByFilterWeight();
    return result;
}

void ImageBlock::fromBitmap(const Bitmap &bitmap) {
    if (bitmap.cols() != cols() || bitmap.rows() != rows())
        throw PathTracerException("Invalid bitmap dimensions!");

    for (int y=0; y<m_size.y(); ++y)
        for (int x=0; x<m_size.x(); ++x)
            coeffRef(y, x) << bitmap.coeff(y, x), 1;
}

void ImageBlock::put(const Vector2f &globalPos, const Color3f &value, float weight) {
    if (!value.isValid()) {
        /* If this happens, go fix your code instead of removing this warning ;) */
        cerr << "Integrator: computed an invalid radiance value: " << value.toString() << endl;
        return;
    }

    Vector2f localPos = globalPos - m_offset.cast<float>();
    if (
        localPos.x() < 0.0f || localPos.x() >= m_size.x() ||
        localPos.y() < 0.0f || localPos.y() >= m_size.y()
    ) return;

    localPos += Vector2f(m_borderSize);
    //coeffRef(localPos.y(), localPos.x()) += Color4f(value, weight);

    /* Compute the rectangle of pixels that will need to be updated */
    int boundMinX = std::max(int(std::ceil(localPos.x() - m_filterRadius)), 0);
    int boundMinY = std::max(int(std::ceil(localPos.y() - m_filterRadius)), 0);
    int boundMaxX = std::min(int(std::floor(localPos.x() + m_filterRadius)), int(cols() - 1));
    int boundMaxY = std::min(int(std::floor(localPos.y() + m_filterRadius)), int(rows() - 1));

    /* Lookup values from the pre-rasterized filter */
    for (int x = boundMinX, idx = 0; x <= boundMaxX; ++x, ++idx)
        m_weightsX[idx] = m_filter[(int)(std::abs(x - localPos.x()) * m_lookupFactor)];
    for (int y = boundMinY, idx = 0; y <= boundMaxY; ++y, ++idx)
        m_weightsY[idx] = m_filter[(int)(std::abs(y - localPos.y()) * m_lookupFactor)];

    for (int y = boundMinY, yr = 0; y <= boundMaxY; ++y, ++yr)
        for (int x = boundMinX, xr = 0; x <= boundMaxX; ++x, ++xr)
            coeffRef(y, x) += Color4f(value, weight) * m_weightsX[xr] * m_weightsY[yr];
}

void ImageBlock::addSample(const Vector2f& globalPos, const Vector3f& value) {
    put(globalPos, Color3f(value.x(), value.y(), value.z()), 1.0f);
}

void ImageBlock::addSplat(const Vector2f& globalPos, const Vector3f& value) {
    tbb::mutex::scoped_lock lock(m_mutex);
    put(globalPos, Color3f(value.x(), value.y(), value.z()), 0.0f);
}
    
void ImageBlock::put(ImageBlock &b) {
    Vector2i offset = b.getOffset() - m_offset +
        Vector2i::Constant(m_borderSize - b.getBorderSize());
    Vector2i size   = b.getSize()   + Vector2i(2*b.getBorderSize());

    tbb::mutex::scoped_lock lock(m_mutex);

    block(offset.y(), offset.x(), size.y(), size.x()) += b.topLeftCorner(size.y(), size.x());
}

std::string ImageBlock::toString() const {
    return tfm::format("ImageBlock[offset=%s, size=%s]]", m_offset.toString(), m_size.toString());
}

BlockGenerator::BlockGenerator(const Vector2i &size, int blockSize) : m_size(size), m_blockSize(blockSize) {
    m_numBlocks = Vector2i(
        (int) std::ceil(size.x() / (float) blockSize),
        (int) std::ceil(size.y() / (float) blockSize)
    );
    m_blocksLeft = m_numBlocks.x() * m_numBlocks.y();
    m_direction = ERight;
    m_block = Vector2i((m_numBlocks - Vector2i(1)) / 2);
    m_stepsLeft = 1;
    m_numSteps = 1;
}

bool BlockGenerator::next(ImageBlock &block) {
    tbb::mutex::scoped_lock lock(m_mutex);

    if (m_blocksLeft == 0)
        return false;

    Vector2i pos = m_block * m_blockSize;
    block.setOffset(pos);
    block.setSize((m_size - pos).cwiseMin(Vector2i::Constant(m_blockSize)));

    if (--m_blocksLeft == 0)
        return true;

    do {
        switch (m_direction) {
            case ERight: ++m_block.x(); break;
            case EDown:  ++m_block.y(); break;
            case ELeft:  --m_block.x(); break;
            case EUp:    --m_block.y(); break;
        }

        if (--m_stepsLeft == 0) {
            m_direction = (m_direction + 1) % 4;
            if (m_direction == ELeft || m_direction == ERight) 
                ++m_numSteps;
            m_stepsLeft = m_numSteps;
        }
    } while ((m_block.array() < 0).any() || (m_block.array() >= m_numBlocks.array()).any());

    return true;
}

}
