/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <pt/color.h>
#include <pt/block.h>
#include <pt/bitmap.h>
#include <tbb/tbb.h>

namespace pt {

ImageBlock::ImageBlock(const Vector2i &size) : m_offset(0, 0), m_size(size) {
    /* Allocate space for pixels and border regions */
    resize(size.y() + 2*m_borderSize, size.x() + 2*m_borderSize);
}

ImageBlock::~ImageBlock() { }

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

void ImageBlock::put(const Vector2f &pos_f, const Color3f &value) {
    if (!value.isValid()) {
        /* If this happens, go fix your code instead of removing this warning ;) */
        cerr << "Integrator: computed an invalid radiance value: " << value.toString() << endl;
        return;
    }

    Vector2i pos = pos_f.cast<int>() - m_offset - Vector2i(m_borderSize);
    int x = clamp(pos.x(), 0, m_size.x() - 1); // some numerical error without clamp maximum value
    int y = clamp(pos.y(), 0, m_size.y() - 1);
    coeffRef(y, x) += Color4f(value);
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
    m_block = Vector2i(m_numBlocks / 2);
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
