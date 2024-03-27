/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#include <pt/bitmap.h>
#include <ImfInputFile.h>
#include <ImfOutputFile.h>
#include <ImfChannelList.h>
#include <ImfStringAttribute.h>
#include <ImfVersion.h>
#include <ImfIO.h>

// STB_IMAGE_IMPLEMENTATION is already been define in an external project.
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

namespace pt {

void Bitmap::loadEXR(const std::string &filename) {
    Imf::InputFile file(filename.c_str());
    const Imf::Header &header = file.header();
    const Imf::ChannelList &channels = header.channels();

    Imath::Box2i dw = file.header().dataWindow();
    resize(dw.max.y - dw.min.y + 1, dw.max.x - dw.min.x + 1);

    //cout << "Reading a " << cols() << "x" << rows() << " OpenEXR file from \""
    //     << filename << "\"" << endl;

    const char *ch_r = nullptr, *ch_g = nullptr, *ch_b = nullptr;
    for (Imf::ChannelList::ConstIterator it = channels.begin(); it != channels.end(); ++it) {
        std::string name = toLower(it.name());

        if (it.channel().xSampling != 1 || it.channel().ySampling != 1) {
            /* Sub-sampled layers are not supported */
            continue;
        }

        if (!ch_r && (name == "r" || name == "red" || endsWith(name, ".r") || endsWith(name, ".red"))) {
            ch_r = it.name();
        } else if (!ch_g && (name == "g" || name == "green" || endsWith(name, ".g") || endsWith(name, ".green"))) {
            ch_g = it.name();
        } else if (!ch_b && (name == "b" || name == "blue" || endsWith(name, ".b") || endsWith(name, ".blue"))) {
            ch_b = it.name();
        }
    }

    if (!ch_r || !ch_g || !ch_b)
        throw PathTracerException("This is not a standard RGB OpenEXR file!");

    size_t compStride = sizeof(float),
           pixelStride = 3 * compStride,
           rowStride = pixelStride * cols();

    char *ptr = reinterpret_cast<char *>(data());

    Imf::FrameBuffer frameBuffer;
    frameBuffer.insert(ch_r, Imf::Slice(Imf::FLOAT, ptr, pixelStride, rowStride)); ptr += compStride;
    frameBuffer.insert(ch_g, Imf::Slice(Imf::FLOAT, ptr, pixelStride, rowStride)); ptr += compStride;
    frameBuffer.insert(ch_b, Imf::Slice(Imf::FLOAT, ptr, pixelStride, rowStride));
    file.setFrameBuffer(frameBuffer);
    file.readPixels(dw.min.y, dw.max.y);
}

void Bitmap::load(const std::string &filename) {
    //cout << "Reading an image from \"" << filename << "\" ..." << endl;

    int width, height, channel;
    uint8_t* rgb8 = stbi_load(filename.c_str(), &width, &height, &channel, 0);

    if (rgb8 == nullptr)
        throw PathTracerException(("Fail to load image file: \"" + filename + "\"!").c_str());

    resize(height, width);

    uint8_t* inp = rgb8;
    for (uint32_t i = 0; i < rows(); ++i) {
        for (uint32_t j = 0; j < cols(); ++j) {
            Color3f pixel(
                (float)(inp[0]) / 255.f,
                (float)(inp[1]) / 255.f,
                (float)(inp[2]) / 255.f
            );
            coeffRef(i, j) << pixel.toLinearRGB();
            inp += channel;
        }
    }

    stbi_image_free(rgb8);
}

void Bitmap::saveEXR(const std::string & path) {
    cout << "Writing a " << cols() << "x" << rows()
         << " OpenEXR file to \"" << path << "\"" << endl;

    Imf::Header header((int) cols(), (int) rows());
    header.insert("comments", Imf::StringAttribute("Generated by PathTracer"));

    Imf::ChannelList &channels = header.channels();
    channels.insert("R", Imf::Channel(Imf::FLOAT));
    channels.insert("G", Imf::Channel(Imf::FLOAT));
    channels.insert("B", Imf::Channel(Imf::FLOAT));

    Imf::FrameBuffer frameBuffer;
    size_t compStride = sizeof(float),
           pixelStride = 3 * compStride,
           rowStride = pixelStride * cols();

    char *ptr = reinterpret_cast<char *>(data());
    frameBuffer.insert("R", Imf::Slice(Imf::FLOAT, ptr, pixelStride, rowStride)); ptr += compStride;
    frameBuffer.insert("G", Imf::Slice(Imf::FLOAT, ptr, pixelStride, rowStride)); ptr += compStride;
    frameBuffer.insert("B", Imf::Slice(Imf::FLOAT, ptr, pixelStride, rowStride));

    Imf::OutputFile file(path.c_str(), header);
    file.setFrameBuffer(frameBuffer);
    file.writePixels((int) rows());
}

void Bitmap::savePNG(const std::string & path, bool tonemap) {
    cout << "Writing a " << cols() << "x" << rows()
         << " PNG file to \"" << path << "\"" << endl;

    uint8_t *rgb8 = new uint8_t[3 * cols() * rows()];
    uint8_t *dst = rgb8;
    for (uint32_t i = 0; i < rows(); ++i) {
        for (uint32_t j = 0; j < cols(); ++j) {
            Color3f tonemapped = tonemap ? coeffRef(i, j).toSRGB() : coeffRef(i, j);
            dst[0] = (uint8_t) std::clamp(255.f * tonemapped[0], 0.f, 255.f);
            dst[1] = (uint8_t) std::clamp(255.f * tonemapped[1], 0.f, 255.f);
            dst[2] = (uint8_t) std::clamp(255.f * tonemapped[2], 0.f, 255.f);
            dst += 3;
        }
    }

    int ret = stbi_write_png(path.c_str(), (int) cols(), (int) rows(), 3, rgb8, 3 * (int) cols());
    if (ret == 0) {
        cout << "Bitmap::savePNG(): Could not save PNG file \"" << path << "%s\"" << endl;
    }

    delete[] rgb8;
}

Color3f Bitmap::sample(const Vector2f& uv) const {
    // simple interpolation
    //int x = std::clamp(int(uv.x() * cols()), 0, cols() - 1),
    //    y = std::clamp(int(uv.y() * rows()), 0, rows() - 1);
    //return coeff(y, x);

    // bilinear interpolation
    float u = std::clamp(uv.x(), 0.0f, 1.0f);
    float v = std::clamp(uv.y(), 0.0f, 1.0f);
    float s = std::max(u * cols() - 0.5f, 0.0f);
    float t = std::max(v * rows() - 0.5f, 0.0f);
    int x0 = s;
    int y0 = t;
    int x1 = std::min(x0 + 1, int(cols() - 1));
    int y1 = std::min(y0 + 1, int(rows() - 1));
    float alpha = s - x0;
    float beta = t - y0;
    Color3f a = mix(coeff(y0, x0), coeff(y0, x1), alpha);
    Color3f b = mix(coeff(y1, x0), coeff(y1, x1), alpha);
    return mix(a, b, beta);
}

}
