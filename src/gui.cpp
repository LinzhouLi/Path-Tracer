/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/


#include <pt/gui.h>
#include <pt/block.h>
#include <nanogui/shader.h>
#include <nanogui/label.h>
#include <nanogui/slider.h>
#include <nanogui/layout.h>
#include <nanogui/renderpass.h>
#include <nanogui/texture.h>

namespace pt {

GUI::GUI(const ImageBlock& sampleBlock, const ImageBlock& splatBlock) :
    nanogui::Screen(nanogui::Vector2i(800, 600)),
    m_sampleBlock(sampleBlock),
    m_splatBlock(splatBlock) 
{
    using namespace nanogui;

    inc_ref();
    const Vector2i& size = sampleBlock.getSize();

    /* Add some UI elements to adjust the exposure value */
    Widget* panel = new Widget(this);
    panel->set_layout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 10, 10));
    new Label(panel, "Exposure value: ", "sans-bold");
    Slider *slider = new Slider(panel);
    slider->set_value(0.5f);
    slider->set_fixed_width(150);
    slider->set_callback([&](float value) { m_tonemapScale = std::pow(2.f, (value - 0.5f) * 20); });
    slider->callback();
    panel->set_size(nanogui::Vector2i(size.x(), size.y()));
    perform_layout();
    
    /* Simple gamma tonemapper as a GLSL shader */
    m_renderPass = new RenderPass({ this });
    m_renderPass->set_clear_color(0, Color(0.3f, 0.3f, 0.3f, 1.f));
    
    m_shader = new Shader(
        m_renderPass,
        /* An identifying name */
        "Tonemapper",
        /* Vertex shader */
        R"(#version 330
        uniform ivec2 size;
        uniform int borderSize;

        in vec2 position;
        out vec2 uv;

        void main() {
            gl_Position = vec4(position.x * 2 - 1, position.y * 2 - 1, 0.0, 1.0);

            // Crop away image border (due to pixel filter)
            vec2 total_size = size + 2 * borderSize;
            vec2 scale = size / total_size;
            uv = vec2(position.x * scale.x + borderSize / total_size.x, 1 - (position.y * scale.y + borderSize / total_size.y));
        })",
        /* Fragment shader */
        R"(#version 330
        uniform sampler2D sampleTexture;
        uniform sampler2D splatTexture;
        uniform float tonemapScale;
        uniform float splatScale;

        in vec2 uv;
        out vec4 out_color;

        float toSRGB(float value) {
            if (value < 0.0031308)
                return 12.92 * value;
            return 1.055 * pow(value, 0.41666) - 0.055;
        }

        void main() {
            vec4 sampleColor = texture(sampleTexture, uv);
            vec4 splatColor = texture(splatTexture, uv);
            if (sampleColor.w != 0.0) sampleColor /= sampleColor.w;
            splatColor *= splatScale;
            vec4 color = (sampleColor + splatColor) * tonemapScale;
            out_color = vec4(toSRGB(color.r), toSRGB(color.g), toSRGB(color.b), 1.0f);
        })"
    );
    
    // Draw 2 triangles
    uint32_t indices[3 * 2] = {
        0, 1, 2,
        2, 3, 0
    };
    float positions[2 * 4] = {
        0.f, 0.f,
        1.f, 0.f,
        1.f, 1.f,
        0.f, 1.f
    };

    m_shader->set_buffer("indices", VariableType::UInt32, {3*2}, indices);
    m_shader->set_buffer("position", VariableType::Float32, {4, 2}, positions);
    
    int borderSize = sampleBlock.getBorderSize();
    m_shader->set_uniform("size", nanogui::Vector2i(size.x(), size.y()));
    m_shader->set_uniform("borderSize", borderSize);
    
    // Allocate texture memory for the rendered image
    m_sampleTexture = new Texture(
        Texture::PixelFormat::RGBA,
        Texture::ComponentFormat::Float32,
        nanogui::Vector2i(size.x() + 2 * borderSize, size.y() + 2 * borderSize),
        Texture::InterpolationMode::Nearest,
        Texture::InterpolationMode::Nearest
    );

    m_splatTexture = new Texture(
        Texture::PixelFormat::RGBA,
        Texture::ComponentFormat::Float32,
        nanogui::Vector2i(size.x() + 2 * borderSize, size.y() + 2 * borderSize),
        Texture::InterpolationMode::Nearest,
        Texture::InterpolationMode::Nearest
    );

    draw_all();
    set_visible(true);
    set_size(nanogui::Vector2i(size.x(), (size.y() + 36)));
    set_caption("PathTracer");
    panel->set_position(nanogui::Vector2i((size.x() - panel->size().x()) / 2, size.y()));
}


void GUI::draw_contents() {
    // Reload the partially rendered image onto the GPU
    m_sampleBlock.lock();
    m_splatBlock.lock();

    const Vector2i &size = m_sampleBlock.getSize();
    m_shader->set_uniform("tonemapScale", m_tonemapScale);
    m_shader->set_uniform("splatScale", m_splatScale);
    m_renderPass->resize(framebuffer_size());
    m_renderPass->begin();
    m_renderPass->set_viewport(
        nanogui::Vector2i(0, 0),
        nanogui::Vector2i(m_pixel_ratio * size[0], m_pixel_ratio * size[1])
    );

    m_sampleTexture->upload((uint8_t *)m_sampleBlock.data());
    m_splatTexture->upload((uint8_t*)m_splatBlock.data());
    m_shader->set_texture("sampleTexture", m_sampleTexture);
    m_shader->set_texture("splatTexture", m_splatTexture);

    m_shader->begin();
    m_shader->draw_array(nanogui::Shader::PrimitiveType::Triangle, 0, 6, true);
    m_shader->end();
    // m_renderPass->set_viewport(nanogui::Vector2i(0, 0), framebuffer_size());
    m_renderPass->end();

    m_sampleBlock.unlock();
    m_splatBlock.unlock();
}

}