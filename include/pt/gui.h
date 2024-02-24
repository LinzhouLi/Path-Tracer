/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <pt/common.h>
#include <nanogui/screen.h>

namespace pt {

class GUI : public nanogui::Screen {
public:
    GUI(const ImageBlock &block);
    void draw_contents() override;
private:
    const ImageBlock &m_block;
    nanogui::ref<nanogui::Shader> m_shader;
    nanogui::ref<nanogui::Texture> m_texture;
    nanogui::ref<nanogui::RenderPass> m_renderPass;
    float m_scale = 1.f;
};

}