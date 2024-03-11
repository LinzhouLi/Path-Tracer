#pragma once

#include <pt/common.h>
#include <nanogui/screen.h>

namespace pt {

class GUI : public nanogui::Screen {
public:
    GUI(const ImageBlock & sampleBlock, const ImageBlock& splatBlock);

    void setSplatScale(float s) { m_splatScale = s; }

    void draw_contents() override;

private:
    const ImageBlock& m_sampleBlock;
    const ImageBlock& m_splatBlock;

    nanogui::ref<nanogui::Shader> m_shader;
    nanogui::ref<nanogui::Texture> m_sampleTexture;
    nanogui::ref<nanogui::Texture> m_splatTexture;
    nanogui::ref<nanogui::RenderPass> m_renderPass;

    float m_tonemapScale = 1.0f;
    float m_splatScale = 1.0f;
};

}