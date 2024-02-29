
#include <pt/gui.h>
#include <pt/block.h>
#include <pt/timer.h>
#include <pt/camera.h>
#include <pt/sampler.h>
#include <pt/integrator.h>
#include <pt/scene.h>
#include <pt/bitmap.h>
#include <pt/material.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/task_scheduler_init.h>
#include <thread>

using namespace pt;

//static int threadCount = -1;
static bool useGui = true;

//static Bitmap* testBitmap;
//static Vector2i outputSize(768, 768);

static void renderBlock(Scene* scene, Sampler* sampler, ImageBlock& block) {
    Vector2i offset = block.getOffset();
    Vector2i size = block.getSize();

    block.clear();
    for (uint32_t y = 0; y < size.y(); ++y) {
        for (uint32_t x = 0; x < size.x(); ++x) {
            for (uint32_t s = 0; s < sampler->getSPP(); s++) {
                Vector2i pixel = Vector2i(x, y) + offset;
                sampler->startPixelSample(pixel, s);
                Vector2f pixelSample = pixel.cast<float>() + sampler->sample2D();

                Ray ray = scene->getCamera()->sampleRay(pixelSample);
                Color3f value = scene->getIntegrator()->Li(scene, ray);

                block.put(pixelSample, value);
            }
        }
    }
}

static void render(Scene* scene, Sampler* sampler) {
    Vector2i screenSize = scene->getCamera()->getScreenSize();
    BlockGenerator blockGenerator(screenSize, PT_BLOCK_SIZE);
    ImageBlock result(screenSize);
    result.clear();

    GUI *gui = nullptr;
    if (useGui) {
        nanogui::init();
        gui = new GUI(result);
    }

    std::thread render_thread([&] {
        tbb::task_scheduler_init init(threadCount);
        cout << "Rendering .. ";
        cout.flush();
        Timer timer;

        tbb::blocked_range<int> range(0, blockGenerator.getBlockCount());

        auto map = [&](const tbb::blocked_range<int>& range) {
            ImageBlock block(Vector2i(PT_BLOCK_SIZE));

            /// Create a clone of the sampler for the current thread
            std::unique_ptr<Sampler> sampler_t(sampler->clone());

            for (int i = range.begin(); i < range.end(); ++i) {
                blockGenerator.next(block);

                renderBlock(scene, sampler_t.get(), block);

                result.put(block);
            }
        };

        tbb::parallel_for(range, map);
        cout << "done. (took " << timer.elapsedString() << ")" << endl;
    });

    if (useGui) nanogui::mainloop(50.f);

    render_thread.join();

    if (useGui) {
        delete gui;
        nanogui::shutdown();
    }

}

int main(int argc, char **argv) {
    threadCount = tbb::task_scheduler_init::automatic;

    Scene scene;
    scene.loadOBJ("D:/code/Rendering/Path-Tracer/scenes/veach-mis/veach-mis.obj");
    scene.loadXML("D:/code/Rendering/Path-Tracer/scenes/veach-mis/veach-mis.xml");
    scene.preprocess();

    SobolSampler sampler(32);

    render(&scene, &sampler);

    return 0;
}