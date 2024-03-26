
#include <pt/gui.h>
#include <pt/block.h>
#include <pt/timer.h>
#include <pt/camera.h>
#include <pt/sampler.h>
#include <pt/integrator.h>
#include <pt/scene.h>
#include <pt/bitmap.h>
#include <pt/material.h>
#include <pt/bdpt.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/task_scheduler_init.h>
#include <thread>

using namespace pt;

static bool useGui = true;

std::unique_ptr<Bitmap> writeBitmap(const ImageBlock& sampleBlock, const ImageBlock& splatBlock, float splatScale) {
    cout << "Writing result to bitmap .. ";
    cout.flush();
    Timer timer;

    sampleBlock.lock();
    splatBlock.lock();

    const Vector2i& size = sampleBlock.getSize();
    int bSize = sampleBlock.getBorderSize();
    auto bitmap = std::make_unique<Bitmap>(size);

    tbb::blocked_range2d<int> range(0, size.y(), 0, size.x()); // rows, cols

    auto map = [&](const tbb::blocked_range2d<int>& r) {
        for (int y = r.rows().begin(); y != r.rows().end(); ++y)
            for (int x = r.cols().begin(); x != r.cols().end(); ++x)
                bitmap->coeffRef(y, x) =
                    sampleBlock.coeff(y + bSize, x + bSize).divideByFilterWeight() +
                    splatBlock.coeff(y + bSize, x + bSize).head<3>() * splatScale;
    };

    tbb::parallel_for(range, map);
    cout << "done. (took " << timer.elapsedString() << ")" << endl;
    return bitmap;
}

void renderBlock(Scene* scene, Sampler* sampler, Integrator* integrator, ImageBlock& block) {
    Vector2i offset = block.getOffset();
    Vector2i size = block.getSize();

    block.clear();
    sampler->startBlockSample(offset);

    for (uint32_t y = 0; y < size.y(); ++y) {
        for (uint32_t x = 0; x < size.x(); ++x) {
            for (uint32_t s = 0; s < sampler->getSPP(); s++) {
                Vector2i pixel = Vector2i(x, y) + offset;
                sampler->startPixelSample(pixel, s);
                Vector2f pixelSample = pixel.cast<float>() + sampler->samplePixel2D();

                Vector3f value = integrator->Li(scene, sampler, pixelSample);

                block.addSample(pixelSample, value);
            }
        }
    }
}

std::unique_ptr<Bitmap> render(Scene* scene, Sampler* sampler, Integrator* integrator) {
    Vector2i screenSize = scene->getCamera()->getScreenSize();
    ImageBlock sampleResult(screenSize, scene->getFilter());
    ImageBlock splatResult(screenSize, scene->getFilter());
    float splatScale = 1.0f / sampler->getSPP();

    sampleResult.clear();
    splatResult.clear();

    integrator->setSplatBlock(&splatResult);
    
    GUI *gui = nullptr;
    if (useGui) {
        nanogui::init();
        gui = new GUI(sampleResult, splatResult);
        gui->setSplatScale(splatScale);
    }

    std::thread render_thread([&] {
        cout << "Rendering .. ";
        cout.flush();
        Timer timer;

        BlockGenerator blockGenerator(screenSize, PT_BLOCK_SIZE);
        tbb::blocked_range<int> range(0, blockGenerator.getBlockCount());

        auto map = [&](const tbb::blocked_range<int>& range) {
            ImageBlock block(Vector2i(PT_BLOCK_SIZE), scene->getFilter());

            // Create a clone of the sampler for the current thread
            std::unique_ptr<Sampler> sampler_t(sampler->clone());

            for (int i = range.begin(); i < range.end(); ++i) {
                blockGenerator.next(block);

                renderBlock(scene, sampler_t.get(), integrator, block);

                sampleResult.put(block);
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

    return writeBitmap(sampleResult, splatResult, splatScale);
}

void configuration(Scene* scene, Sampler* sampler, Integrator* integrator) {
    std::cout << tfm::format(
        "Configuration[\n"
        "  integrator = %s,\n"
        "  sampler = %s,\n"
        "  scene = %s\n"
        "]",
        indent(integrator->toString()),
        indent(sampler->toString()),
        indent(scene->toString())
    ) << endl;
}

int main(int argc, char **argv) {
    threadCount = tbb::task_scheduler_init::automatic;
    tbb::task_scheduler_init init(threadCount);

    std::string obj_path = "D:/code/Rendering/Path-Tracer/scenes/veach-mis/veach-mis.obj";
    std::string xml_path = "D:/code/Rendering/Path-Tracer/scenes/veach-mis/veach-mis.xml";

    try {
        // create scene
        Scene scene;
        scene.loadOBJ(obj_path);
        scene.loadXML(xml_path);
        scene.preprocess();

        // create sampler
        uint32_t spp = 128;
        SobolSampler sampler(spp, scene.getCamera()->getScreenSize());

        // create Integrator
        //BDPTIntegrator integrator;
        PathIntegrator integrator;

        // rendering
        configuration(&scene, &sampler, &integrator);
        auto result = render(&scene, &sampler, &integrator);

        // save result
        std::string folder_path = getFolderPath(obj_path);
        result.get()->savePNG(folder_path + "result.png");
        result.get()->saveEXR(folder_path + "result.exr");
    }
    catch (const PathTracerException& e) {
		cerr << "Error: " << e.what() << endl;
		return 1;
	}

    return 0;
}