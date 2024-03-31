
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
#include <pt/bdpt2.h>

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range2d.h>
#include <tbb/task_scheduler_init.h>
#include <thread>

using namespace pt;

std::unique_ptr<Bitmap> writeBitmap(ImageBlock* sampleBlock, ImageBlock* splatBlock = nullptr, float splatScale = 1.0) {
    std::cout << "Writing result to bitmap .. ";
    std::cout.flush();
    Timer timer;

    sampleBlock->lock();
    if (splatBlock) splatBlock->lock();

    const Vector2i& size = sampleBlock->getSize();
    int bSize = sampleBlock->getBorderSize();
    auto bitmap = std::make_unique<Bitmap>(size);

    tbb::blocked_range2d<int> range(0, size.y(), 0, size.x()); // rows, cols

    auto map = [&](const tbb::blocked_range2d<int>& r) {
        for (int y = r.rows().begin(); y != r.rows().end(); ++y)
            for (int x = r.cols().begin(); x != r.cols().end(); ++x) {
                auto tmp = sampleBlock->coeff(y + bSize, x + bSize).divideByFilterWeight();
                if (splatBlock) 
                    tmp += splatBlock->coeff(y + bSize, x + bSize).head<3>() * splatScale;
                bitmap->coeffRef(y, x) = tmp;
            }
    };

    tbb::parallel_for(range, map);
    std::cout << "done. (took " << timer.elapsedString() << ")" << endl;

    sampleBlock->unlock();
    if (splatBlock) splatBlock->unlock();
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

void render(Scene* scene, Sampler* sampler, Integrator* integrator, ImageBlock* result) {
    Vector2i screenSize = scene->getCamera()->getScreenSize();

    BlockGenerator blockGenerator(screenSize, PT_BLOCK_SIZE);
    tbb::blocked_range<int> range(0, blockGenerator.getBlockCount());

    auto map = [&](const tbb::blocked_range<int>& range) {
        ImageBlock block(Vector2i(PT_BLOCK_SIZE), scene->getFilter());

        // Create a clone of the sampler for the current thread
        std::unique_ptr<Sampler> sampler_t(sampler->clone());

        for (int i = range.begin(); i < range.end(); ++i) {
            blockGenerator.next(block);

            renderBlock(scene, sampler_t.get(), integrator, block);

            result->put(block);
        }
    };

    tbb::parallel_for(range, map);
}

int main(int argc, char **argv) {

    // default settings
    int threadCount = tbb::task_scheduler_init::automatic;
    std::string sceneName = "cornell-box"; // bathroom, cornell-box, library, veach-mis
    uint32_t spp = 256;
    bool useGui = true;
    bool useBDPT = false;

    // parsing arguments
    for (int i = 1; i < argc; ++i) {
        std::string token(argv[i]);
        if (token == "-t" || token == "--threads") {
            if (i + 1 >= argc) {
                cerr << "\"--threads\" argument expects a positive integer following it." << endl;
                return -1;
            }
            threadCount = atoi(argv[i + 1]);
            i++;
            if (threadCount <= 0) {
                cerr << "\"--threads\" argument expects a positive integer following it." << endl;
                return -1;
            }
            continue;
        }
        else if (token == "-s" || token == "--spp") {
            if (i + 1 >= argc) {
                cerr << "\"--spp\" argument expects a positive integer following it." << endl;
                return -1;
            }
            spp = atoi(argv[i + 1]);
            i++;
            if (spp <= 0) {
                cerr << "\"--spp\" argument expects a positive integer following it." << endl;
                return -1;
            }
            continue;
        }
        else if (token == "--no-gui") {
            useGui = false;
            continue;
        }
        else if (token == "--bdpt") {
            useBDPT = true;
            continue;
        }

        if (token == "bathroom" || token == "cornell-box" || token == "library" || token == "veach-mis") {
            sceneName = token;
        }
        else {
			cerr << "Unknown argument: " << token << endl;
			return -1;
		}
    }

    // apply settings
    tbb::task_scheduler_init init(threadCount);
    std::string obj_path = tfm::format("./scenes/%s/%s.obj", sceneName, sceneName);
    std::string xml_path = tfm::format("./scenes/%s/%s.xml", sceneName, sceneName);
    std::string folder_path = getFolderPath(obj_path);

    try {
        // create scene
        Scene scene;
        scene.loadOBJ(obj_path);
        scene.loadXML(xml_path);
        scene.preprocess();
        std::cout << scene.toString() << std::endl;

        // result block
        Vector2i screenSize = scene.getCamera()->getScreenSize();
        ImageBlock sampleResult(screenSize, scene.getFilter());
        ImageBlock splatResult(screenSize, scene.getFilter());
        float splatScale = 1.0f / spp;

        // gui
        GUI* gui = nullptr;
        if (useGui) {
            nanogui::init();
            gui = new GUI(sampleResult, splatResult);
            gui->setSplatScale(splatScale);
        }

        // rendering albedo map
        std::thread render_thread([&] {
            {
                std::cout << "Rendering albedo map .. ";
                std::cout.flush();
                Timer timer;

                BaseColorIntegrator integrator;
                SobolSampler sampler(32, screenSize);

                sampleResult.clear();
                splatResult.clear();
                render(&scene, &sampler, &integrator, &sampleResult);
                std::cout << "done. (took " << timer.elapsedString() << ")" << endl;

                auto result = writeBitmap(&sampleResult);
                result.get()->savePNG(folder_path + "albedo.png");
                result.get()->saveEXR(folder_path + "albedo.exr");
            }

            // rendering normal map
            {
                std::cout << "Rendering normal map .. ";
                std::cout.flush();
                Timer timer;

                GeometryIntegrator integrator;
                SobolSampler sampler(32, screenSize);

                sampleResult.clear();
                splatResult.clear();
                render(&scene, &sampler, &integrator, &sampleResult);
                std::cout << "done. (took " << timer.elapsedString() << ")" << endl;

                auto result = writeBitmap(&sampleResult);
                result.get()->savePNG(folder_path + "normal.png", false);
                result.get()->saveEXR(folder_path + "normal.exr");
            }

            // rendering
            {
                std::cout << "Rendering .. ";
                std::cout.flush();
                Timer timer;

                Integrator* integrator;
                if (useBDPT) { 
                    integrator = new BDPTIntegrator2(); // a wrong BDPT integrator
                    integrator->setSplatBlock(&splatResult);
                }
                else {
                    integrator = new PathIntegrator();
                }
                SobolSampler sampler(spp, screenSize);
                //IndependentSampler sampler(spp);

                sampleResult.clear();
                splatResult.clear();
                render(&scene, &sampler, integrator, &sampleResult);
                delete integrator;

                auto result = writeBitmap(&sampleResult, &splatResult, splatScale);
                result.get()->savePNG(folder_path + "result.png");
                result.get()->saveEXR(folder_path + "result.exr");
                std::cout << "done. (took " << timer.elapsedString() << ")" << endl;
            }
        });

        if (useGui) nanogui::mainloop(50.f);

        render_thread.join();

        if (useGui) {
            delete gui;
            nanogui::shutdown();
        }
    }
    catch (const PathTracerException& e) {
		cerr << "Error: " << e.what() << endl;
		return 1;
	}

    return 0;
}