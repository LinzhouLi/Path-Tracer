
#include <pt/gui.h>
#include <pt/block.h>
#include <pt/timer.h>
#include <pt/bitmap.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/task_scheduler_init.h>
#include <thread>

using namespace pt;

static int threadCount = -1;
static bool useGui = true;

static Bitmap testBitmap;
static Vector2i outputSize(768, 768);

static void renderBlock(ImageBlock& block) {
    Point2i offset = block.getOffset();
    Vector2i size = block.getSize();

    block.clear();
    for (int y = 0; y < size.y(); ++y) {
        for (int x = 0; x < size.x(); ++x) {
            Point2f pixelSample = Point2f((float)(x + offset.x()) + 0.5, (float)(y + offset.y()) + 0.5);
            //block.put(pixelSample, Color3f(0, 1, 0));
            Point2f uv = Point2f(
                pixelSample.x() / (float)outputSize.x(),
                pixelSample.y() / (float)outputSize.y()
            );
            block.put(pixelSample, testBitmap.sample(uv));
        }
    }
}

static void render() {
    BlockGenerator blockGenerator(outputSize, PT_BLOCK_SIZE);
    ImageBlock result(outputSize);
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

            /// sampler

            for (int i = range.begin(); i < range.end(); ++i) {
                blockGenerator.next(block);

                /// sampler

                renderBlock(block);

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
    testBitmap.load("D:/dataset/youtube/frames/0ORaAnJYROg/0001.jpg");

    render();

    return 0;
}