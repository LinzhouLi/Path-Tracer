
#include <pt/gui.h>
#include <pt/block.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/task_scheduler_init.h>
#include <thread>

using namespace pt;

static int threadCount = -1;
static bool useGui = true;

static void renderBlock() {

}

static void render() {
    Vector2i outputSize(768, 768);
    BlockGenerator blockGenerator(outputSize, NORI_BLOCK_SIZE);
    ImageBlock result(outputSize);

    GUI *gui = nullptr;
    if (useGui) {
        nanogui::init();
        gui = new GUI(result);
    }

    

}

int main(int argc, char **argv) {
    threadCount = tbb::task_scheduler_init::automatic;

    render();

    return 0;
}