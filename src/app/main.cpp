#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "app.h"
#include "scenes.h"

namespace {

void usage() {
    std::printf(
        "Particle Simulator\n\n"
        "usage: psim [options]\n"
        "  --scene N           start with scene N (0-based):\n");
    for (size_t i = 0; i < sceneList().size(); i++) std::printf("                        %2zu  %s\n", i, sceneList()[i].name);
    std::printf(
        "  --size WxH          window size (default 1600x900)\n"
        "  --threads N         worker threads (default: all cores)\n"
        "  --record FILE.mp4   render a fixed 60 fps timeline to a video (needs ffmpeg)\n"
        "  --record-frames N   stop after N frames when recording\n"
        "  --caption-file F    show the contents of F as a caption (re-read every frame)\n"
        "  --status-file F     write the current frame number to F every frame\n"
        "  --gpu-selftest      run the GPU path headless, compare with expectations, exit\n");
}

}  // namespace

int main(int argc, char** argv) {
    AppOptions opt;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--scene") opt.scene = std::atoi(next());
        else if (a == "--size") {
            if (std::sscanf(next(), "%dx%d", &opt.width, &opt.height) != 2) {
                std::fprintf(stderr, "--size expects WxH\n");
                return 2;
            }
        } else if (a == "--threads") opt.threads = std::atoi(next());
        else if (a == "--record") opt.recordPath = next();
        else if (a == "--record-frames") opt.recordFrames = std::atoi(next());
        else if (a == "--caption-file") opt.captionFile = next();
        else if (a == "--status-file") opt.statusFile = next();
        else if (a == "--gpu-selftest") opt.gpuSelfTest = true;
        else if (a == "--help" || a == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "unknown option %s\n\n", a.c_str());
            usage();
            return 2;
        }
    }
    App app(opt);
    return app.run();
}
