#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <fstream>
#include <sstream>
#include <map>
#include <array>
#include <vector>
#include <csignal>
#include <atomic>
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <cerrno>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <fcntl.h>

/* Third party include */
#include <libcamera/libcamera.h>

/* Project include (optional) */
#if __has_include(<errlog/err_error.h>)
#include <errlog/err_error.h>   // if ARCK error logging is available
#endif

using namespace libcamera;
using namespace std::chrono_literals;

/* ----------------------------------------------------------------------------
 * Global declarations
 * ----------------------------------------------------------------------------
 */

static int gSV_FrameCounter = 0;
static std::shared_ptr<Camera> gSV_Camera;
static std::atomic<bool> gSV_StopRequested{false};
static constexpr int gSV_RecentImageSlots = 50;
static std::string gSV_AckFile = "/tmp/coords_ack.txt";
static pid_t gSV_AnglePid = -1;
static int gSV_AngleStdinFd = -1;

/* ----------------------------------------------------------------------------
 * Function prototypes
 * ----------------------------------------------------------------------------
 */

std::map<std::string, std::string> SV_LoadConfig(const std::string &filename);
std::string SV_SaveFrameRaw(Request *request, FrameBuffer *buffer, int frameNumber);
static void SV_RequestComplete(Request *request);
static void SV_ProcessFrame(const std::string &rawFile);
static void SV_HandleSignal(int signum);
static pid_t SV_StartAngleProcess();

/* ----------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------
 */

/* ----------------------------------------------------------------------------
 * Function : SV_LoadConfig
 * Description : Parse simple INI key=value configuration file
 * ----------------------------------------------------------------------------
 */
std::map<std::string, std::string> SV_LoadConfig(const std::string &filename)
{
    std::map<std::string, std::string> config;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == ';' || line[0] == '#')
            continue;
        size_t eq = line.find('=');
        if (eq == std::string::npos)
            continue;

        std::string key = line.substr(0, eq);
        std::string value = line.substr(eq + 1);

        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);

        config[key] = value;
    }
    return config;
}

/* ----------------------------------------------------------------------------
 * Function : SV_SaveFrameRaw
 * Description : Save raw buffer and metadata to files
 * ----------------------------------------------------------------------------
 */
std::string SV_SaveFrameRaw(Request *request, FrameBuffer *buffer, int frameNumber)
{
    std::string outDir = "/home/raspberrypi/captures/";
    mkdir(outDir.c_str(), 0777);

    const FrameBuffer::Plane &plane = buffer->planes()[0];
    size_t length = buffer->metadata().planes()[0].bytesused;
    void *mem = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (mem == MAP_FAILED) {
        perror("mmap");
        return std::string();
    }

    const int slot = frameNumber % gSV_RecentImageSlots;
    std::ostringstream base;
    base << outDir << "recent_slot_" << std::setw(2) << std::setfill('0') << slot;
    std::string rawFile = base.str() + ".raw";
    std::string metaFile = base.str() + "_metadata.txt";

    // Save RAW
    std::ofstream file(rawFile, std::ios::binary);
    file.write(static_cast<char*>(mem), length);
    file.close();
    munmap(mem, plane.length);
    std::cout << "Saved RAW: " << rawFile << " (" << length << " bytes)" << std::endl;

    // Save metadata
    std::ofstream meta(metaFile);
    const FrameMetadata &metadata = buffer->metadata();
    meta << "Frame " << frameNumber << " Metadata\n"
         << "--------------------------------------\n";
    meta << "RingSlot: " << slot << " / " << gSV_RecentImageSlots << "\n";
    meta << "Sequence: " << metadata.sequence << "\n";
    meta << "Timestamp: " << metadata.timestamp << "\n";

    const ControlList &md = request->metadata();
    if (!md.empty()) {
        if (md.contains(controls::ExposureTime.id()))
            meta << "ExposureTime (us): " << md.get(controls::ExposureTime).value_or(0) << "\n";
        if (md.contains(controls::AnalogueGain.id()))
            meta << "AnalogueGain: " << md.get(controls::AnalogueGain).value_or(0.0f) << "\n";
        if (md.contains(controls::DigitalGain.id()))
            meta << "DigitalGain: " << md.get(controls::DigitalGain).value_or(0.0f) << "\n";
    }
    meta.close();
    std::cout << "Metadata saved: " << metaFile << std::endl;
    return rawFile;
}

static void SV_ProcessFrame(const std::string &rawFile)
{
    if (rawFile.empty())
        return;

    const char *scriptEnv = std::getenv("SIRRAH_LED_SCRIPT");
    const char *pythonEnv = std::getenv("SIRRAH_PYTHON");

    const std::string script = scriptEnv ? scriptEnv : "/usr/share/sirrah_camera/led_centroid.py";
    const std::string python = pythonEnv ? pythonEnv : "python3";

    std::ostringstream cmd;
    cmd << python << " " << script
        << " --input '" << rawFile << "'"
        << " --coords-stdout --quiet"
        << " --save-debug"
        << " --debug-dir '/home/raspberrypi/captures/led_debug'";

    FILE *pipe = popen(cmd.str().c_str(), "r");
    if (!pipe) {
        std::cerr << "LED processing failed for " << rawFile << " (popen error)\n";
        if (gSV_AngleStdinFd >= 0) {
            const long long tsMs = static_cast<long long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
            std::ostringstream out;
            out << tsMs << ",None\n";
            const std::string payload = out.str();
            (void)write(gSV_AngleStdinFd, payload.c_str(), payload.size());
        }
        return;
    }

    char buffer[256] = {};
    std::string line;
    if (std::fgets(buffer, sizeof(buffer), pipe)) {
        line = buffer;
    }
    const int rc = pclose(pipe);
    if (rc != 0) {
        std::cerr << "LED processing failed for " << rawFile << " (rc=" << rc << ")\n";
        if (gSV_AngleStdinFd >= 0) {
            const long long tsMs = static_cast<long long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
            std::ostringstream out;
            out << tsMs << ",None\n";
            const std::string payload = out.str();
            (void)write(gSV_AngleStdinFd, payload.c_str(), payload.size());
        }
        return;
    }

    if (line.empty()) {
        if (gSV_AngleStdinFd >= 0) {
            const long long tsMs = static_cast<long long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
            std::ostringstream out;
            out << tsMs << ",None\n";
            const std::string payload = out.str();
            (void)write(gSV_AngleStdinFd, payload.c_str(), payload.size());
        }
        return;
    }

    std::string parsed = line;
    for (char &ch : parsed) {
        if (ch == ',')
            ch = ' ';
    }

    std::istringstream iss(parsed);
    std::vector<float> values;
    float v = 0.0f;
    while (iss >> v) {
        values.push_back(v);
    }
    if (values.size() < 3U || (values.size() % 2U) == 0U) {
        std::cerr << "LED processing output malformed for " << rawFile << "\n";
        if (gSV_AngleStdinFd >= 0) {
            const long long tsMs = static_cast<long long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now().time_since_epoch())
                    .count());
            std::ostringstream out;
            out << tsMs << ",None\n";
            const std::string payload = out.str();
            (void)write(gSV_AngleStdinFd, payload.c_str(), payload.size());
        }
        return;
    }
    const long long tsMs = static_cast<long long>(values[0]);

    if (gSV_AngleStdinFd >= 0) {
        std::string payload = line;
        while (!payload.empty() && (payload.back() == '\n' || payload.back() == '\r')) {
            payload.pop_back();
        }
        payload.push_back('\n');
        const ssize_t written = write(gSV_AngleStdinFd, payload.c_str(), payload.size());
        if (written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::cerr << "Angle pipe busy, dropping coordinates for ts=" << tsMs << "\n";
            } else {
                std::perror("write");
            }
        }
    }

}

/* ----------------------------------------------------------------------------
 * Function : SV_RequestComplete
 * Description : Callback executed when a frame capture is complete
 * ----------------------------------------------------------------------------
 */
static void SV_RequestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;

    if (gSV_StopRequested.load())
        return;

    for (auto &pair : request->buffers()) {
        const std::string rawFile = SV_SaveFrameRaw(request, pair.second, gSV_FrameCounter++);
        SV_ProcessFrame(rawFile);
    }

    request->reuse(Request::ReuseBuffers);
    gSV_Camera->queueRequest(request);
}

static void SV_HandleSignal(int signum)
{
    (void)signum;
    gSV_StopRequested.store(true);
}

static pid_t SV_StartAngleProcess()
{
    const char *angleEnv = std::getenv("SIRRAH_ANGLE_EXE");
    const std::string angleExe = angleEnv ? angleEnv : "/usr/bin/sirrah_demo";
    if (access(angleExe.c_str(), X_OK) != 0) {
        std::cerr << "Angle executable not found: " << angleExe << "\n";
        return -1;
    }

    int pipefd[2] = {-1, -1};
    if (pipe(pipefd) != 0) {
        std::perror("pipe");
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0) {
        std::perror("fork");
        close(pipefd[0]);
        close(pipefd[1]);
        return -1;
    }
    if (pid == 0) {
        dup2(pipefd[0], STDIN_FILENO);
        close(pipefd[0]);
        close(pipefd[1]);
        execl(angleExe.c_str(),
              angleExe.c_str(),
              "--ack-file",
              gSV_AckFile.c_str(),
              static_cast<char*>(nullptr));
        std::perror("execl");
        _exit(127);
    }
    close(pipefd[0]);
    gSV_AngleStdinFd = pipefd[1];
    const int flags = fcntl(gSV_AngleStdinFd, F_GETFL, 0);
    if (flags >= 0) {
        (void)fcntl(gSV_AngleStdinFd, F_SETFL, flags | O_NONBLOCK);
    }
    return pid;
}

/* ----------------------------------------------------------------------------
 * Function : main
 * Description : Application entry point
 * ----------------------------------------------------------------------------
 */
int main()
{
    auto params = SV_LoadConfig("/home/raspberrypi/build/configParam.ini");

    std::signal(SIGINT, SV_HandleSignal);
    std::signal(SIGTERM, SV_HandleSignal);

    {
        std::ofstream ackReset(gSV_AckFile, std::ios::trunc);
    }
    gSV_AnglePid = SV_StartAngleProcess();

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();
    if (cm->cameras().empty())
        return -1;

    gSV_Camera = cm->get(cm->cameras()[0]->id());
    gSV_Camera->acquire();

    auto config = gSV_Camera->generateConfiguration({ StreamRole::Raw });
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.pixelFormat = libcamera::formats::SRGGB10;
    streamConfig.size = {1456, 1088};
    config->validate();
    gSV_Camera->configure(config.get());

    std::unique_ptr<FrameBufferAllocator> allocator =
        std::make_unique<FrameBufferAllocator>(gSV_Camera);
    for (StreamConfiguration &cfg : *config)
        allocator->allocate(cfg.stream());

    Stream *stream = streamConfig.stream();
    const auto &buffers = allocator->buffers(stream);

    std::vector<std::unique_ptr<Request>> requests;
    for (const auto &b : buffers) {
        auto req = gSV_Camera->createRequest();
        req->addBuffer(stream, b.get());
        requests.push_back(std::move(req));
    }

    ControlList controls(gSV_Camera->controls());

    // Disable auto-exposure and set manual parameters
    controls.set(controls::AeEnable, false);
    controls.set(controls::ExposureTimeMode, controls::ExposureTimeModeManual);
    controls.set(controls::AnalogueGainMode, controls::AnalogueGainModeManual);

    int exposure = 20000;
    float gain = 1.0f;
    try {
        const auto itExposure = params.find("ExposureTime");
        if (itExposure != params.end()) {
            exposure = std::stoi(itExposure->second);
        }
    } catch (...) {
        std::cerr << "Invalid ExposureTime in config, using default 20000 us\n";
        exposure = 20000;
    }
    try {
        const auto itGain = params.find("Gain");
        if (itGain != params.end()) {
            gain = std::stof(itGain->second);
        }
    } catch (...) {
        std::cerr << "Invalid Gain in config, using default 1.0\n";
        gain = 1.0f;
    }
    if (gain < 1.0f)
        gain = 1.0f;

    controls.set(controls::ExposureTime, exposure);
    controls.set(controls::AnalogueGain, gain);
    std::array<int64_t, 2> frameDurationLimits = {50000, 50000}; // 20 FPS
    controls.set(controls::FrameDurationLimits, frameDurationLimits);

    std::cout << "Applied manual settings:\n"
              << "  ExposureTime = " << exposure << " us\n"
              << "  AnalogueGain = " << gain << "\n"
              << "  FrameRate = 20 FPS\n";

    // Start capture
    gSV_Camera->requestCompleted.connect(SV_RequestComplete);
    gSV_Camera->start(&controls);
    for (auto &req : requests)
        gSV_Camera->queueRequest(req.get());

    while (!gSV_StopRequested.load())
        std::this_thread::sleep_for(100ms);

    gSV_Camera->stop();
    allocator->free(stream);
    gSV_Camera->release();
    cm->stop();

    if (gSV_AnglePid > 0) {
        kill(gSV_AnglePid, SIGTERM);
        waitpid(gSV_AnglePid, nullptr, 0);
        gSV_AnglePid = -1;
    }

    std::cout << "Done.\n";
    return 0;
}

/* end of file */
