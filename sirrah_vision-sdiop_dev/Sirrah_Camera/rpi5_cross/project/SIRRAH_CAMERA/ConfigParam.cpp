/* ============================================================================
 * Project   : SIRRAH VISION
 * ----------------------------------------------------------------------------
 * Title     : RAW Image Capture with Manual Exposure and Metadata Saving
 * Author    : Serigne Saliou Mbacké Diop
 * Date      : 30/09/2025
 * ----------------------------------------------------------------------------
 * Confidential file
 * Copyright (C) ARCK Sensor - All rights reserved
 * ----------------------------------------------------------------------------
 * Description :
 *    Main program for configuring and capturing RAW frames from the
 *    TIS_36 camera using libcamera. Each frame is stored in
 *    RAW format along with metadata (exposure time, gain, sequence, etc.).
 *
 *    The program supports manual exposure and analogue gain control.
 * ----------------------------------------------------------------------------
 * History :
 *    30/09/2025  S.Diop : creation
 *    02/03/2026  S.Diop : added async saving 
 * ============================================================================
 */

/* ----------------------------------------------------------------------------
 * Include
 * ----------------------------------------------------------------------------
*/

#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <fstream>
#include <sstream>
#include <map>
#include <array>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstring>
#include <cstdint>
#include <sys/mman.h>
#include <sys/stat.h>
#include <chrono>

#include <libcamera/libcamera.h>

using namespace libcamera;
using namespace std::chrono_literals;

static int frameCounter = 0;
static std::shared_ptr<Camera> camera;
static const std::string outDir = "/home/raspberrypi/captures/led_test_fps_tis36/";

// Timing globals
static std::chrono::steady_clock::time_point lastTimestamp;
static bool firstFrame = true;

struct FrameSaveTask
{
    int frameNumber;
    std::vector<uint8_t> rawData;
    uint32_t sequence;
    uint64_t timestamp;
    bool hasExposure;
    int64_t exposureUs;
    bool hasAnalogueGain;
    float analogueGain;
    bool hasDigitalGain;
    float digitalGain;
};

static std::deque<FrameSaveTask> saveQueue;
static std::mutex saveQueueMutex;
static std::condition_variable saveQueueCv;
static std::atomic<bool> writerStop{false};
static std::atomic<bool> captureRunning{false};
static std::atomic<uint64_t> capturedFrames{0};
static std::atomic<uint64_t> enqueuedFrames{0};
static std::atomic<uint64_t> savedFrames{0};
static std::atomic<uint64_t> droppedFrames{0};
static std::thread writerThread;

/* ---------------- INI FILE PARSER ---------------- */
std::map<std::string, std::string> loadConfig(const std::string &filename)
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

/* ---------------- ASYNC SAVE ---------------- */
static void writerLoop()
{
    mkdir(outDir.c_str(), 0777);

    while (true) {
        FrameSaveTask task;
        {
            std::unique_lock<std::mutex> lock(saveQueueMutex);
            saveQueueCv.wait(lock, [] {
                return writerStop.load() || !saveQueue.empty();
            });

            if (saveQueue.empty()) {
                if (writerStop.load())
                    break;
                continue;
            }

            task = std::move(saveQueue.front());
            saveQueue.pop_front();
        }

        std::ostringstream base;
        base << outDir << "test_fps_tis36" << task.frameNumber;
        std::string rawFile = base.str() + ".raw";
        std::string metaFile = base.str() + "_metadata.txt";

        std::ofstream file(rawFile, std::ios::binary);
        file.write(reinterpret_cast<const char *>(task.rawData.data()), task.rawData.size());
        file.close();
        std::cout << "Saved RAW: " << rawFile << " (" << task.rawData.size() << " bytes)" << std::endl;

        std::ofstream meta(metaFile);
        meta << "Frame " << task.frameNumber << " Metadata\n"
             << "--------------------------------------\n";
        meta << "Sequence: " << task.sequence << "\n";
        meta << "Timestamp: " << task.timestamp << "\n";
        if (task.hasExposure)
            meta << "ExposureTime (us): " << task.exposureUs << "\n";
        if (task.hasAnalogueGain)
            meta << "AnalogueGain: " << task.analogueGain << "\n";
        if (task.hasDigitalGain)
            meta << "DigitalGain: " << task.digitalGain << "\n";
        meta.close();
        std::cout << "Metadata saved: " << metaFile << std::endl;
        savedFrames.fetch_add(1, std::memory_order_relaxed);
    }
}

static void enqueueFrameSave(Request *request, FrameBuffer *buffer, int frameNumber)
{
    const FrameBuffer::Plane &plane = buffer->planes()[0];
    size_t length = buffer->metadata().planes()[0].bytesused;
    void *mem = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (mem == MAP_FAILED) {
        perror("mmap");
        droppedFrames.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    FrameSaveTask task{};
    task.frameNumber = frameNumber;
    task.rawData.resize(length);
    std::memcpy(task.rawData.data(), mem, length);
    munmap(mem, plane.length);

    const FrameMetadata &metadata = buffer->metadata();
    task.sequence = metadata.sequence;
    task.timestamp = metadata.timestamp;

    const ControlList &md = request->metadata();
    task.hasExposure = md.contains(controls::ExposureTime.id());
    if (task.hasExposure)
        task.exposureUs = md.get(controls::ExposureTime).value_or(0);
    task.hasAnalogueGain = md.contains(controls::AnalogueGain.id());
    if (task.hasAnalogueGain)
        task.analogueGain = md.get(controls::AnalogueGain).value_or(0.0f);
    task.hasDigitalGain = md.contains(controls::DigitalGain.id());
    if (task.hasDigitalGain)
        task.digitalGain = md.get(controls::DigitalGain).value_or(0.0f);

    {
        std::lock_guard<std::mutex> lock(saveQueueMutex);
        saveQueue.push_back(std::move(task));
    }
    enqueuedFrames.fetch_add(1, std::memory_order_relaxed);
    saveQueueCv.notify_one();
}

/* ---------------- CALLBACK ---------------- */
static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;
    if (!captureRunning.load(std::memory_order_relaxed))
        return;

    auto now = std::chrono::steady_clock::now();
    if (!firstFrame) {
        auto deltaUs = std::chrono::duration_cast<std::chrono::microseconds>(now - lastTimestamp).count();
        if (deltaUs > 0) {
            double fps = 1000000.0 / static_cast<double>(deltaUs);
            std::cout << "Frame interval: " << std::fixed << std::setprecision(3)
                      << (deltaUs / 1000.0) << " ms  (" << std::setprecision(2)
                      << fps << " FPS)" << std::endl;
        }
    } else {
        firstFrame = false;
    }
    lastTimestamp = now;

    for (auto &pair : request->buffers()) {
        capturedFrames.fetch_add(1, std::memory_order_relaxed);
        enqueueFrameSave(request, pair.second, frameCounter++);
    }

    if (captureRunning.load(std::memory_order_relaxed)) {
        request->reuse(Request::ReuseBuffers);
        camera->queueRequest(request);
    }
}

/* ---------------- MAIN PROGRAM ---------------- */
int main()
{
    auto params = loadConfig("/home/raspberrypi/build/configParam.ini");

    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();
    if (cm->cameras().empty())
        return -1;

    camera = cm->get(cm->cameras()[0]->id());
    camera->acquire();

    auto config = camera->generateConfiguration({ StreamRole::Raw });
    StreamConfiguration &streamConfig = config->at(0);
    streamConfig.pixelFormat = libcamera::formats::SRGGB10;
    streamConfig.size = {2064, 1552};
    config->validate();
    camera->configure(config.get());

    std::unique_ptr<FrameBufferAllocator> allocator =
        std::make_unique<FrameBufferAllocator>(camera);
    for (StreamConfiguration &cfg : *config)
        allocator->allocate(cfg.stream());

    Stream *stream = streamConfig.stream();
    const auto &buffers = allocator->buffers(stream);

    std::vector<std::unique_ptr<Request>> requests;
    for (const auto &b : buffers) {
        auto req = camera->createRequest();
        req->addBuffer(stream, b.get());
        requests.push_back(std::move(req));
    }

    ControlList controls(camera->controls());
    controls.set(controls::AeEnable, false);
    controls.set(controls::ExposureTimeMode, controls::ExposureTimeModeManual);
    controls.set(controls::AnalogueGainMode, controls::AnalogueGainModeManual);

    int exposure = std::stoi(params["ExposureTime"]);
    float gain = std::stof(params["Gain"]);
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

    camera->requestCompleted.connect(requestComplete);
    writerStop = false;
    captureRunning = true;
    writerThread = std::thread(writerLoop);
    camera->start(&controls);

    for (auto &req : requests)
        camera->queueRequest(req.get());

    std::this_thread::sleep_for(30000ms);

    captureRunning = false;
    camera->stop();
    camera->requestCompleted.disconnect(requestComplete);
    writerStop = true;
    saveQueueCv.notify_all();
    if (writerThread.joinable())
        writerThread.join();
    requests.clear();
    allocator->free(stream);
    camera->release();
    camera.reset();
    cm->stop();

    size_t pendingQueue = 0;
    {
        std::lock_guard<std::mutex> lock(saveQueueMutex);
        pendingQueue = saveQueue.size();
    }

    std::cout << "Capture stats: captured=" << capturedFrames.load()
              << " enqueued=" << enqueuedFrames.load()
              << " saved=" << savedFrames.load()
              << " dropped=" << droppedFrames.load()
              << " pending=" << pendingQueue << "\n";
    std::cout << "Done.\n";
    return 0;
}
