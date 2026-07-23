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
 *    Global Shutter camera using libcamera. Each frame is stored in
 *    RAW format along with metadata (exposure time, gain, sequence, etc.).
 *
 *    The program supports manual exposure and analogue gain control.
 * ----------------------------------------------------------------------------
 * History :
 *    30/09/2025  S.Diop : creation
 * ============================================================================
 */

/* ----------------------------------------------------------------------------
 * Include
 * ----------------------------------------------------------------------------
 */

/* Standard include */
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <fstream>
#include <sstream>
#include <map>
#include <sys/mman.h>
#include <sys/stat.h>

/* Third party include */
#include <libcamera/libcamera.h>

/* Project include */
#include <errlog/err_error.h>   // if ARCK error logging is available

using namespace libcamera;
using namespace std::chrono_literals;

/* ----------------------------------------------------------------------------
 * Global declarations
 * ----------------------------------------------------------------------------
 */

static int gSV_FrameCounter = 0;
static std::shared_ptr<Camera> gSV_Camera;

/* ----------------------------------------------------------------------------
 * Function prototypes
 * ----------------------------------------------------------------------------
 */

std::map<std::string, std::string> SV_LoadConfig(const std::string &filename);
void SV_SaveFrameRaw(Request *request, FrameBuffer *buffer, int frameNumber);
static void SV_RequestComplete(Request *request);

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
void SV_SaveFrameRaw(Request *request, FrameBuffer *buffer, int frameNumber)
{
    std::string outDir = "/home/pi/captures/";
    mkdir(outDir.c_str(), 0777);

    const FrameBuffer::Plane &plane = buffer->planes()[0];
    size_t length = buffer->metadata().planes()[0].bytesused;
    void *mem = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (mem == MAP_FAILED) {
        perror("mmap");
        return;
    }

    std::ostringstream base;
    base << outDir << "frame_" << frameNumber;
    std::string rawFile = base.str() + ".raw";
    std::string metaFile = base.str() + "_metadata.txt";

    // Save RAW
    std::ofstream file(rawFile, std::ios::binary);
    file.write(static_cast<char*>(mem), length);
    file.close();
    munmap(mem, plane.length);
    std::cout << "✅ Saved RAW: " << rawFile << " (" << length << " bytes)" << std::endl;

    // Save metadata
    std::ofstream meta(metaFile);
    const FrameMetadata &metadata = buffer->metadata();
    meta << "Frame " << frameNumber << " Metadata\n"
         << "--------------------------------------\n";
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
    std::cout << " Metadata saved: " << metaFile << std::endl;
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

    for (auto &pair : request->buffers())
        SV_SaveFrameRaw(request, pair.second, gSV_FrameCounter++);

    request->reuse(Request::ReuseBuffers);
    gSV_Camera->queueRequest(request);
}

/* ----------------------------------------------------------------------------
 * Function : main
 * Description : Application entry point
 * ----------------------------------------------------------------------------
 */
int main()
{
    auto params = SV_LoadConfig("config.ini");

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

    int exposure = std::stoi(params["ExposureTime"]);
    float gain = std::stof(params["Gain"]);
    if (gain < 1.0f)
        gain = 1.0f;

    controls.set(controls::ExposureTime, exposure);
    controls.set(controls::AnalogueGain, gain);

    std::cout << "Applied manual settings:\n"
              << "  ExposureTime = " << exposure << " us\n"
              << "  AnalogueGain = " << gain << "\n";

    // Start capture
    gSV_Camera->requestCompleted.connect(SV_RequestComplete);
    gSV_Camera->start(&controls);
    for (auto &req : requests)
        gSV_Camera->queueRequest(req.get());

    std::this_thread::sleep_for(3000ms);
    gSV_Camera->stop();
    allocator->free(stream);
    gSV_Camera->release();
    cm->stop();

    std::cout << "✅ Done.\n";
    return 0;
}

/* end of file */
