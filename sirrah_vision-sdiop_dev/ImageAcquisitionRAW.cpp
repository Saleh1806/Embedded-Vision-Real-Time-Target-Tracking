#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <fstream>
#include <sys/mman.h> // for mmap

#include <libcamera/libcamera.h>

using namespace libcamera;
using namespace std::chrono_literals;

// Global counter to name RAW files
static int frameCounter = 0;
static std::shared_ptr<Camera> camera;

// Function to save a FrameBuffer to a RAW file
void saveFrameRaw(FrameBuffer *buffer, int frameNumber)
{
    const FrameBuffer::Plane &plane = buffer->planes()[0];
    size_t length = buffer->metadata().planes()[0].bytesused;

    // Map the buffer into memory
    void *mem = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (mem == MAP_FAILED) {
        perror("mmap");
        return;
    }

    // Generate a unique filename
    std::string filename = "frame_" + std::to_string(frameNumber) + ".raw";

    // Write raw data to file
    std::ofstream file(filename, std::ios::binary);
    file.write(static_cast<char*>(mem), length);
    file.close();

    munmap(mem, plane.length);
    std::cout << "Saved " << filename << " (" << length << " bytes)" << std::endl;
}

// Callback for completed requests
static void requestComplete(Request *request)
{
    if (request->status() == Request::RequestCancelled)
        return;

    const std::map<const Stream *, FrameBuffer *> &buffers = request->buffers();
    for (auto bufferPair : buffers)
    {
        FrameBuffer *buffer = bufferPair.second;
        const FrameMetadata &metadata = buffer->metadata();

        std::cout << " seq: " << std::setw(6) << std::setfill('0')
                  << metadata.sequence << " bytesused: ";

        unsigned int nplane = 0;
        for (const FrameMetadata::Plane &plane : metadata.planes())
        {
            std::cout << plane.bytesused;
            if (++nplane < metadata.planes().size())
                std::cout << "/";
        }
        std::cout << std::endl;

        // Save the RAW image
        saveFrameRaw(buffer, frameCounter++);
    }

    request->reuse(Request::ReuseBuffers);
    camera->queueRequest(request);
}

int main()
{
    std::unique_ptr<CameraManager> cm = std::make_unique<CameraManager>();
    cm->start();

    auto cameras = cm->cameras();
    if (cameras.empty()) {
        std::cout << "No cameras were identified on the system." << std::endl;
        cm->stop();
        return EXIT_FAILURE;
    }

    // Retrieve the first camera
    std::string cameraId = cameras[0]->id();
    camera = cm->get(cameraId);

    camera->acquire();
    std::unique_ptr<CameraConfiguration> config =
        camera->generateConfiguration({ StreamRole::Raw });

    StreamConfiguration &streamConfig = config->at(0);
    std::cout << "Default viewfinder configuration is: "
              << streamConfig.toString() << std::endl;

    // Set pixel format and image size
    streamConfig.pixelFormat = libcamera::formats::SRGGB10;
    streamConfig.size.width = 640;
    streamConfig.size.height = 480;

    // Validate configuration
    config->validate();
    std::cout << "Validated viewfinder configuration is: "
              << streamConfig.toString() << std::endl;

    // Apply configuration to the camera
    camera->configure(config.get());

    std::unique_ptr<FrameBufferAllocator> allocator =
        std::make_unique<FrameBufferAllocator>(camera);

    // Allocate buffers for each stream
    for (StreamConfiguration &cfg : *config) {
        int ret = allocator->allocate(cfg.stream());
        if (ret < 0) {
            std::cerr << "Can't allocate buffers" << std::endl;
            return -ENOMEM;
        }
        size_t allocated = allocator->buffers(cfg.stream()).size();
        std::cout << "Allocated " << allocated << " buffers for stream" << std::endl;
    }

    Stream *stream = streamConfig.stream();
    const auto &buffers = allocator->buffers(stream);

    // Create requests for each buffer
    std::vector<std::unique_ptr<Request>> requests;
    for (unsigned int i = 0; i < buffers.size(); ++i) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            std::cerr << "Can't create request" << std::endl;
            return -ENOMEM;
        }
        int ret = request->addBuffer(stream, buffers[i].get());
        if (ret < 0) {
            std::cerr << "Can't set buffer for request" << std::endl;
            return ret;
        }
        requests.push_back(std::move(request));
    }

    // Connect request completion signal to the callback
    camera->requestCompleted.connect(requestComplete);
    camera->start();

    // Queue all requests to start capturing
    for (auto &request : requests)
        camera->queueRequest(request.get());

    // Capture frames for 3 seconds
    std::this_thread::sleep_for(3000ms);

    // Stop camera and release resources
    camera->stop();
    allocator->free(stream);
    camera->release();
    camera.reset();
    cm->stop();

    return 0;
}
