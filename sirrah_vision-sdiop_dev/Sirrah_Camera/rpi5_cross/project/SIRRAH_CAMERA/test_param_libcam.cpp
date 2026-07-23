#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <fstream>
#include <sys/mman.h> // pour mmap

#include <libcamera/libcamera.h>

using namespace libcamera;
using namespace std::chrono_literals;

// Compteur global pour nommer les fichiers RAW
static int frameCounter = 0;
static std::shared_ptr<Camera> camera;

// Fonction pour sauvegarder un FrameBuffer en fichier RAW
void saveFrameRaw(FrameBuffer *buffer, int frameNumber)
{
    const FrameBuffer::Plane &plane = buffer->planes()[0];
    size_t length = buffer->metadata().planes()[0].bytesused;

    // Mapper le buffer en mémoire
    void *mem = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
    if (mem == MAP_FAILED) {
        perror("mmap");
        return;
    }

    // Générer un nom de fichier unique
    std::string filename = "frame_" + std::to_string(frameNumber) + ".raw";

    // Écriture brute dans le fichier
    std::ofstream file(filename, std::ios::binary);
    file.write(static_cast<char*>(mem), length);
    file.close();

    munmap(mem, plane.length);
    std::cout << "Saved " << filename << " (" << length << " bytes)" << std::endl;
}
