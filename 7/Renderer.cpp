//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <pool.h>

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

static void* task(void **args);
static void initCountLock(int w, int h);
static void tick();

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);

    bool usingThreads = true;
    // change the spp value to change sample ammount
    int spp = 32;
    std::cout << "SPP: " << spp << "\n";
    if (usingThreads) {
        thread_pool_t pool;
        thread_pool_init(&pool, CPU_CORE_COUNT);
        initCountLock(scene.width, scene.height);
        for (uint32_t j = 0; j < scene.height; ++j) {
            void **args = new void*[7];
            args[0] = (Scene *)(&scene);
            args[1] = &imageAspectRatio;
            args[2] = &scale;
            args[3] = &spp;
            args[4] = (void *)((size_t)j);
            args[5] = &eye_pos;
            args[6] = &framebuffer;
            thread_pool_add_task(&pool, (func)task, (void *)args);
        }
        printf("all task submitted -------------- \n");
        thread_pool_destroy(&pool);
        printf("all task done -------------- \n");
    } else {
        for (uint32_t j = 0; j < scene.height; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                          imageAspectRatio * scale;
                float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++){
                    framebuffer[scene.width * j + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
            }
            UpdateProgress(j / (float)scene.height);
        }
    }

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}

static void* task(void **args) {
    Scene *scene = (Scene *)(args[0]);
    float imageAspectRatio = *(float *)(args[1]);
    float scale = *(float *)(args[2]);
    int spp = *(int *)(args[3]);
    uint32_t j = uint32_t((size_t)(args[4]));
    Vector3f *eye_pos = (Vector3f *)(args[5]);
    std::vector<Vector3f> *framebuffer = (std::vector<Vector3f> *)(args[6]);
    
    auto co1 = imageAspectRatio * scale;
    for (uint32_t i = 0; i < scene->width; ++i) {
        // generate primary ray direction
        float x = (2 * (i + 0.5) / (float)scene->width - 1) * co1;
        float y = (1 - 2 * (j + 0.5) / (float)scene->height) * scale;

        Vector3f dir = normalize(Vector3f(-x, y, 1));
        auto m = scene->width * j + i;
        for (int k = 0; k < spp; k++){
            framebuffer->at(m) += scene->castRay(Ray(*eye_pos, dir), 0) / spp;
        }
    }
    tick();

    delete [] args;
}

static int __cur = 0;
static int __width = 0;
static int __h = 0;
static pthread_mutex_t m_mutex;

static void initCountLock(int w, int h) {
    __cur = 0;
    __width = w;
    __h = h;
    m_mutex = PTHREAD_MUTEX_INITIALIZER;
}

static void tick() {
    pthread_mutex_lock(&m_mutex);
    __cur ++;
    int percent = 100.0 * __cur / __h;
    pthread_mutex_unlock(&m_mutex);
    if (percent % 3) {
        return;
    }
    printf("%d%%\n", percent);
}
