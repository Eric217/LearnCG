//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
#ifndef SCREEN_MINIFICATION
    #define SCREEN_MINIFICATION 1
#endif
    float bufferCoeff = SCREEN_MINIFICATION;
    int bufferW = scene.width * bufferCoeff;
    int bufferH = scene.height * bufferCoeff;
    std::vector<Vector3f> framebuffer(bufferH * bufferW);

    float scale = std::tan(deg2rad(scene.fov * 0.5f));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(-1, 5, 10);
//    Vector3f eye_pos(0, 0, 10);
    int m = 0;
     
    for (uint32_t j = 0; j < bufferH; ++j) {
        for (uint32_t i = 0; i < bufferW; ++i) {
            // generate primary ray direction
            float x = (2 * (i + 0.5) / (float)bufferW - 1) *
                      imageAspectRatio * scale;
            float y = (1 - 2 * (j + 0.5) / (float)bufferH) * scale;
            auto ray = Ray(eye_pos, normalize(Vector3f(x, y, -1)));
            framebuffer[m++] = scene.castRay(ray, 0);
            
            // TODO: Find the x and y positions of the current pixel to get the
            // direction
            //  vector that passes through it.
            // Also, don't forget to multiply both of them with the variable
            // *scale*, and x (horizontal) variable with the *imageAspectRatio*

            // Don't forget to normalize this direction!

        }
        UpdateProgress(j / (float)bufferH);
    }
    UpdateProgress(1.f);

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", bufferW, bufferH);
    for (auto i = 0; i < bufferW * bufferH; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].x));
        color[1] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].y));
        color[2] = (unsigned char)(255 * clamp(0, 1, framebuffer[i].z));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
