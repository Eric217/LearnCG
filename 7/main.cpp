#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{

    // Change the definition here to change resolution
    Scene scene(784, 784);
//    Scene scene(400, 400);
//    Scene scene(200, 200);

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);

    std::string cornell_dir = CORNELL_DIR;
    cornell_dir.append("/");
    
    MeshTriangle floor(cornell_dir + "floor.obj", white);
    //MeshTriangle shortbox(cornell_dir + "shortbox.obj", white);
    //MeshTriangle tallbox(cornell_dir + "tallbox.obj", white);
    MeshTriangle left(cornell_dir + "left.obj", red);
    MeshTriangle right(cornell_dir + "right.obj", green);
    MeshTriangle light_(cornell_dir + "light.obj", light);

    Material* whiteBunny = new Material(DIFFUSE, Vector3f(0.0f));
    whiteBunny->Kd = Vector3f(0.98f, 0.98f, 0.98f);
    MeshTriangle bunny(BUNNY_PATH, whiteBunny, 1500, Vector3f(200, -60, 320));
    scene.Add(&bunny);
    
    MeshTriangle cow(COW_PATH, whiteBunny, 232, Vector3f(400, 138, 350));
    scene.Add(&cow);
    
    scene.Add(&floor);
   // scene.Add(&shortbox);
   // scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " <<   std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() % 60 << " minutes\n";
    
    auto ss = std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() % 60;
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() % 1000;

    std::cout << "          : " << ss << " seconds\n";
    std::cout << "          : " << ms << " millseconds\n";
    
    return 0;
}
