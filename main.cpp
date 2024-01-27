#include <iostream>

#include "scene_setup.h"
#include "visualization.h"

// Simulate State
Eigen::VectorXd q;
Eigen::VectorXd qdot;

// simulation time and time step
double t = 0;     // simulation time
double dt = 0.01; // time step

// simulation loop
bool simulating = true;

bool simulation_callback();
bool draw_callback(igl::opengl::glfw::Viewer &viewer);

int main(int argc, char **argv)
{
    scene_setup(argc, argv, q, qdot);

    // run simulation in seperate thread to avoid slowing down the UI
    std::thread simulation_thread(simulation_callback);
    simulation_thread.detach();

    Visualize::setup(q, qdot);
    Visualize::viewer().callback_post_draw = &draw_callback;

    Visualize::viewer().launch();

    return 0;
}

bool draw_callback(igl::opengl::glfw::Viewer &viewer)
{

    draw(q, qdot, t);

    return false;
}

bool simulation_callback()
{

    while (simulating)
    {
        simulate(q, qdot, dt, t);
        t += dt;
    }

    return false;
}
