#pragma once

#include <igl/readMESH.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/readOFF.h>
#include <igl/boundary_facets.h>

#include "visualization.h"
#include "EigenTypes.h"
#include "build_skinning_matrix.h"
#include "init_state.h"
#include "mass_matrix_mesh.h"
#include "find_min_vertices.h"
#include "fixed_point_constraints.h"

// Variable for geometry
Eigen::MatrixXd V; // vertices of simulation mesh
Eigen::MatrixXi T; // faces of simulation mesh
Eigen::MatrixXi F; // faces of simulation mesh

// variables for skinning
Eigen::MatrixXd V_skin;
Eigen::MatrixXi F_skin;
Eigen::SparseMatrixd N;

// material parameters
double density = 0.1;
double YM = 6e5;      // young's modulus
double poisson = 0.3; // poissons ratio
double D = 0.5 * (YM * poisson) / ((1.0 + poisson) * (1.0 - 2.0 * poisson));
double C = 0.5 * YM / (2.0 * (1.0 + poisson));

// BC
std::vector<unsigned int> fixed_point_indices;
Eigen::SparseMatrixd P;
Eigen::VectorXd x0;

// mass matrix
Eigen::SparseMatrixd M;
Eigen::VectorXd v0;

bool stable = false;
bool skinning_on = false;
bool fully_implicit = false;

inline void simulate(Eigen::VectorXd &q, Eigen::VectorXd &qdot, double dt, double t)
{
}

inline void draw(Eigen::Ref<const Eigen::VectorXd> q, Eigen::Ref<const Eigen::VectorXd> qdot, double t)
{
    // update vertex positions using simulation
    Visualize::update_vertex_positions(0, P.transpose() * q + x0);
}

bool key_down_callback(igl::opengl::glfw::Viewer &viewer, unsigned char key, int modifiers)
{

    if (key == 'S')
    {
        skinning_on = !skinning_on;
        Visualize::toggle_skinning(skinning_on);
    }

    return false;
}

inline void scene_setup(int argc, char **argv, Eigen::VectorXd &q, Eigen::VectorXd &qdot)
{
    igl::readMESH("../data/coarse_bunny.mesh", V, T, F);
    igl::readOBJ("../data/bunny_skin.obj", V_skin, F_skin);

    fully_implicit = true;
    if (argc > 1)
    {
        if (strcmp(argv[1], "stable") == 0)
            stable = true;
    }

    igl::boundary_facets(T, F);
    F = F.rowwise().reverse().eval();

    build_skinning_matrix(N, V, T, V_skin);

    init_state(q, qdot, V);

    Visualize::add_object_to_scene(V, F, V_skin, F_skin, N, Eigen::RowVector3d(30, 60, 224) / 255.);
    Visualize::toggle_skinning(true);
    Visualize::set_picking_tolerance(0.01);

    igl::volume(V, T, v0);

    // Mass Matrix
    mass_matrix_mesh(M, qdot, T, density, v0);

    if (M.rows() == 0)
    {
        std::cout << "Mass Matrix not implemented, quitting \n";
        std::exit(0);
    }

    // setup constraint matrix
    find_min_vertices(fixed_point_indices, V, 0.1);

    P.resize(q.rows(), q.rows());
    P.setIdentity();
    fixed_point_constraints(P, q.rows(), fixed_point_indices);

    x0 = q - P.transpose() * P * q; // vertex x0 contains position of all fixed nodes, zero for everything else
    // correct M, q and qdot so they are the right size
    q = P * q;
    qdot = P * qdot;
    M = P * M * P.transpose();

    Visualize::viewer().callback_key_down = key_down_callback;
}