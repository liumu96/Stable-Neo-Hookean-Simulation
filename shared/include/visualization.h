#pragma once

#define IMGUI_DEFINE_MATH_OPERATORS

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>

#include <imgui.h>
#include <imgui_internal.h>

// stl
#include <deque>

// Eigen
#include <Eigen/Dense>
#include "EigenTypes.h"

namespace Visualize
{
    void setup(
        const Eigen::VectorXd &q,
        const Eigen::VectorXd &qdot);

    void add_object_to_scene(
        const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
        const Eigen::MatrixXd &V_skin, const Eigen::MatrixXi &F_skin,
        const Eigen::SparseMatrixd &N,
        Eigen::RowVector3d color);

    void update_vertex_positions(
        unsigned int id,
        Eigen::Ref<const Eigen::VectorXd> pos);

    void set_picking_tolerance(double);

    igl::opengl::glfw::Viewer &viewer();

    void toggle_skinning(bool skinning);

}