#include "visualization.h"
#include "EigenTypes.h"

namespace Visualize
{
    igl::opengl::glfw::Viewer g_viewer;

    igl::opengl::glfw::imgui::ImGuiMenu menu;
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;

    // meshes in the scene
    std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>> g_geometry;
    std::vector<Eigen::RowVector3d> g_color;
    std::vector<std::pair<Eigen::MatrixXd, Eigen::MatrixXi>> g_skin;
    std::vector<Eigen::SparseMatrixd> g_N;
    bool g_skinning = true;

    std::vector<unsigned int> g_id; // id into libigl for these meshes

    // pointers to q and qdot
    Eigen::VectorXd const *g_q;
    Eigen::VectorXd const *g_qdot;

    // mouse UI state variables
    bool g_mouse_dragging = false;
    double g_picking_tol = 0.001;
    Eigen::Vector3d g_mouse_win;  // mouse window coordinates
    Eigen::Vector3d g_mouse_drag; // last mouse drag vector
    Eigen::Vector3d g_mouse_world;
    Eigen::Vector3d g_mouse_drag_world; // mouse drag delta in the world space
}

igl::opengl::glfw::Viewer &Visualize::viewer()
{
    return g_viewer;
}

void Visualize::setup(const Eigen::VectorXd &q,
                      const Eigen::VectorXd &qdot)
{
    g_q = &q;
    g_qdot = &qdot;

    // add new menu for phase space plotting
    Visualize::g_viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);

    menu.callback_draw_viewer_menu = [&]()
    {
        menu.draw_viewer_menu();
    };

    Visualize::g_viewer.core().background_color.setConstant(1.0);
    Visualize::g_viewer.core().is_animating = false;
}

void Visualize::set_picking_tolerance(double r)
{
    g_picking_tol = r;
}

void Visualize::update_vertex_positions(
    unsigned int id,
    Eigen::Ref<const Eigen::VectorXd> pos)
{
    // update vertex positions
    for (unsigned int ii = 0; ii < g_geometry[id].first.rows(); ii++)
    {
        g_geometry[g_id[id]].first.row(ii) = pos.segment<3>(3 * ii).transpose();
    }

    if (g_skinning)
        g_viewer.data_list[g_id[id]].V = g_N[id] * g_geometry[g_id[id]].first;
    else
        g_viewer.data_list[g_id[id]].V = g_geometry[g_id[id]].first;

    // tell viewer to update
    g_viewer.data_list[g_id[id]].dirty |= igl::opengl::MeshGL::DIRTY_POSITION;
}

void Visualize::add_object_to_scene(
    const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
    const Eigen::MatrixXd &V_skin, const Eigen::MatrixXi &F_skin,
    const Eigen::SparseMatrixd &N,
    Eigen::RowVector3d color)
{
    // add mesh to libigl and store id for access later
    if (g_geometry.size() == 0)
    {
        g_id.push_back(0);
    }
    else
    {
        g_id.push_back(g_viewer.append_mesh());
    }

    g_viewer.data().set_mesh(V_skin, F_skin);
    g_viewer.data().set_colors(color);
    g_viewer.data().show_lines = false;

    // add mesh to geometry vector
    g_geometry.push_back(std::make_pair(V, F));
    g_color.push_back(color);
    g_skin.push_back(std::make_pair(V_skin, F_skin));
    g_N.push_back(N);
}

void Visualize::toggle_skinning(bool skinning)
{
    g_skinning = skinning;
    if (!skinning)
    {
        for (unsigned int ii = 0; ii < g_geometry.size(); ++ii)
        {
            g_viewer.data_list[g_id[ii]].clear();
            g_viewer.data_list[g_id[ii]].set_mesh(g_geometry[ii].first, g_geometry[ii].second);
            g_viewer.data_list[g_id[ii]].set_colors(g_color[ii]);
        }
    }
    else
    {
        for (unsigned int ii = 0; ii < g_geometry.size(); ++ii)
        {
            g_viewer.data_list[g_id[ii]].clear();
            g_viewer.data_list[g_id[ii]].set_mesh(g_skin[ii].first, g_skin[ii].second);
            g_viewer.data_list[g_id[ii]].set_colors(g_color[ii]);
        }
    }
}