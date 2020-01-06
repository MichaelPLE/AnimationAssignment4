
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"

#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <fstream>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <sstream>
#include <igl/readOFF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <sstream>
void config(igl::opengl::glfw::Viewer& viewer, Renderer& renderer)
{

    std::ifstream myFile;
    myFile.open("configuration.txt");
    int i = 0;
    if (myFile) {
	    std::string path;
	    while (getline(myFile, path))
	    {
		
		    viewer.load_mesh_from_file(path);
		    //viewer.parentId.push_back(-1);
		    //viewer.data().MyTranslate(Eigen::Vector3f(5, 0, 0));
            viewer.selected_data_index = i;
            viewer.data_list[i].show_lines = false;
		    //renderer.core().toggle(viewer.data_list[i].show_lines);
            i++;
	    }
    }
    else
    {
	    std::cout << "Couldn't find configuration file" << std::endl;
	    return;
    }
    viewer.data_list[0].MyTranslate(Eigen::Vector3f(-0.3, 0, 0));
    viewer.data_list[1].MyTranslate(Eigen::Vector3f(0.3, 0, 0));
    viewer.selected_data_index = 0;
}

int main(int argc, char *argv[])
{
  Display *disp = new Display(1000, 800, "Wellcome");
  Renderer renderer;
  igl::opengl::glfw::Viewer viewer;
  viewer.reset();
  Eigen::Vector3f mov = Eigen::Vector3f(0, 0, -1);
  viewer.MyTranslate(mov);
  config(viewer, renderer);

  //renderer.core().toggle(viewer.data().show_lines);
  Init(*disp);
  renderer.init(&viewer);
  disp->SetRenderer(&renderer);
  //int t = viewer.data_list.size();
  //for (int i = 0; i < t; i++) {
  //    viewer.selected_data_index = i;
  //    update_visualization(&viewer, &renderer);
  //}

  disp->launch_rendering(true);
  delete disp;
}
