#pragma once
#include "igl/opengl/glfw/Display.h"
#include <igl/cat.h>
#include <igl/edge_lengths.h>
#include <igl/parula.h>
#include <igl/per_edge_normals.h>
#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/readMESH.h>
#include <igl/signed_distance.h>
#include <igl/slice_mask.h>
#include <igl/marching_tets.h>
#include <igl/upsample.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/writeOBJ.h>
#include <Eigen/Sparse>
#include <iostream>


static void glfw_mouse_press(GLFWwindow* window, int button, int action, int modifier)
{

	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);

	if (action == GLFW_PRESS)
	{
		double x2, y2;
		glfwGetCursorPos(window, &x2, &y2);
		igl::opengl::glfw::Viewer* scn = rndr->GetScene();
		float found;
		int i = 0, savedIndx = scn->selected_data_index;
		int nearest = -1;
		float min_t = 2 ^ 31;

		for (; i < scn->data_list.size(); i++)
		{
			scn->selected_data_index = i;
			found = rndr->Picking(x2, y2);
			if (found >= 0 && found < min_t) // fid is positive(or 0) -> the cursor hit the obj i
			{
				min_t = found;
				nearest = i;
			}
		}
		scn->selected_data_index = nearest;

		if (nearest == -1)
		{
			
			scn->selected_data_index = -1;
		}

		rndr->UpdatePosition(x2, y2);
	}
	//lk.lock();
}




 void glfw_mouse_move(GLFWwindow* window, double x, double y)
{
	 Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	 rndr->UpdatePosition(x, y);
	 if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
	 {
		 rndr->MouseProcessing(GLFW_MOUSE_BUTTON_RIGHT);
		
	 }
	 else if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	 {
		 rndr->MouseProcessing(GLFW_MOUSE_BUTTON_LEFT);
		
	 }
	
}

static void glfw_mouse_scroll(GLFWwindow* window, double x, double y)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	if (rndr->GetScene()->selected_data_index == -1) {
		rndr->GetScene()->MyScale(Eigen::Vector3f(1 + y * 0.01, 1 + y * 0.01, 1 + y * 0.01));
		return;
	}
	rndr->GetScene()->data().MyScale(Eigen::Vector3f(1 + y * 0.01,1 + y * 0.01,1+y*0.01));

}

void glfw_window_size(GLFWwindow* window, int width, int height)
{
	Renderer* rndr = (Renderer*)glfwGetWindowUserPointer(window);
	//igl::opengl::glfw::Viewer* scn = rndr->GetScene();

    rndr->post_resize(window,width, height);

}


static void glfw_key_callback(GLFWwindow* window, int key, int scancode, int action, int modifier)
{
	Renderer* rndr = (Renderer*) glfwGetWindowUserPointer(window);
	igl::opengl::glfw::Viewer* scn = rndr->GetScene();
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	else if(action == GLFW_PRESS || action == GLFW_REPEAT)
		switch (key)
		{
		case ' ':
			scn->collided = !scn->collided;
			break;

		case 'S':
			
			break;
		case GLFW_KEY_RIGHT: // rotate y positive
			if (scn->selected_data_index != -1)
			{
				scn->data().MyRotate(Eigen::Vector3f(0, 1, 0), (2.0 / 18.0));
 
			}
			else {
				scn->MyRotate(Eigen::Vector3f(0, 1, 0), (2.0 / 18.0));
			}
			break;
		case GLFW_KEY_LEFT: // rotate y negetive
			if (scn->selected_data_index != -1)
			{
				scn->data().MyRotate(Eigen::Vector3f(0, 1, 0), -(2.0 / 18.0));
			
			}
			else {
				scn->MyRotate(Eigen::Vector3f(0, 1, 0), -(2.0 / 18.0));
			}
			break;
		case GLFW_KEY_UP: // rotate y positive
			if (scn->selected_data_index != -1)
			{
				scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), (2.0 / 18.0));

				//scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), 0.1);
			}
			else {
				scn->MyRotate(Eigen::Vector3f(1, 0, 0), (2.0 / 18.0));
			}

			break;
		case GLFW_KEY_DOWN: // rotate y negetive
			if (scn->selected_data_index != -1)
			{
				scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), -(2.0 / 18.0)); 

				//scn->data().MyRotate(Eigen::Vector3f(1, 0, 0), -0.1);
			}
			else {
				scn->MyRotate(Eigen::Vector3f(1, 0, 0), -(2.0 / 18.0));
			}

			break;
		case 'A':
		case 'a':
		{
			rndr->core().is_animating = !rndr->core().is_animating;
			break;
		}
		case 'F':
		case 'f':
		{
			scn->data().set_face_based(!scn->data().face_based);
			break;
		}
		case 'I':
		case 'i':
		{
			scn->data().dirty |= igl::opengl::MeshGL::DIRTY_NORMAL;
			scn->data().invert_normals = !scn->data().invert_normals;
			break;
		}
		case 'L':
		case 'l':
		{
			rndr->core().toggle(scn->data().show_lines);
			break;
		}
		case 'O':
		case 'o':
		{
			rndr->core().orthographic = !rndr->core().orthographic;
			break;
		}
		case 'T':
		case 't':
		{
			rndr->core().toggle(scn->data().show_faces);
			break;
		}
		case '1':
		case '2':
		{
			scn->selected_data_index =
				(scn->selected_data_index + scn->data_list.size() + (key == '2' ? 1 : -1)) % scn->data_list.size();
			break;
		}
		case '[':
		case ']':
		{
			rndr->ChangeCamera(key);
			break;
		}
		case ';':
			scn->data().show_vertid = !scn->data().show_vertid;
			break;
		case ':':
			scn->data().show_faceid = !scn->data().show_faceid;
			break;
		default: break;//do nothing
		}
}


void Init(Display& display)
{
	display.AddKeyCallBack(glfw_key_callback);
	display.AddMouseCallBacks(glfw_mouse_press, glfw_mouse_scroll, glfw_mouse_move);
	display.AddResizeCallBack(glfw_window_size);
}



