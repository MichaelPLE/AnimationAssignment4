// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>
#include <igl/circulation.h>
#include <igl/collapse_edge.h>
#include <igl/edge_flaps.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <iostream>
#include <set>


// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
	namespace opengl
	{
		namespace glfw
		{

			IGL_INLINE void Viewer::init()
			{

			}

			//IGL_INLINE void Viewer::init_plugins()
			//{
			//  // Init all plugins
			//  for (unsigned int i = 0; i<plugins.size(); ++i)
			//  {
			//    plugins[i]->init(this);
			//  }
			//}

			//IGL_INLINE void Viewer::shutdown_plugins()
			//{
			//  for (unsigned int i = 0; i<plugins.size(); ++i)
			//  {
			//    plugins[i]->shutdown();
			//  }
			//}

			IGL_INLINE Viewer::Viewer() : Movable(),
				data_list(1),
				selected_data_index(0),
				next_data_id(1),
				overlay(true),slice_z(0.5), collided(true)
			{
				data_list.front().id = 0;



				// Temporary variables initialization
			   // down = false;
			  //  hack_never_moved = true;
				scroll_position = 0.0f;

				// Per face
				data().set_face_based(false);


#ifndef IGL_VIEWER_VIEWER_QUIET
				const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
				);
				std::cout << usage << std::endl;
#endif
			}

			IGL_INLINE Viewer::~Viewer()
			{
			}

			IGL_INLINE bool Viewer::load_mesh_from_file(
				const std::string& mesh_file_name_string)
			{

				// Create new data slot and set to selected
				if (!(data().F.rows() == 0 && data().V.rows() == 0))
				{
					append_mesh();
				}
				data().clear();

				size_t last_dot = mesh_file_name_string.rfind('.');
				if (last_dot == std::string::npos)
				{
					std::cerr << "Error: No file extension found in " <<
						mesh_file_name_string << std::endl;
					return false;
				}

				std::string extension = mesh_file_name_string.substr(last_dot + 1);

				if (extension == "off" || extension == "OFF")
				{
					Eigen::MatrixXd V;
					Eigen::MatrixXi F;
					if (!igl::readOFF(mesh_file_name_string, V, F))
						return false;


					data().set_mesh(V, F);

				}
				else if (extension == "obj" || extension == "OBJ")
				{
					Eigen::MatrixXd corner_normals;
					Eigen::MatrixXi fNormIndices;

					Eigen::MatrixXd UV_V;
					Eigen::MatrixXi UV_F;
					Eigen::MatrixXd V;
					Eigen::MatrixXi F;

					if (!(
						igl::readOBJ(
							mesh_file_name_string,
							V, UV_V, corner_normals, F, UV_F, fNormIndices)))
					{
						return false;
					}

					data().set_mesh(V, F);
					data().set_uv(UV_V, UV_F);
					

				}
				else if (extension == "mesh" || extension == "MESH") {
					Eigen::MatrixXd V;
					Eigen::MatrixXi F;
					Eigen::MatrixXi T;
					if (!igl::readMESH(mesh_file_name_string, V, T, F))
						return false;

					/*if (!igl::readOFF(mesh_file_name_string, V, F))
						return false;*/

						// Encapsulated call to point_mesh_squared_distance to determine bounds
					{
						Eigen::VectorXd sqrD;
						Eigen::VectorXi I;
						Eigen::MatrixXd C;
						igl::point_mesh_squared_distance(V, V, F, sqrD, I, C);
						data().max_distance = sqrt(sqrD.maxCoeff());
					}
					//data().T = T;
					data().set_mesh(V, F);
				}
				else
				{
					// unrecognized file type
					printf("Error: %s is not a recognized file type.\n", extension.c_str());
					return false;
				}

				data().compute_normals();
				data().uniform_colors(Eigen::Vector3d(51.0 / 255.0, 43.0 / 255.0, 33.3 / 255.0),
					Eigen::Vector3d(255.0 / 255.0, 228.0 / 255.0, 58.0 / 255.0),
					Eigen::Vector3d(255.0 / 255.0, 235.0 / 255.0, 80.0 / 255.0));

				// Alec: why?
				if (data().V_uv.rows() == 0)
				{
					data().grid_texture();
				}


				//for (unsigned int i = 0; i<plugins.size(); ++i)
				//  if (plugins[i]->post_load())
				//    return true;

				
				
		


				return true;
			}

			IGL_INLINE bool Viewer::save_mesh_to_file(
				const std::string& mesh_file_name_string)
			{
				// first try to load it with a plugin
				//for (unsigned int i = 0; i<plugins.size(); ++i)
				//  if (plugins[i]->save(mesh_file_name_string))
				//    return true;

				size_t last_dot = mesh_file_name_string.rfind('.');
				if (last_dot == std::string::npos)
				{
					// No file type determined
					std::cerr << "Error: No file extension found in " <<
						mesh_file_name_string << std::endl;
					return false;
				}
				std::string extension = mesh_file_name_string.substr(last_dot + 1);
				if (extension == "off" || extension == "OFF")
				{
					return igl::writeOFF(
						mesh_file_name_string, data().V, data().F);
				}
				else if (extension == "obj" || extension == "OBJ")
				{
					Eigen::MatrixXd corner_normals;
					Eigen::MatrixXi fNormIndices;

					Eigen::MatrixXd UV_V;
					Eigen::MatrixXi UV_F;

					return igl::writeOBJ(mesh_file_name_string,
						data().V,
						data().F,
						corner_normals, fNormIndices, UV_V, UV_F);
				}
				else
				{
					// unrecognized file type
					printf("Error: %s is not a recognized file type.\n", extension.c_str());
					return false;
				}
				return true;
			}

			IGL_INLINE bool Viewer::load_scene()
			{
				std::string fname = igl::file_dialog_open();
				if (fname.length() == 0)
					return false;
				return load_scene(fname);
			}

			IGL_INLINE bool Viewer::load_scene(std::string fname)
			{
				// igl::deserialize(core(),"Core",fname.c_str());
				igl::deserialize(data(), "Data", fname.c_str());
				return true;
			}

			IGL_INLINE bool Viewer::save_scene()
			{
				std::string fname = igl::file_dialog_save();
				if (fname.length() == 0)
					return false;
				return save_scene(fname);
			}

			IGL_INLINE bool Viewer::save_scene(std::string fname)
			{
				//igl::serialize(core(),"Core",fname.c_str(),true);
				igl::serialize(data(), "Data", fname.c_str());

				return true;
			}

			IGL_INLINE void Viewer::open_dialog_load_mesh()
			{
				std::string fname = igl::file_dialog_open();

				if (fname.length() == 0)
					return;

				this->load_mesh_from_file(fname.c_str());
			}

			IGL_INLINE void Viewer::open_dialog_save_mesh()
			{
				std::string fname = igl::file_dialog_save();

				if (fname.length() == 0)
					return;

				this->save_mesh_to_file(fname.c_str());
			}

			IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
			{
				assert(!data_list.empty() && "data_list should never be empty");
				int index;
				if (mesh_id == -1)
					index = selected_data_index;
				else
					index = mesh_index(mesh_id);

				assert((index >= 0 && index < data_list.size()) &&
					"selected_data_index or mesh_id should be in bounds");
				return data_list[index];
			}

			IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
			{
				assert(!data_list.empty() && "data_list should never be empty");
				int index;
				if (mesh_id == -1)
					index = selected_data_index;
				else {
					index = mesh_index(mesh_id);
				}
				assert((index >= 0 && index < data_list.size()) &&
					"selected_data_index or mesh_id should be in bounds");
				return data_list[index];
			}

			IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
			{
				assert(data_list.size() >= 1);

				data_list.emplace_back();
				selected_data_index = data_list.size() - 1;
				data_list.back().id = next_data_id++;
				//if (visible)
				//    for (int i = 0; i < core_list.size(); i++)
				//        data_list.back().set_visible(true, core_list[i].id);
				//else
				//    data_list.back().is_visible = 0;
				return data_list.back().id;
			}

			IGL_INLINE bool Viewer::erase_mesh(const size_t index)
			{
				assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
				assert(data_list.size() >= 1);
				if (data_list.size() == 1)
				{
					// Cannot remove last mesh
					return false;
				}
				data_list[index].meshgl.free();
				data_list.erase(data_list.begin() + index);
				if (selected_data_index >= index && selected_data_index > 0)
				{
					selected_data_index--;
				}

				return true;
			}

			IGL_INLINE size_t Viewer::mesh_index(const int id) const {
				for (size_t i = 0; i < data_list.size(); ++i)
				{
					if (data_list[i].id == id)
						return i;
				}
				return 0;
			}

			void Viewer::anim()
			{

				if (data_list.size() == 2) {
					if(!collided)
						collided = ifCollide(data_list[0], data_list[1]);
					
					
					if (!collided)
						data_list[1].MyTranslate(Eigen::Vector3f(-0.01, 0, 0));
				}

			}

			bool Viewer::ifCollide(ViewerData& data1, ViewerData& data2)
			{
				return ifCollide(data1, data2,data1.MakeTrans(), &data1.tree, data2.MakeTrans(), &data2.tree);
			}

			bool Viewer::ifCollide(ViewerData& data1, ViewerData& data2, Eigen::Matrix4f trans1, igl::AABB<Eigen::MatrixXd, 3> *tree1,  Eigen::Matrix4f trans2, igl::AABB<Eigen::MatrixXd, 3> *tree2)
			{
				bool this_box_collide = ifCollide(tree1->m_box, tree2->m_box, trans1, trans2);
				if (this_box_collide) {
					
					//if (false) {//testing


						bool son_collide = false;
						if (tree1->m_left != nullptr && tree2->m_left != nullptr) {
							son_collide = ifCollide(data1, data2, trans1, tree1->m_left, trans2, tree2->m_left);
							if (son_collide) return true;
						}

						if (tree1->m_right != nullptr && tree2->m_left != nullptr) {
							son_collide = ifCollide(data1, data2, trans1, tree1->m_right, trans2, tree2->m_left);
							if (son_collide) return true;
						}
						if (tree1->m_right != nullptr && tree2->m_right != nullptr) {
							son_collide = ifCollide(data1, data2, trans1, tree1->m_right, trans2, tree2->m_right);
							if (son_collide) return true;
						}
						if (tree1->m_left != nullptr && tree2->m_right != nullptr) {
							son_collide = ifCollide(data1, data2, trans1, tree1->m_left, trans2, tree2->m_right);
							if (son_collide) return true;
						}


						//only of trees has null leaf
						if (tree1->m_left == nullptr && tree1->m_right == nullptr && tree2->m_right != nullptr) {
							son_collide = ifCollide(data1, data2, trans1, tree1, trans2, tree2->m_right);
							if (son_collide) return true;
						}
						if (tree1->m_left == nullptr && tree1->m_right == nullptr && tree2->m_left != nullptr) {
							son_collide = ifCollide(data1, data2, trans1, tree1, trans2, tree2->m_left);
							if (son_collide) return true;
						}

						if (tree2->m_left == nullptr && tree2->m_right == nullptr && tree1->m_right != nullptr) {
							son_collide = ifCollide(data1, data2, trans1, tree2, trans2, tree1->m_right);
							if (son_collide) return true;
						}
						if (tree2->m_left == nullptr && tree2->m_right == nullptr && tree1->m_left != nullptr) {
							son_collide = ifCollide(data1, data2, trans1, tree2, trans2, tree1->m_left);
							if (son_collide) return true;
						}
					//}
						if (tree1->m_left == nullptr && tree2->m_left == nullptr && tree1->m_right == nullptr && tree2->m_right == nullptr) {
								data1.rec_tree_boxes(tree1, Eigen::RowVector3d(1, 0, 0), 1);
								data2.rec_tree_boxes(tree2, Eigen::RowVector3d(0, 0, 1), 1);
								bool test = ifCollide(tree1->m_box, tree2->m_box, trans1, trans2);
								return true;//				
						}
						return false;
						
				}
				else return false;
			}

			bool Viewer::ifCollide(Eigen::AlignedBox<double, 3> box1, Eigen::AlignedBox<double, 3> box2,Eigen::Matrix4f trans1, Eigen::Matrix4f trans2) {
				Eigen::Vector3f c1 = Eigen::Vector3f(box1.center()(0), box1.center()(1), box1.center()(2));
				Eigen::Vector3f c2 = Eigen::Vector3f(box2.center()(0), box2.center()(1), box2.center()(2));
				Eigen::Vector3f center1 = (trans1 * Eigen::Vector4f(c1(0), c1(1), c1(2), 1.0)).block < 3, 1 > (0, 0);
				Eigen::Vector3f center2 = (trans2 * Eigen::Vector4f(c2(0), c2(1), c2(2), 1.0)).block < 3, 1 >(0, 0);
				Eigen::Matrix3f box1Rot = trans1.block<3, 3>(0, 0);
				Eigen::Matrix3f box2Rot = trans2.block<3, 3>(0, 0);
				Eigen::Vector3f D = center2 - center1;
				Eigen::Matrix3f C = box1Rot.transpose() * box2Rot;

				Eigen::Vector3f a((box1.corner(box1.TopRightCeil)(0) - box1.corner(box1.BottomLeftFloor )(0))/2,
					(box1.corner(box1.TopRightCeil)(1) - box1.corner(box1.BottomLeftFloor)(1))/2,
					(box1.corner(box1.TopRightCeil)(2) - box1.corner(box1.BottomLeftFloor)(2))/2);

				Eigen::Vector3f b((box2.corner(box2.TopRightCeil)(0) - box2.corner(box2.BottomLeftFloor)(0)) / 2,
					(box2.corner(box2.TopRightCeil)(1) - box2.corner(box2.BottomLeftFloor)(1)) / 2,
					(box2.corner(box2.TopRightCeil)(2) - box2.corner(box2.BottomLeftFloor)(2)) / 2);

				for (int i = 0; i < 3; i++)
					for (int j = 0; j < 3; j++)
						C(i,j) = std::abs(C(i,j));
			
				//Eigen::Matrix3f tempb = C * b;
				if (a(0) + C(0, 0) * b(0) + C(0, 1) * b(1) + C(0, 2) * b(2) < std::abs(Eigen::Vector3f(1, 0, 0).transpose() * box1Rot * D))//A0
					return false;
				if (a(1) + C(1, 0) * b(0) + C(1, 1) * b(1) + C(1, 2) * b(2) < std::abs(Eigen::Vector3f(0, 1, 0).transpose() * box1Rot * D))//A1
					return false;
				if (a(2) + C(2, 0) * b(0) + C(2, 1) * b(1) + C(2, 2) * b(2) < std::abs(Eigen::Vector3f(0, 0, 1).transpose() * box1Rot * D))//A2
					return false;
				if (b(0) + C(0, 0) * a(0) + C(1, 0) * a(1) + C(2, 0) * a(2) < std::abs(Eigen::Vector3f(1, 0, 0).transpose() * box2Rot * D))//B0
					return false;
				if (b(1) + C(0, 1) * a(0) + C(1, 1) * a(1) + C(2, 1) * a(2) < std::abs(Eigen::Vector3f(0, 1, 0).transpose() * box2Rot * D))//B1
					return false;
				if (b(2) + C(0, 2) * a(0) + C(1, 2) * a(1) + C(2, 2) * a(2) < std::abs(Eigen::Vector3f(0, 0, 1).transpose() * box2Rot * D))//B2
					return false;
				//////////////////////////////A0xB0
				float R0 = a(1) * C(2, 0) + a(2) * C(1, 0);
				float R1 = b(1) * C(0, 2) + b(2) * C(0, 1);
				float R_1 = C(1, 0) * Eigen::Vector3f(0, 0, 1).transpose() * box1Rot * D;
				float R_2 = C(2, 0) * Eigen::Vector3f(0, 1, 0).transpose() * box1Rot * D;
				if (R0+R1  < std::abs(R_1 - R_2))
					return false;
				////////////////////////////////////A0xB1
				 R0 = a(1) * C(2, 1) + a(2) * C(1, 1);
				 R1 = b(0) * C(0, 2) + b(2) * C(0, 0);
				 R_1 = C(1, 1) * Eigen::Vector3f(0, 0, 1).transpose() * box1Rot * D;
				 R_2 = C(2, 1) * Eigen::Vector3f(0, 1, 0).transpose() * box1Rot * D;
				if (R0 + R1 < std::abs(R_1 - R_2))
					return false;
				//////////////////////////////////////A0xB2
				 R0 = a(1) * C(2, 2) + a(2) * C(1, 2);
				 R1 = b(0) * C(0, 1) + b(1) * C(0, 0);
				 R_1 = C(1, 2) * Eigen::Vector3f(0, 0, 1).transpose() * box1Rot * D;
				 R_2 = C(2, 2) * Eigen::Vector3f(0, 1, 0).transpose() * box1Rot * D;
				if (R0 + R1 < std::abs(R_1 - R_2))
					return false;
				//////////////////////////////A1xB0
				R0 = a(0) * C(2, 0) + a(2) * C(0, 0);
				R1 = b(1) * C(1, 2) + b(2) * C(1, 1);
				R_1 = C(2, 0) * Eigen::Vector3f(1, 0, 0).transpose() * box1Rot * D;
				R_2 = C(0, 0) * Eigen::Vector3f(0, 0, 1).transpose() * box1Rot * D;
				if (R0 + R1 < std::abs(R_1 - R_2))
					return false;

				//////////////////////////////////A1xB1
				R0 = a(0) * C(2, 1) + a(2) * C(0, 1);
				R1 = b(0) * C(1, 2) + b(2) * C(1, 0);
				R_1 = C(2, 1) * Eigen::Vector3f(1, 0, 0).transpose() * box1Rot * D;
				R_2 = C(0, 1) * Eigen::Vector3f(0, 0, 1).transpose() * box1Rot * D;
				if (R0 + R1 < std::abs(R_1 - R_2))
					return false;
				//////////////////////////////////////A1xB2
				R0 = a(0) * C(2, 2) + a(2) * C(0, 2);
				R1 = b(0) * C(1, 1) + b(1) * C(1, 0);
				R_1 = C(2, 2) * Eigen::Vector3f(1, 0, 0).transpose() * box1Rot * D;
				R_2 = C(0, 2) * Eigen::Vector3f(0, 0, 1).transpose() * box1Rot * D;
				if (R0 + R1 < std::abs(R_1 - R_2))
					return false;

				//////////////////////////////A2xB0
				R0 = a(0) * C(1, 0) + a(1) * C(0, 0);
				R1 = b(1) * C(2, 2) + b(2) * C(2, 1);
				R_1 = C(0, 0) * Eigen::Vector3f(0, 1, 0).transpose() * box1Rot * D;
				R_2 = C(1, 0) * Eigen::Vector3f(1, 0, 0).transpose() * box1Rot * D;
				if (R0 + R1 < std::abs(R_1 - R_2))
					return false;
				//////////////////////////////////A2xB1
				R0 = a(0) * C(1, 1) + a(1) * C(0, 0);
				R1 = b(0) * C(2, 2) + b(2) * C(2, 0);
				R_1 = C(0, 1) * Eigen::Vector3f(0, 1, 0).transpose() * box1Rot * D;
				R_2 = C(1, 1) * Eigen::Vector3f(1, 0, 0).transpose() * box1Rot * D;
				if (R0 + R1 < std::abs(R_1 - R_2))
					return false;
				//////////////////////////////////////A2xB2
				R0 = a(0) * C(1, 2) + a(1) * C(0, 2);
				R1 = b(0) * C(2, 1) + b(1) * C(2, 0);
				R_1 = C(0, 2) * Eigen::Vector3f(0, 1, 0).transpose() * box1Rot * D;
				R_2 = C(1, 2) * Eigen::Vector3f(1, 0, 0).transpose() * box1Rot * D;
				if (R0 + R1 < std::abs(R_1 - R_2))
					return false;
				return true;
			}

		

		} // end namespace
	} // end namespace
}

