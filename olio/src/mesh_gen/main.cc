// ======================================================================
// Olio: Simple renderer
// Copyright (C) 2022 by Hadi Fadaifard
//
// Author: Hadi Fadaifard, 2022
// ======================================================================

//! \file       main.cc
//! \brief      main.cc file for olio mesh_gen
//! \author     Hadi Fadaifard, 2022

#include <vector>
#include <iostream>
#include <boost/program_options.hpp>
#include <spdlog/spdlog.h>

#include "core/types.h"
#include "core/node.h"
#include "core/utils/segfault_handler.h"
#include "core/geometry/trimesh.h"

using namespace olio::core;
using namespace std;
namespace po = boost::program_options;

bool ParseArguments(int argc, char **argv, uint *sphere_type, uint *grid_nx,
                    uint *grid_ny, std::string *output_name) {
  po::options_description desc("options");
  try {
    desc.add_options()
      ("help,h", "print usage")
      ("sphere_type,s",
       po::value             (sphere_type)->required(),
       "Sphere type: 0: with texture coordinates (not watertight), "
       "1: watertight (without texture coordinates)")
      ("grid_nx,m",
       po::value             (grid_nx)->default_value(10),
       "grid_nx: number of grid's horizontal cells")
      ("grid_ny,n",
       po::value             (grid_ny)->default_value(10),
       "grid_ny: number of grid's vertical cells")
      ("output,o",
       po::value             (output_name)->required(),
       "Output mesh name");

    // parse arguments
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) {
      cout << desc << endl;
      return false;
    }
    po::notify(vm);
  } catch(std::exception &e) {
    cout << desc << endl;
    spdlog::error("{}", e.what());
    return false;
  } catch(...) {
    cout << desc << endl;
    spdlog::error("Invalid arguments");
    return false;
  }
  return true;
}


//! \brief Build a sphere mesh without seam
//! \param[in] center Sphere center
//! \param[in] radius Sphere radius
//! \param[in] grid_nx number of longitudinal (horizontal) division for grid defining sphere
//! \param[in] grid_ny number of latitudinal (vertical) division for grid defining sphere
//! \return Smart pointer to create mesh
TriMesh::Ptr
BuildWatertightSphere(const Vec3r &center, Real radius, uint grid_nx,
                      uint grid_ny)
{
  // ======================================================================
  // *** Homework: Implement function
  // ======================================================================
  // ***** START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
    // Conditions of Manifold mesh:
  //  (1) every edge shared by exactly 2 faces
  //  (2) every vertex has a complete loop of triangles around it
  if (grid_nx < 1 || grid_ny < 1){
    return nullptr;
  }
  
  TriMesh::Ptr mesh = TriMesh::Create();
  // add vertices
  mesh->request_vertex_normals();
  mesh->request_face_normals();
  MatXi vertex_indices{grid_ny -1, grid_nx + 1};
  Real d_phi = k2Pi/ grid_nx; // theta steps from 0 to pi
  Real d_theta = kPi / grid_ny; // phi steps from 0 to 2*pi

  Vec3r top{center[0], center[1], center[2]+ radius*1};
  auto v0 = mesh->add_vertex(top);
  
  for(uint j = 0; j <= grid_ny -2; ++j){
    for(uint i = 0; i<= grid_nx + 1; ++i){
      // add mesh vertices
      Real phi = d_phi * i;
      phi = phi >= 0 ? phi : (phi + k2Pi);
      Real theta = d_theta * (j+1);
      Real x = center[0] + radius * cos(phi) * sin(theta);
      Real y = center[1] + radius * sin(phi) * sin(theta);
      Real z = center[2] + radius * cos(theta);

      Vec3r point{x,y,z};
      auto vh = mesh->add_vertex(point);
      vertex_indices(j,i) = vh.idx();

    }
  }
  Vec3r bottom{center[0], center[1], center[2] - radius*1};
  auto v1 = mesh->add_vertex(bottom);
  
  

  // bottom and top levels will be missing
  for(uint j = 0; j < grid_ny - 2; ++j){
    for(uint i = 0; i < grid_nx + 1; ++i){
      vector<TriMesh::VertexHandle> face1{3};
      face1[0] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j,(i+1)%(grid_nx+1))));
      face1[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j,i)));
      face1[2] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j+1,i)));
      mesh->add_face(face1);
      face1.clear();

      vector<TriMesh::VertexHandle> face2{3};
      face2[0] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j,(i+1)%(grid_nx+1))));
      face2[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j+1,i)));
      face2[2] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j+1,(i+1)%(grid_nx+1))));
      mesh->add_face(face2);
      face2.clear();
      
    }
  }
  for(uint i = 0; i< grid_nx + 1; ++i){
    vector<TriMesh::VertexHandle> face3{3};
    face3[0] = v1;
    face3[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(grid_ny-2,(i+1)%(grid_nx+1))));
    face3[2] = mesh->vertex_handle(static_cast<uint>(vertex_indices(grid_ny-2,i)));
    mesh->add_face(face3);
    face3.clear();
  }
  


  for(uint i = 0; i< grid_nx + 1; ++i){
    vector<TriMesh::VertexHandle> face{3};
    face[0] = v0;
    face[2] = mesh->vertex_handle(static_cast<uint>(vertex_indices(0,(i+1)%(grid_nx+1))));
    face[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(0,i)));
    mesh->add_face(face);
    face.clear();

    // face[0] = v1;
    // face[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(grid_ny-2,(i+1)%(grid_nx+1))));
    // face[2] = mesh->vertex_handle(static_cast<uint>(vertex_indices(grid_ny-2,i)));
    // mesh->add_face(face);
    // face.clear();

    // vector<TriMesh::VertexHandle> face4{3};
    // face4[0] = v1;
    // face4[2] = mesh->vertex_handle(static_cast<uint>(vertex_indices(grid_ny-1,i+1)));
    // face4[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(grid_ny-1,i)));
    // mesh->add_face(face4);
    // face4.clear();

  }
  

  

  //make pole caps. 
  // for(uint i = 0; i< grid_nx + 1; ++i){
  //     vector<TriMesh::VertexHandle> face{3};
  //     // top
  //     face[2] = v0;
  //     face[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(0,(i+1)%(grid_nx+1))));
  //     face[0] = mesh->vertex_handle(static_cast<uint>(vertex_indices(0,i)));
      
  //     mesh->add_face(face);
  //     face.clear();
  //     // bottom
  //     face[2] = v1;
  //     face[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(grid_ny-1,i)));
  //     face[0] = mesh->vertex_handle(static_cast<uint>(vertex_indices(grid_ny-1,(i+1)%(grid_nx+1))));
  //     mesh->add_face(face);
  //     face.clear();
  // }
  mesh->ComputeFaceNormals();
  mesh->ComputeVertexNormals();

  return mesh;
  // return nullptr;  //!< remove this line and add your own code
  // ***** END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
}


//! \brief Build a sphere mesh with texture coordinates
//! \param[in] center Sphere center
//! \param[in] radius Sphere radius
//! \param[in] grid_nx number of longitudinal (horizontal) division for grid defining sphere
//! \param[in] grid_ny number of latitudinal (vertical) division for grid defining sphere
//! \return Smart pointer to create mesh
TriMesh::Ptr
BuildSphereWithTexCoords(const Vec3r &center, Real radius, uint grid_nx,
                         uint grid_ny)
{
  // ======================================================================
  // *** Homework: Implement function
  // ======================================================================
  // ***** START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
  // allocate 
  if (grid_nx < 1 || grid_ny < 1){
    return nullptr;
  }
  TriMesh::Ptr mesh = TriMesh::Create();
  
  // add vertices
  mesh->request_vertex_texcoords2D();
  mesh->request_vertex_normals();
  mesh->request_face_normals();
  if( !mesh->has_vertex_texcoords2D()){
    spdlog::error("failed to allocate vertex texcoords2d for mesh");
    return nullptr;
  }
  MatXi vertex_indices{grid_ny + 1, grid_nx + 1};
  // Real theta_start = 0;
  // Real phi_start = 0;
  Real d_phi = k2Pi/grid_nx; // sector count
  Real d_theta = kPi/grid_ny; // stack count
  
  for(uint j = 0; j <= grid_ny; ++j){
    for(uint i = 0; i<= grid_nx; ++i){
      // add mesh vertices
      Real phi = d_phi * i; // around
      phi = phi >= 0 ? phi : (phi + k2Pi);
      Real theta = d_theta * j; // up and down
      Real x = center[0] + radius * cos(phi) * sin(theta);
      Real y = center[1] + radius * sin(phi) * sin(theta);
      Real z = center[2] + radius * cos(theta);

      Vec3r point{x,y,z};
      auto vh = mesh->add_vertex(point);
      vertex_indices(j,i) = vh.idx();

      Real u = phi/ k2Pi;
      Real v = theta/ kPi;
      // cout << Vec2r{u,v};
      // set tex coords at each vertex
      mesh->set_texcoord2D(vh, Vec2r{u,v});
    }
  }
  

  // generate triangles
  for(uint j = 0; j < grid_ny; ++j){
    for(uint i = 0; i < grid_nx; ++i){
      //Real theta = d_theta * i;
      vector<TriMesh::VertexHandle> face1{3};
      face1[0] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j,i+1)));
      face1[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j,i)));
      face1[2] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j+1,i)));
      // check if j == top (north pole)
      if(j > 0 ){
        mesh->add_face(face1);
      }
      vector<TriMesh::VertexHandle> face2{3};
      face2[0] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j,i+1)));
      face2[1] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j+1,i)));
      face2[2] = mesh->vertex_handle(static_cast<uint>(vertex_indices(j+1,i+1)));
      // check if j = bottom (south pole)
      if(j != grid_ny-1){
        mesh->add_face(face2);
      }
      face2.clear();
    }
  }
  mesh->ComputeFaceNormals();
  mesh->ComputeVertexNormals();
  return mesh;
  
  // ***** END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
}


int
main(int argc, char **argv)
{
  utils::InstallSegfaultHandler();
  srand(123543);

  // parse command line arguments
  uint sphere_type, grid_nx, grid_ny;
  string output_name;
  if (!ParseArguments(argc, argv, &sphere_type, &grid_nx, &grid_ny,
                      &output_name))
    return -1;

  // Depending on the sphere_type, generate either a watertight
  // sphere or a sphere with texture coordinates. Both spheres should
  // have a radius of 2 and be centered at (1, 2, 3). See hw5 for more
  // details.
  TriMesh::Ptr sphere;
  Real radius = 2.0f;
  Vec3r center{1, 2, 3};
  switch (sphere_type) {
  case 0:
    sphere = BuildSphereWithTexCoords(center, radius, grid_nx, grid_ny);
    break;
  case 1:
    sphere = BuildWatertightSphere(center, radius, grid_nx, grid_ny);
    break;
  default:
    spdlog::error("invalid sphere type.");
    return -1;
  }

  // Save the generated sphere mesh in the file specified by the
  // command line argument
  OpenMesh::IO::Options options{OpenMesh::IO::Options::VertexNormal};
  if (sphere) {
    if (sphere->has_vertex_texcoords2D())
      options = options | OpenMesh::IO::Options::VertexTexCoord;
    sphere->Save(output_name, options);
  }

  // TriMesh::Ptr sphere = BuildWatertightSphere(center, radius, 10, 10);
  sphere->request_vertex_colors();
  for (auto vit = sphere->vertices_begin(); vit != sphere->vertices_end(); ++vit) {
    TriMesh::Color vert_color{255, 255, 255};
    if (sphere->is_boundary(*vit))
      vert_color = TriMesh::Color{255, 0, 0};
    sphere->set_color(*vit, vert_color);
  }
  OpenMesh::IO::write_mesh(*sphere, "sphere.ply", OpenMesh::IO::Options::VertexColor);

  return 0;
}
