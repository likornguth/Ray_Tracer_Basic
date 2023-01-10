// ======================================================================
// Olio: Simple renderer
// Copyright (C) 2022 by Hadi Fadaifard
//
// Author: Hadi Fadaifard, 2022
// ======================================================================

//! \file       trimesh.cc
//! \brief      TriMesh class
//! \author     Hadi Fadaifard, 2022

#include "core/geometry/trimesh.h"
#include <spdlog/spdlog.h>
#include "core/ray.h"
#include "core/material/material.h"
#include "core/geometry/triangle.h"

namespace olio {
namespace core {

using namespace std;
namespace fs=boost::filesystem;

TriMesh::TriMesh(const std::string &name) :
  OMTriMesh{},
  Surface{name}
{
  name_ = name.size() ? name : "TriMesh";
}


// ======================================================================
// *** Homework: Implement unimplemented TriMesh functions here
// ======================================================================
// ***** START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
// TriMesh::Ptr mesh = this; // allocate mesh


bool TriMesh::Hit(const Ray &ray, Real tmin, Real tmax,
           HitRecord &hit_record)
{
  // TriMesh::Ptr mesh = TriMesh::Create();
  // TriMesh::FaceHandle fh;

  bool had_hit = false;
  int id = 0;
  // for each face in mesh:
  for(TriMesh::FaceIter f_it=faces_begin(); f_it!=faces_end();++f_it)
  {
    if(RayFaceHit(*f_it, ray, tmin, tmax, hit_record)){
      tmax = hit_record.GetRayT();
      had_hit = true;
      
    }
  }
  
  return had_hit; 
  
}


bool TriMesh::RayFaceHit(TriMesh::FaceHandle fh, const Ray &ray, Real tmin, Real tmax, HitRecord &hit_record)
{
  auto heh = halfedge_handle(fh); // get the half edge this face points to
  auto vh0 = from_vertex_handle(heh);
  auto vh1 = to_vertex_handle(heh);
  auto vh2 = to_vertex_handle(next_halfedge_handle(heh));
  
  Vec3r p0, p1, p2;
  p0 = point(vh0);
  p1 = point(vh1);
  p2 = point(vh2);
  Vec3r face_normal = (p1-p0).cross(p2 - p0);
  face_normal.normalize();
  
  Real ray_t{0};
  Vec2r uv{0,0};
  if(Triangle::RayTriangleHit(p0, p1, p2, ray, tmin, tmax, ray_t, uv)){
    // UV coordinates of the hit point inside the
  //!             triangle. Can be used to compute the barycentric
  //!             coordinates as: (1 - uv[0] - uv[1], uv[0], uv[1])
    Vec3r barycentric{1-uv[0] - uv[1], uv[0], uv[1]};
    // compute linearly interpolated normal @ hit point
    Vec3r point_normal = barycentric[0] * normal(vh0) + barycentric[1] * normal(vh1) + barycentric[2] * normal(vh2);
    point_normal /= point_normal.norm();
    // Fill hit_record
    // hit_record.SetNormal(ray, point_normal);
    // hit_record.SetSurface(this);
    
    // get id
    int id = fh.idx();
    Vec2r global_uv{-1,-1};
    // calculate interpolated texture coordinates 
    if(has_vertex_texcoords2D()){
        global_uv = barycentric[0]*texcoord2D(vh0) + barycentric[1]*texcoord2D(vh1) + barycentric[2]*texcoord2D(vh2);
        // cout << global_uv;
        // global_uv /= global_uv.norm();
    }
    FaceGeoUV face_geouv{id, uv, global_uv};

    const Vec3r &hit_point = ray.At(ray_t);
    hit_record.SetRayT(ray_t);
    hit_record.SetPoint(hit_point);
    hit_record.SetNormal(ray, point_normal);
    hit_record.SetSurface(GetPtr());
    hit_record.SetFaceGeoUV(face_geouv);
    return true;
    
  }
  else{
    return false;
  }
}

bool TriMesh::Load(const boost::filesystem::path &filepath)
{
   //allocate mesh
  // check to see if file contains vertex normals, face normals, and 2Dtexcoords
  // if no face/vertex norms, compute them

  // first request vertex normal, face normal, and 2dcoord properties
  request_vertex_normals();
  request_face_normals();
  // request_halfedge_texcoords2D();
  request_vertex_texcoords2D();

  // if(!this->has_vertex_normals()){
  //   std::cerr<<"ERROR: Standard vertex property not available! \n";
  //   return false;
  // }
  // if(!this->has_face_normals()){
  //   std::cerr<<"ERROR: Standard face property not available! \n";
  //   return false;
  // }
  // if(!this->has_halfedge_texcoords2D()){
  //   std::cerr<<"ERROR: Standard halfedge property not available! \n";
  //   return false;
  // }
  OpenMesh::IO::Options ropt = OpenMesh::IO::Options::VertexTexCoord|
    OpenMesh::IO::Options::VertexNormal|
    OpenMesh::IO::Options::FaceNormal;
  // TriMesh::Ptr mesh = TriMesh::Create();
  TriMesh mesh = *this;
  std::string pathstring = filepath.string();
  std::string prefix = "../data";
  pathstring.replace(0,2,prefix);
  if(! OpenMesh::IO::read_mesh(*this, pathstring, ropt))
  {
    std::cerr <<"read error\n";
    return false;
  }
  // &mesh = this;
  // Compute face and vertex normals if we don't have them
  if(! ropt.check(OpenMesh::IO::Options::FaceNormal)){
    ComputeFaceNormals();
  }
  if(! ropt.check(OpenMesh::IO::Options::VertexNormal)){
    ComputeVertexNormals();
  }
  if(! ropt.check(OpenMesh::IO::Options::VertexTexCoord)){
    release_vertex_texcoords2D();
  }

  return true;
  


}

bool TriMesh::Save(const boost::filesystem::path &filepath, OpenMesh::IO::Options opts)
{
  TriMesh mesh = *this;
  // OpenMesh::IO::Options wopt;
  // std::string pathstring = filepath.string();
  // std::string prefix = "../data";
  // pathstring.replace(0,2,prefix);
  auto fp = filepath;
  if (!fp.is_absolute())
    fp = boost::filesystem::absolute(fp,
    boost::filesystem::current_path());
  boost::filesystem::path path_prefix(fp.parent_path());
  std::string pathstring = fp.string();

  
  if(! OpenMesh::IO::write_mesh(*this, pathstring, opts))
  {
    std::cerr << "Error";
    return false;
  }

  return true;
}

AABB 
TriMesh::GetBoundingBox(bool force_recompute)
{
    // if bound is clean, just return existing bbox_
    if (!force_recompute && !IsBoundDirty())
      return bbox_;

    // compute bounding box of vertices given in the TriMesh
    bbox_.Reset();
    // Iterate over every vertex and expand bounding box by point coords
    for (auto vit =  vertices_begin(); vit != vertices_end(); ++vit){
      Vec3r point = this->point(*vit);
      bbox_.ExpandBy(point);
    }

    bound_dirty_ = false;
    return bbox_;
  
}

bool TriMesh::ComputeFaceNormals()
{
  // iterate over all faces in mesh
  for (auto fit = faces_begin(); fit != faces_end(); ++fit){
    vector<Vec3r> points;
    points.reserve(3);
    // extract all vertices around face
    for(auto fvit = fv_iter(*fit); fvit.is_valid(); ++fvit){
      points.push_back(point(*fvit));
    }
    // compute flat normal using cross product
    Vec3r face_normal = (points[1]-points[0]).cross(points[2]-points[0]);
    face_normal.normalize();
    set_normal(*fit, face_normal);
  }
  return true;
  
}

bool TriMesh::ComputeVertexNormals()
{
  for ( auto vit =  vertices_begin(); vit != vertices_end(); ++vit){
    //const Vec3r &vertex_normal = VertexNormal(this, *vit);
    Vec3r vertex_normal{0,0,0};
    for(TriMesh::VertexFaceIter vf_it = vf_iter(*vit); vf_it.is_valid();++vf_it){
      // for each adjacent face to vertex vit:
      // sum face normal NOTE we must have face normals computed
      // or already loaded before Computing Vertex Normals
      vertex_normal += normal(*vf_it);
    }
    // take the average
    vertex_normal /= vertex_normal.norm();
    // store averaged vertex normal compuation in vertex
    set_normal(*vit, vertex_normal);
  }
  return true;

}




// ***** END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****

}  // namespace core
}  // namespace olio
