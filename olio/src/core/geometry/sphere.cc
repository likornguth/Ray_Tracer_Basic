// ======================================================================
// Olio: Simple renderer
// Copyright (C) 2022 by Hadi Fadaifard
//
// Author: Hadi Fadaifard, 2022
// ======================================================================

//! \file       sphere.cc
//! \brief      Sphere class
//! \author     Hadi Fadaifard, 2022

#include "core/geometry/sphere.h"
#include <spdlog/spdlog.h>
#include "core/ray.h"
#include "core/face_geouv.h"

namespace olio {
namespace core {

Sphere::Sphere(const std::string &name) :
  Surface{}
{
  name_ = name.size() ? name : "Sphere";
}


Sphere::Sphere(const Vec3r &center, Real radius, const std::string &name) :
  Surface{},
  center_{center},
  radius_{radius}
{
  name_ = name.size() ? name : "Sphere";
}


void
Sphere::SetCenter(const Vec3r &center)
{
  center_ = center;
  bound_dirty_ = true;
}


void
Sphere::SetRadius(Real radius)
{
  radius_ = radius;
  bound_dirty_ = true;
}


AABB
Sphere::GetBoundingBox(bool force_recompute)
{
  // if bound is clean, just return existing bbox_
  if (!force_recompute && !IsBoundDirty())
    return bbox_;

  // compute sphere bbox
  bbox_.Reset();
  Vec3r radius3{radius_, radius_, radius_};
  bbox_.ExpandBy(AABB{center_ - radius3, center_ + radius3});
  bound_dirty_ = false;
  return bbox_;
}


bool
Sphere::Hit(const Ray &ray, Real tmin, Real tmax, HitRecord &hit_record)
{
  auto p0 = ray.GetOrigin() - center_;
  auto v = ray.GetDirection();
  auto a = v.squaredNorm();
  auto b = 2 * p0.dot(v);
  auto c = p0.squaredNorm() - radius_ * radius_;

  // solve quadratic equation to find t
  auto a2 = 2 * a;
  auto discriminant = b * b - 2 * a2 * c;
  if (discriminant < 0)
    return false;
  auto s = static_cast<Real>(sqrt(discriminant));
  auto t = (-b - s) / a2;
  if (t < tmin)
    t = (-b + s) / a2;
  if (t < tmin || t > tmax)
    return false;

  // fill hit record
  
  const Vec3r &hit_point = ray.At(t);
  Real x = hit_point[0];
  Real y = hit_point[1];
  Real z = hit_point[2];
  Real phi = atan2(y,x);
  phi = phi >= 0 ? phi : (phi + k2Pi);
  Real theta = acos(z/sqrt(x*x + y*y + z*z));
  
  int id = -1;
  Vec2r uv{-1,-1};
  Vec2r global_uv{(phi/k2Pi),(theta/kPi)};
  // FaceGeoUV face_geouv{id, uv, global_uv};
  FaceGeoUV face_geouv{id, uv, global_uv};


  
  hit_record.SetRayT(t);
  hit_record.SetPoint(hit_point);
  hit_record.SetNormal(ray, (hit_point - center_).normalized());
  hit_record.SetSurface(GetPtr());
  hit_record.SetFaceGeoUV(face_geouv);

  return true;
}

}  // namespace core
}  // namespace olio
