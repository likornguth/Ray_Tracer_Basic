// ======================================================================
// Olio: Simple renderer
// Copyright (C) 2022 by Hadi Fadaifard
//
// Author: Hadi Fadaifard, 2022
// ======================================================================

//! \file       phong_material.cc
//! \brief      PhongMaterial class
//! \author     Hadi Fadaifard, 2022

#include "core/material/phong_material.h"
#include <spdlog/spdlog.h>
#include "core/light/light.h"
#include "core/texture/texture.h"
#include "core/texture/image_texture.h"

namespace olio {
namespace core {

using namespace std;

PhongMaterial::PhongMaterial(const std::string &name) :
  Material{}
{
  name_ = name.size() ? name : "PhongMaterial";
  SetDiffuse(Vec3r{0, 0, 0});
}


PhongMaterial::PhongMaterial(const Vec3r &ambient, const Vec3r &diffuse,
                             const Vec3r &specular, Real shininess,
                             const Vec3r &mirror, const std::string &name) :
  Material{},
  ambient_{ambient},
  specular_{specular},
  shininess_{shininess},
  mirror_{mirror}
{
  name_ = name.size() ? name : "PhongMaterial";
  SetDiffuse(diffuse);
}


void
PhongMaterial::SetDiffuse(const Vec3r &diffuse)
{
  // diffuse_ = diffuse;
  diffuse_ = SolidTexture::Create(diffuse);
}

void
PhongMaterial::SetDiffuse(Texture::Ptr diffuse){
  diffuse_ = diffuse;
}


Vec3r
PhongMaterial::Evaluate(const HitRecord &hit_record, const Vec3r &light_vec,
                        const Vec3r &view_vec) const
{
  // Blinn-Phong halfway vector formulation
  const Vec3r &normal = hit_record.GetNormal();
  Vec3r half = (view_vec + light_vec).normalized();
  auto half_dot = fmax(0, half.dot(normal));
  Real specular_falloff = pow(half_dot, shininess_);
  Vec3r specular_color = specular_falloff * specular_;

  // handle backfacing surfaces
  // Vec3r diffuse_color = diffuse_;
  // auto surface = hit_record.GetSurface();
  Vec3r position = hit_record.GetPoint();
  Vec3r diffuse_color{0,0,0};
  if(diffuse_){
    const auto &uv = hit_record.GetFaceGeoUV().GetGlobalUV();
    // cout << uv;
    diffuse_color = diffuse_->Value(uv, position);

  }
  
  // Vec2r uv = fg_uv->GetUV();
  
  // Vec3r diffuse_color = diffuse_->Value(uv, position);


  if (!hit_record.IsFrontFace()) {
    diffuse_color = Vec3r{1, 1, 0};
    specular_color = Vec3r{0, 0, 0};
  }

  return specular_color + diffuse_color;
}

}  // namespace core
}  // namespace olio
