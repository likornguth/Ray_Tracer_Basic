

//! \file       ray.cc
//! \brief      Ray class
//! \author     Hadi Fadaifard, 2022

#include "core/face_geouv.h"
#include <spdlog/spdlog.h>
#include "core/ray.h"

namespace olio {
namespace core {

using namespace std;

FaceGeoUV::FaceGeoUV(int face_id, const Vec2r &uv, const Vec2r &global_uv) :
  face_id_{face_id},
  uv_{uv},
  global_uv_{global_uv}
{
}


}  // namespace core
}  // namespace olio