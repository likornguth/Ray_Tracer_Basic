//! \file       ray.h
//! \brief      Ray class
//! \author     Hadi Fadaifard, 2022

#pragma once

#include <memory>
#include <string>
#include "core/types.h"

namespace olio{
namespace core{

class FaceGeoUV {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FaceGeoUV() = default;

    FaceGeoUV(int face_id, const Vec2r &uv, const Vec2r &global_uv);

    inline void SetFaceID(int face_id){face_id_ = face_id;}

    inline void SetUV(const Vec2r &uv){uv_ = uv;}

    inline void SetGlobalUV(const Vec2r &global_uv){global_uv_ = global_uv;}

    inline int GetFaceID() const {return face_id_;}

    inline Vec2r GetUV() const {return uv_;}

    inline Vec2r GetGlobalUV() const {return global_uv_;}



protected:
    int face_id_;
    Vec2r uv_;
    Vec2r global_uv_;

};


} // namespace core
} // namespace olio