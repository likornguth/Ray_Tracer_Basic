// ======================================================================
// Olio: Simple renderer
// Copyright (C) 2022 by Hadi Fadaifard
//
// Author: Hadi Fadaifard, 2022
// ======================================================================

//! \file       bvh_node.cc
//! \brief      BVHNode class
//! \author     Hadi Fadaifard, 2022

#include "core/geometry/bvh_node.h"
#include <spdlog/spdlog.h>
#include "core/ray.h"
#include "core/material/material.h"

namespace olio {
namespace core {

using namespace std;

BVHNode::BVHNode(const std::string &name) :
  Surface{}
{
  name_ = name.size() ? name : "BVHNode";
}


AABB
BVHNode::GetBoundingBox(bool force_recompute)
{
  // if bound is clean, just return existing bbox_
  if (!force_recompute && !IsBoundDirty())
    return bbox_;

  bbox_.Reset();
  if (left_)
    bbox_.ExpandBy(left_->GetBoundingBox(force_recompute));
  if (right_)
    bbox_.ExpandBy(right_->GetBoundingBox(force_recompute));
  // spdlog::info("{}: {}", GetName(), bbox_);
  bound_dirty_ = false;
  return bbox_;
}


bool
BVHNode::Hit(const Ray &ray, Real tmin, Real tmax, HitRecord &hit_record)
{
  // ======================================================================
  // *** Homework: Implement function
  // ======================================================================
  // ***** START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
  if(GetBoundingBox().Hit(ray, tmin, tmax)){
    // if leaf: try to intersect surface record hit 
    // adjust tmax if necessary, and return true/false
    if(left_ == nullptr && right_ == nullptr){
      auto surface = hit_record.GetSurface();
      if(surface->Hit(ray, tmin, tmax, hit_record)){
        auto t = hit_record.GetRayT();
        tmax = max(t, tmax);
        return true;
      }
      
      else{
        return false;
      }
    }
    
    // ELSE:
    bool hit = false;
    if ((left_ != nullptr) && (left_->Hit(ray, tmin, tmax, hit_record))){
      tmax = hit_record.GetRayT();
      hit = true;
    }
    if((right_ != nullptr) && (right_->Hit(ray, tmin, tmax, hit_record))){
      hit = true;
    }
    return hit;
  //   HitRecord lrec, rrec;
  //   bool left_hit = (left_ != nullptr) && (left_->Hit(ray, tmin, tmax, lrec));
  //   bool right_hit = (right_ != nullptr) && (right_->Hit(ray, tmin, tmax, rrec));

  //   if(left_hit && right_hit){
  //     if(lrec.GetRayT() < rrec.GetRayT()){
  //       hit_record = lrec;
  //     }
  //     else{
  //       hit_record = rrec;
  //     }
  //     return true;
  //   }
  //   else if(left_hit){
  //     hit_record = lrec;
  //     return true;
  //   }
  //   else if(right_hit){
  //     hit_record = rrec;
  //     return true;
  //   } 
  }
  return false;

  
  //return false;  //!< remove this line and add your own code
  // ***** END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
}


BVHNode::Ptr
BVHNode::BuildBVH(std::vector<Surface::Ptr> surfaces, const string &name)
{
  spdlog::info("Building BVH ({})", name);

  // error checking
  auto surface_count = surfaces.size();
  
  if (!surface_count)
    return nullptr;

  // make sure we have valid bboxes for surfaces
  for (auto surface : surfaces) {
    if (surface)
      surface->GetBoundingBox();
  }

  // build bvh
  uint split_axis = 0;
  auto bvh_node = BuildBVH(surfaces, 0, surface_count, split_axis, name);

  // compute bboxes
  if (bvh_node)
    bvh_node->GetBoundingBox();

  spdlog::info("Done building BVH ({})", name);
  return bvh_node;
}


BVHNode::Ptr
BVHNode::BuildBVH(std::vector<Surface::Ptr> &surfaces, size_t start,
                  size_t end, uint split_axis, const std::string &name)
{
  // ======================================================================
  // *** Homework: Implement function
  // ======================================================================
  // ***** START OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
  BVHNode::Ptr bvh_node = BVHNode::Create();
  size_t N = end-start;
  if(N == 1){
    bvh_node->left_ = surfaces[0];
    bvh_node->right_ = nullptr;
    bvh_node->SetBoundingBox(surfaces[0]->GetBoundingBox());
  }
  else if(N == 2){
    bvh_node->left_= surfaces[0];
    bvh_node->right_= surfaces[1];
    AABB new_bbox = surfaces[0]->GetBoundingBox();
    // bvh_node->bbox_ = BBoxCombine(A[0]->BBox, A[1]->BBox)
    new_bbox.ExpandBy(surfaces[1]->GetBoundingBox());
    bvh_node->SetBoundingBox(new_bbox);
  }
  else {
    // sort A by surface or bbox centers along split_axis
    sort(surfaces.begin(), surfaces.end(), 
        [&](const Surface::Ptr a, const Surface::Ptr b) -> bool
    { 
        // compare by bbox centers along split_axis
        auto bbox_centera = (a->GetBoundingBox().GetMax()[split_axis] + a->GetBoundingBox().GetMin()[split_axis])/2 ;
        auto bbox_centerb = (b->GetBoundingBox().GetMax()[split_axis] + b->GetBoundingBox().GetMin()[split_axis])/2 ;

        return bbox_centera > bbox_centerb; 
    });

    vector<Surface::Ptr> sublist1;
    vector<Surface::Ptr> sublist2;
    for(size_t i = 0; i<N; ++i){
      if(i < N/2){
        sublist1.push_back(surfaces[i]);
      }
      else{
        sublist2.push_back(surfaces[i]);
      }
    }

    bvh_node->left_ = BVHNode::BuildBVH(sublist1, 0, sublist1.size(), (split_axis+1)%3, name);
    bvh_node->right_ = BVHNode::BuildBVH(sublist2, 0, sublist2.size(), (split_axis+1)%3, name);
    AABB new_bbox = bvh_node->left_->GetBoundingBox();
    new_bbox.ExpandBy(bvh_node->right_->GetBoundingBox());
    bvh_node->SetBoundingBox(new_bbox);
  }
  return bvh_node;
  // return nullptr;  //!< remove this line and add your own code
  // ***** END OF YOUR CODE (DO NOT DELETE/MODIFY THIS LINE) *****
}

}  // namespace core
}  // namespace olio
