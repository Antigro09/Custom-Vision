#pragma once
#include "pnp_math.hpp"
#include <cstddef>
#include <memory>

namespace vision_multitag {
constexpr std::size_t kCapacity=256;
enum Reason { success=0, invalid_input=1, insufficient_tags=2, no_consensus=3,
              initialization_failed=4, reprojection_error=5 };
// best/alternate transform field-coordinate XYZ into OpenCV camera XYZ.
// accepted indexes the input observations, never individual corners.
struct Result {
    vision_pose::Candidate best{},alternate{};
    double ambiguity=1,initial_error=0,initial_alternate_error=0;
    int valid=0,reason=invalid_input,inlier_count=0,coplanar=0,has_alternate=0,iterations=0;
    unsigned char accepted[kCapacity]{};
    double tag_errors[kCapacity]{};
};
}

// One independent stream and persistent workspace per camera. Serialize calls
// on each instance. All PnP initialization, consensus and refinement run on GPU.
class CudaMultiTag {
public:
    explicit CudaMultiTag(const vision_pose::Camera& camera,double tag_size);
    ~CudaMultiTag();
    CudaMultiTag(const CudaMultiTag&)=delete;
    CudaMultiTag& operator=(const CudaMultiTag&)=delete;
    // field_corners is contiguous [count,4,3], in the same decoded corner order
    // as observations. Result storage remains owned until the next call.
    const vision_multitag::Result* solve(const vision_pose::Input* observations,
                                        const double* field_corners,std::size_t count,
                                        double max_error_px,int iterations=30);
    static constexpr std::size_t capacity(){return vision_multitag::kCapacity;}
    float kernel_ms() const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
