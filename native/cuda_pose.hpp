#pragma once
#include "pnp_math.hpp"
#include <cstddef>
#include <memory>
// One instance/stream per native camera detector. Call only under its mutex.
class CudaPoseBatch {
public:
    explicit CudaPoseBatch(const vision_pose::Camera& camera,double size,int iterations=30);
    ~CudaPoseBatch();
    CudaPoseBatch(const CudaPoseBatch&)=delete;
    CudaPoseBatch& operator=(const CudaPoseBatch&)=delete;
    // Up to capacity() results, owned until the next solve. Empty batches do no work.
    const vision_pose::Result* solve(const vision_pose::Input* inputs,std::size_t count);
    static constexpr std::size_t capacity() { return 256; }
    float kernel_ms() const;
    static int device_count();
private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
