// Optional full single-tag CUDA PnP. IPPE initialization and LM execute on GPU.
// A warp per tag cooperates on projection Jacobians and the normal equations.
#include "cuda_pose.hpp"
#include <cuda_runtime.h>
#include <cstring>
#include <stdexcept>
#include <string>
using namespace vision_pose;
namespace {
void check(cudaError_t e,const char* operation) {
    if(e!=cudaSuccess)throw std::runtime_error(std::string("CUDA PnP ")+operation+": "+cudaGetErrorString(e));
}
__global__ void solve_kernel(Camera camera,const Input* input,Result* output,int count,double size,int iterations) {
    const int tag=blockIdx.x,lane=threadIdx.x;
    if(tag>=count)return;
    __shared__ Result r;
    __shared__ Input observed;
    __shared__ double normalized[8],J[48],residual[8],A[36],g[6],lambda;
    __shared__ int ok[4],done;
    // Mapped Tegra host memory is inexpensive for this tiny, single-pass input;
    // cache it once before iterative reuse, rather than repeatedly loading it.
    if(lane<8)observed.uv[lane]=input[tag].uv[lane];
    __syncthreads();
    // Distortion inversion is independent for each corner; do all four in
    // parallel instead of serializing their Newton/backtracking iterations.
    if(lane<4)ok[lane]=undistort(camera,observed.uv[2*lane],observed.uv[2*lane+1],normalized+2*lane);
    __syncthreads();
    if(lane==0) {
        // solve(...,0) performs dual IPPE initialization with the GPU-inverted
        // rays, while retaining all input/convexity and cheirality checks.
        if(ok[0]&&ok[1]&&ok[2]&&ok[3])r=solve(camera,observed,size,0,normalized);
        else {r=Result{};r.reason=2;r.ambiguity=1;r.best.error=r.alternate.error=HUGE_VAL;}
        lambda=1e-3;done=!r.valid;
    }
    __syncthreads();
    for(int iteration=0;iteration<iterations;++iteration) {
        if(done)break;
        if(lane<4) {
            double p[3],uv[2],jac[12];object_point(lane,size,p);
            ok[lane]=project(camera,r.best,p,uv,jac);
            for(int axis=0;axis<2;++axis){
                residual[2*lane+axis]=ok[lane]?observed.uv[2*lane+axis]-uv[axis]:0;
                for(int col=0;col<6;++col)J[(2*lane+axis)*6+col]=ok[lane]?jac[axis*6+col]:0;
            }
        }
        __syncthreads();
        for(int index=lane;index<36;index+=32) {
            const int row=index/6,col=index%6;double sum=0;
            for(int k=0;k<8;++k)sum+=J[k*6+row]*J[k*6+col];
            A[index]=sum+(row==col?lambda*maxv(sum,1e-12):0);
        }
        if(lane<6){double sum=0;for(int k=0;k<8;++k)sum+=J[k*6+lane]*residual[k];g[lane]=sum;}
        __syncthreads();
        if(lane==0) {
            double step[6]={0};r.iterations=iteration+1;
            if(!ok[0]||!ok[1]||!ok[2]||!ok[3]||!linear<6>(A,g,step)||dot(step,step,6)<1e-20)done=1;
            else {
                Candidate next=increment(r.best,step);next.error=error(camera,observed,size,next);
                if(vision_pose::finite(next.error)&&next.error<r.best.error) {
                    double improvement=r.best.error-next.error;r.best=next;lambda=maxv(1e-12,lambda*.3);
                    if(improvement<1e-10)done=1;
                } else {lambda*=10;if(lambda>1e12)done=1;}
            }
        }
        __syncthreads();
    }
    if(lane==0)output[tag]=r;
}
} // namespace
struct CudaPoseBatch::Impl {
    Camera camera;double size;int iterations;
    Input *host_input=nullptr,*device_input=nullptr;
    Result *host_output=nullptr,*device_output=nullptr;
    cudaStream_t stream=nullptr;
    cudaEvent_t start=nullptr,end=nullptr,completed=nullptr;
    bool mapped=false;
    float kernel=0;
    Impl(const Camera& c,double s,int i):camera(c),size(s),iterations(i){}
    ~Impl() {
        cudaSetDevice(0);
        if(stream)cudaStreamSynchronize(stream);
        if(start)cudaEventDestroy(start);if(end)cudaEventDestroy(end);if(completed)cudaEventDestroy(completed);
        if(!mapped){if(device_input)cudaFree(device_input);if(device_output)cudaFree(device_output);}
        if(host_input)cudaFreeHost(host_input);if(host_output)cudaFreeHost(host_output);
        if(stream)cudaStreamDestroy(stream);
    }
};
CudaPoseBatch::CudaPoseBatch(const Camera& camera,double size,int iterations):impl_(new Impl(camera,size,iterations)) {
    if(iterations<1||iterations>100)throw std::invalid_argument("cuda_pose_iterations must be 1..100");
    check(cudaSetDevice(0),"select device");
    auto& p=*impl_;
    cudaDeviceProp properties{};check(cudaGetDeviceProperties(&properties,0),"query device");
    p.mapped=properties.integrated&&properties.canMapHostMemory;
    check(cudaStreamCreateWithFlags(&p.stream,cudaStreamNonBlocking),"create stream");
    if(p.mapped) {
        // Orin's CPU and GPU share physical memory. Copies add launch/driver
        // overhead without improving locality for these small pose buffers.
        check(cudaHostAlloc(reinterpret_cast<void**>(&p.host_input),capacity()*sizeof(Input),cudaHostAllocMapped),"allocate mapped inputs");
        check(cudaHostAlloc(reinterpret_cast<void**>(&p.host_output),capacity()*sizeof(Result),cudaHostAllocMapped),"allocate mapped outputs");
        check(cudaHostGetDevicePointer(reinterpret_cast<void**>(&p.device_input),p.host_input,0),"map inputs");
        check(cudaHostGetDevicePointer(reinterpret_cast<void**>(&p.device_output),p.host_output,0),"map outputs");
    } else {
        check(cudaMalloc(reinterpret_cast<void**>(&p.device_input),capacity()*sizeof(Input)),"allocate inputs");
        check(cudaMalloc(reinterpret_cast<void**>(&p.device_output),capacity()*sizeof(Result)),"allocate outputs");
        check(cudaMallocHost(reinterpret_cast<void**>(&p.host_input),capacity()*sizeof(Input)),"allocate pinned inputs");
        check(cudaMallocHost(reinterpret_cast<void**>(&p.host_output),capacity()*sizeof(Result)),"allocate pinned outputs");
        check(cudaEventCreateWithFlags(&p.completed,cudaEventDisableTiming),"create completion event");
    }
    // No cudaEventBlockingSync: cudaEventSynchronize waits actively for these
    // short solves, independent of Tegra's platform-dependent stream sync policy.
    // This applies only to our completion events, not global CUDA scheduling.
    check(cudaEventCreate(&p.start),"create start event");check(cudaEventCreate(&p.end),"create end event");
}
CudaPoseBatch::~CudaPoseBatch()=default;
const Result* CudaPoseBatch::solve(const Input* inputs,std::size_t count) {
    if(count>capacity())throw std::invalid_argument("CUDA PnP batch exceeds capacity");
    auto& p=*impl_;p.kernel=0;if(!count)return p.host_output;
    if(!inputs)throw std::invalid_argument("CUDA PnP nonempty batch requires input");
    check(cudaSetDevice(0),"select device");
    std::memcpy(p.host_input,inputs,count*sizeof(Input));
    try {
        if(!p.mapped)check(cudaMemcpyAsync(p.device_input,p.host_input,count*sizeof(Input),cudaMemcpyHostToDevice,p.stream),"upload");
        check(cudaEventRecord(p.start,p.stream),"record start");
        solve_kernel<<<static_cast<unsigned>(count),32,0,p.stream>>>(p.camera,p.device_input,p.device_output,static_cast<int>(count),p.size,p.iterations);
        check(cudaGetLastError(),"launch");
        check(cudaEventRecord(p.end,p.stream),"record end");
        if(!p.mapped) {
            check(cudaMemcpyAsync(p.host_output,p.device_output,count*sizeof(Result),cudaMemcpyDeviceToHost,p.stream),"download");
            check(cudaEventRecord(p.completed,p.stream),"record completion");
        }
        check(cudaEventSynchronize(p.mapped?p.end:p.completed),"finish batch");
        check(cudaEventElapsedTime(&p.kernel,p.start,p.end),"read kernel timing");
    } catch(...) {cudaStreamSynchronize(p.stream);throw;}
    return p.host_output;
}
float CudaPoseBatch::kernel_ms() const { return impl_->kernel; }
int CudaPoseBatch::device_count() {int n=0;return cudaGetDeviceCount(&n)==cudaSuccess?n:0;}
