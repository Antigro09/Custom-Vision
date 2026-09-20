// Full GPU mapped-square MultiTag PnP: device IPPE seeds, whole-tag consensus,
// joint planar initialization, robust multi-start and joint LM refinement.
#include "cuda_multitag.hpp"
#include "cuda_multitag_math.hpp"
#include <cuda_runtime.h>
#include <cstring>
#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>
using namespace vision_pose;
using namespace vision_multitag_math;
namespace {
constexpr int max_tags=vision_multitag::kCapacity;
constexpr int max_hypotheses=2*max_tags+6;
struct Workspace {
    double rays[max_tags*8];
    unsigned char input_valid[max_tags],geometry_valid[max_tags];
    Candidate hypotheses[max_hypotheses];
    int hypothesis_valid[max_hypotheses],support[max_hypotheses];
    double score[max_hypotheses];
    unsigned char masks[max_hypotheses][max_tags];
    int robust_start[4];
    Candidate final_pose[max_hypotheses];
    int final_active[max_hypotheses],final_iterations[max_hypotheses],final_count;
    vision_multitag::Result result;
};
void check(cudaError_t e,const char* op){if(e!=cudaSuccess)throw std::runtime_error(std::string("CUDA MultiTag ")+op+": "+cudaGetErrorString(e));}
__global__ void seeds_kernel(Camera camera,const Input* observed,const double* field,int count,double size,Workspace* w){
    const int tag=blockIdx.x,lane=threadIdx.x;if(tag>=count)return;
    __shared__ Input pixels;__shared__ double normalized[8];__shared__ int ok[4];
    if(lane<8){pixels.uv[lane]=observed[tag].uv[lane];normalized[lane]=0;}__syncthreads();
    if(lane<4)ok[lane]=undistort(camera,pixels.uv[2*lane],pixels.uv[2*lane+1],normalized+2*lane);__syncthreads();
    // Failed inversion cannot leave partially written or nonfinite scratch rays
    // in the persistent workspace; input_valid still excludes that entire tag.
    if(lane<8)w->rays[8*tag+lane]=ok[lane/2]?normalized[lane]:0;
    if(lane==0){
        double basis[9],center[3];Result result{};
        w->geometry_valid[tag]=field_basis(field+12*tag,size,basis,center);
        if(ok[0]&&ok[1]&&ok[2]&&ok[3]&&w->geometry_valid[tag])result=solve(camera,pixels,size,0,normalized);
        w->input_valid[tag]=result.valid;
        w->hypothesis_valid[2*tag]=result.valid;w->hypothesis_valid[2*tag+1]=result.valid&&result.has_alternate;
        if(result.valid){w->hypotheses[2*tag]=world_pose(result.best,basis,center);w->hypotheses[2*tag+1]=world_pose(result.alternate,basis,center);}
    }
}
__global__ void global_planar_kernel(Camera camera,const Input* observed,const double* field,int count,double size,Workspace* w){
    if(threadIdx.x||blockIdx.x)return;
    const int base=2*count;Candidate poses[2];int planar=0;
    const bool valid=planar_initialize(camera,observed,w->rays,field,count,w->input_valid,size,poses,planar);
    for(int i=0;i<2;++i){w->hypothesis_valid[base+i]=valid&&vision_pose::finite(poses[i].error);if(w->hypothesis_valid[base+i])w->hypotheses[base+i]=poses[i];}
}
__global__ void score_kernel(Camera camera,const Input* observed,const double* field,int count,double threshold,int offset,int hypotheses,Workspace* w){
    const int h=offset+blockIdx.x,lane=threadIdx.x;if(blockIdx.x>=hypotheses)return;
    int accepted=0;double squared=0;
    const bool valid=w->hypothesis_valid[h];
    const Candidate pose=valid?w->hypotheses[h]:Candidate{};
    for(int first=0;first<count;first+=8){const int tag=first+lane/4,corner=lane%4;double e=0;int ok=0;
        if(tag<count&&valid&&w->input_valid[tag]){double uv[2];ok=project(camera,pose,field+12*tag+3*corner,uv);
            if(ok){const double dx=uv[0]-observed[tag].uv[2*corner],dy=uv[1]-observed[tag].uv[2*corner+1];e=dx*dx+dy*dy;}}
        for(int delta=2;delta;delta/=2){e+=__shfl_down_sync(0xffffffff,e,delta,4);ok+=__shfl_down_sync(0xffffffff,ok,delta,4);}
        if(corner==0&&tag<count){const bool keep=ok==4&&vision_pose::finite(e)&&e<=4*threshold*threshold;w->masks[h][tag]=keep;
            if(keep){++accepted;squared+=e*.25;}}
    }
    for(int delta=16;delta;delta/=2){accepted+=__shfl_down_sync(0xffffffff,accepted,delta);squared+=__shfl_down_sync(0xffffffff,squared,delta);}
    if(lane==0){w->support[h]=accepted;w->score[h]=accepted?::sqrt(squared/accepted):HUGE_VAL;}
}
__global__ void choose_robust_kernel(int count,Workspace* w){
    if(threadIdx.x||blockIdx.x)return;const int nh=2*count+2;int total=0,best=-1;
    for(int i=0;i<count;++i)total+=w->input_valid[i];
    for(int h=0;h<nh;++h)if(w->hypothesis_valid[h]&&(best<0||w->support[h]>w->support[best]||(w->support[h]==w->support[best]&&w->score[h]<w->score[best])))best=h;
    for(int k=0;k<4;++k){w->robust_start[k]=-1;w->hypothesis_valid[nh+k]=0;}
    if(best<0||w->support[best]==total)return;
    // Keep the four strongest distinct seed indices. Original starts
    // remain in the consensus pool if robust refinement makes their fit worse.
    for(int k=0;k<4;++k){int chosen=-1;
        for(int h=0;h<nh;++h)if(w->hypothesis_valid[h]){bool used=false;for(int p=0;p<k;++p)if(w->robust_start[p]==h)used=true;
            if(!used&&(chosen<0||w->support[h]>w->support[chosen]||(w->support[h]==w->support[chosen]&&w->score[h]<w->score[chosen])))chosen=h;}
        w->robust_start[k]=chosen;
    }
}
__device__ double block_sum(double value,double* partial){
    const int lane=threadIdx.x;for(int offset=16;offset;offset/=2)value+=__shfl_down_sync(0xffffffff,value,offset);
    if(lane%32==0)partial[lane/32]=value;__syncthreads();const double result=partial[0]+partial[1];__syncthreads();return result;
}
__device__ double centered_tag_error(Camera camera,const Candidate& pose,const Input& pixels,const double* field,const double* center){
    double e=0;for(int j=0;j<4;++j){double xyz[3],uv[2];for(int k=0;k<3;++k)xyz[k]=field[3*j+k]-center[k];
        if(!project(camera,pose,xyz,uv))return HUGE_VAL;const double dx=uv[0]-pixels.uv[2*j],dy=uv[1]-pixels.uv[2*j+1];e+=dx*dx+dy*dy;}
    return e*.25;
}
__device__ double cost_and_weights(Camera camera,const Candidate& pose,const Input* observed,const double* field,int count,
                                   const unsigned char* mask,const double* center,double threshold,bool robust,double* weights,double* partial){
    double cost=0;const double c2=threshold*threshold;
    for(int tag=threadIdx.x;tag<count;tag+=64){double weight=0;if(mask[tag]){
        const double e=centered_tag_error(camera,pose,observed[tag],field+12*tag,center);
        if(robust){if(vision_pose::finite(e)){weight=1/(1+e/c2);cost+=c2*::log1p(e/c2);}else cost+=1e12*c2;}
        else {weight=vision_pose::finite(e)?1:0;cost+=e;}}
        if(weights)weights[tag]=weight;
    }
    return block_sum(cost,partial);
}
// Each block jointly refines one start. All accepted corners contribute to one
// normal equation; this is not an average of separately estimated tag poses.
__global__ void refine_kernel(Camera camera,const Input* observed,const double* field,int count,double threshold,int iterations,bool robust,Workspace* w){
    const int h=blockIdx.x,lane=threadIdx.x,nh=2*count+2;
    if(robust){if(h>=4||w->robust_start[h]<0)return;}
    else if(h>=w->final_count||!w->final_active[h])return;
    __shared__ Candidate pose,next;__shared__ double center[3],weights[max_tags],J[128*6],residual[128],A[36],g[6],partial[2],lambda,old_cost;
    __shared__ int done,attempts;
    const unsigned char* mask=robust?w->input_valid:w->result.accepted;
    if(lane==0){pose=robust?w->hypotheses[w->robust_start[h]]:w->final_pose[h];
        int used=0;center[0]=center[1]=center[2]=0;
        for(int t=0;t<count;++t)if(mask[t])for(int j=0;j<4;++j){++used;for(int k=0;k<3;++k)center[k]+=field[12*t+3*j+k];}
        for(int k=0;k<3;++k)center[k]/=used?used:1;
        for(int r=0;r<3;++r)pose.t[r]+=dot(pose.R+3*r,center,3);
        lambda=1e-3;done=!used;attempts=0;}
    __syncthreads();
    for(int it=0;it<iterations;++it){if(done)break;
        const double cost=cost_and_weights(camera,pose,observed,field,count,mask,center,threshold,robust,weights,partial);
        if(lane==0){old_cost=cost;attempts=it+1;if(!vision_pose::finite(cost))done=1;}__syncthreads();if(done)break;
        double accumulation=0;
        for(int first=0;first<4*count;first+=64){const int point=first+lane,tag=point/4,corner=point%4;
            double jac[12]={0},uv[2]={0},res[2]={0};
            if(tag<count&&weights[tag]>0){double xyz[3];for(int k=0;k<3;++k)xyz[k]=field[12*tag+3*corner+k]-center[k];
                if(project(camera,pose,xyz,uv,jac)){const double weight=::sqrt(weights[tag]);
                    for(int k=0;k<12;++k)jac[k]*=weight;
                    for(int axis=0;axis<2;++axis)res[axis]=(observed[tag].uv[2*corner+axis]-uv[axis])*weight;}}
            for(int axis=0;axis<2;++axis){residual[2*lane+axis]=res[axis];for(int col=0;col<6;++col)J[(2*lane+axis)*6+col]=jac[axis*6+col];}
            __syncthreads();
            // The last tile can contain far fewer than 64 real corners (eight
            // for a two-tag fit). Its remaining Jacobian rows are identically
            // zero; do not spend FP64 work accumulating that padding every LM
            // iteration. Every actual corner retains the same reduction order.
            const int rows=2*min(64,4*count-first);
            if(lane<36){const int row=lane/6,col=lane%6;for(int k=0;k<rows;++k)accumulation+=J[k*6+row]*J[k*6+col];}
            else if(lane<42){const int row=lane-36;for(int k=0;k<rows;++k)accumulation+=J[k*6+row]*residual[k];}
            __syncthreads();
        }
        if(lane<36)A[lane]=accumulation;else if(lane<42)g[lane-36]=accumulation;__syncthreads();
        if(lane==0){for(int k=0;k<6;++k)A[k*7]+=lambda*maxv(A[k*7],1e-12);double step[6]={0};
            if(!linear<6>(A,g,step)||dot(step,step,6)<1e-20)done=1;else next=increment(pose,step);}
        __syncthreads();if(done)break;
        const double candidate_cost=cost_and_weights(camera,next,observed,field,count,mask,center,threshold,robust,nullptr,partial);
        if(lane==0){if(vision_pose::finite(candidate_cost)&&candidate_cost<old_cost){pose=next;lambda=maxv(1e-12,lambda*.3);if(old_cost-candidate_cost<1e-10)done=1;}
            else {lambda*=10;if(lambda>1e12)done=1;}}
        __syncthreads();
    }
    if(lane==0){for(int r=0;r<3;++r)pose.t[r]-=dot(pose.R+3*r,center,3);
        pose.error=joint_error(camera,pose,observed,field,count,mask);
        if(robust){w->hypotheses[nh+h]=pose;w->hypothesis_valid[nh+h]=1;}
        else {w->final_pose[h]=pose;w->final_active[h]=vision_pose::finite(pose.error);w->final_iterations[h]=attempts;}}
}
__global__ void final_seed_kernel(Camera camera,const Input* observed,const double* field,int count,double size,Workspace* w){
    if(threadIdx.x||blockIdx.x)return;
    auto& result=w->result;result=vision_multitag::Result{};result.best.error=result.alternate.error=HUGE_VAL;w->final_count=0;
    for(int t=0;t<count;++t)if(!w->geometry_valid[t]){result.reason=vision_multitag::invalid_input;return;}
    int best=-1;const int nh=2*count+6;
    for(int h=0;h<nh;++h)if(w->hypothesis_valid[h]&&(best<0||w->support[h]>w->support[best]||(w->support[h]==w->support[best]&&w->score[h]<w->score[best])))best=h;
    if(best<0||w->support[best]<2){result.reason=vision_multitag::no_consensus;return;}
    result.inlier_count=w->support[best];for(int t=0;t<count;++t)result.accepted[t]=w->masks[best][t];
    Candidate planar[2];int coplanar=0;
    const bool initialized=planar_initialize(camera,observed,w->rays,field,count,result.accepted,size,planar,coplanar);result.coplanar=coplanar;
    if(coplanar){if(!initialized){result.reason=vision_multitag::initialization_failed;return;}
        if(planar[1].error<planar[0].error){Candidate swap=planar[0];planar[0]=planar[1];planar[1]=swap;}
        result.best=planar[0];result.alternate=planar[1];result.initial_error=planar[0].error;result.initial_alternate_error=planar[1].error;
        result.has_alternate=vision_pose::finite(planar[1].error);
        result.ambiguity=result.has_alternate?(planar[1].error<1e-9?1:minv(1,planar[0].error/planar[1].error)):0;
        w->final_pose[0]=planar[0];w->final_active[0]=vision_pose::finite(planar[0].error);w->final_count=1;
    }else{
        // Retain every valid tag-derived start for the nonplanar joint solve;
        // selecting only near-duplicate low-error starts could miss a real mode.
        w->final_count=nh;for(int h=0;h<nh;++h){w->final_active[h]=0;if(w->hypothesis_valid[h]){
            // The first fixed-mask refinement step already checks all corner
            // depths and residuals. Do not repeat that O(tags^2) projection work
            // serially here; every start is still checked by its own GPU block.
            w->final_pose[h]=w->hypotheses[h];w->final_active[h]=1;}}
    }
    result.reason=vision_multitag::initialization_failed;
}
__global__ void finalize_kernel(Camera camera,const Input* observed,const double* field,int count,double threshold,Workspace* w,vision_multitag::Result* output){
    if(threadIdx.x||blockIdx.x)return;auto& r=w->result;int best=-1;
    for(int h=0;h<w->final_count;++h)if(w->final_active[h]&&(best<0||w->final_pose[h].error<w->final_pose[best].error))best=h;
    if(best>=0){r.best=w->final_pose[best];r.iterations=w->final_iterations[best];
        if(!r.coplanar){r.has_alternate=0;r.ambiguity=0;r.initial_error=r.best.error;r.initial_alternate_error=HUGE_VAL;
            int alternate=-1;for(int h=0;h<w->final_count;++h)if(w->final_active[h]&&h!=best&&!same_pose(r.best,w->final_pose[h])&&
                (alternate<0||w->final_pose[h].error<w->final_pose[alternate].error))alternate=h;
            if(alternate>=0){r.alternate=w->final_pose[alternate];r.has_alternate=1;r.initial_alternate_error=r.alternate.error;
                r.ambiguity=r.alternate.error<1e-9?1:minv(1,r.best.error/r.alternate.error);}}
        r.valid=1;r.reason=vision_multitag::success;
        for(int t=0;t<count;++t){r.tag_errors[t]=::sqrt(tag_squared_error(camera,r.best,observed[t],field+12*t));
            if(r.accepted[t]&&(!vision_pose::finite(r.tag_errors[t])||r.tag_errors[t]>threshold)){r.valid=0;r.reason=vision_multitag::reprojection_error;}}
    }
    *output=r;
}
} // namespace
struct CudaMultiTag::Impl {
    Camera camera;double size;bool mapped=false;
    Input *host_observed=nullptr,*device_observed=nullptr;
    double *host_field=nullptr,*device_field=nullptr;
    vision_multitag::Result *host_result=nullptr,*device_result=nullptr;
    Workspace* work=nullptr;cudaStream_t stream=nullptr;cudaEvent_t begin=nullptr,end=nullptr,done=nullptr;float timing=0;
    // A bounded per-instance cache handles changing visible tag counts without
    // rebuilding a graph every frame or sharing mutable work across cameras.
    struct Graph {cudaGraphExec_t exec=nullptr;std::size_t count=0;double threshold=0;int iterations=0;std::uint64_t used=0;};
    std::array<Graph,8> graphs{};std::uint64_t clock=0;
    void enqueue(std::size_t count,double threshold,int iterations);
    cudaGraphExec_t executable(std::size_t count,double threshold,int iterations);
    Impl(const Camera& c,double s):camera(c),size(s){}
    ~Impl(){cudaSetDevice(0);if(stream)cudaStreamSynchronize(stream);
        for(auto& graph:graphs)if(graph.exec)cudaGraphExecDestroy(graph.exec);
        if(begin)cudaEventDestroy(begin);if(end)cudaEventDestroy(end);if(done)cudaEventDestroy(done);if(work)cudaFree(work);
        if(device_observed)cudaFree(device_observed);if(device_field)cudaFree(device_field);
        if(!mapped&&device_result)cudaFree(device_result);
        if(host_observed)cudaFreeHost(host_observed);if(host_field)cudaFreeHost(host_field);if(host_result)cudaFreeHost(host_result);if(stream)cudaStreamDestroy(stream);}
};
void CudaMultiTag::Impl::enqueue(std::size_t count,double threshold,int iterations){
    auto& p=*this;
    check(cudaMemcpyAsync(p.device_observed,p.host_observed,count*sizeof(Input),cudaMemcpyHostToDevice,p.stream),"upload observations");
    check(cudaMemcpyAsync(p.device_field,p.host_field,count*12*sizeof(double),cudaMemcpyHostToDevice,p.stream),"upload field corners");
    check(cudaEventRecordWithFlags(p.begin,p.stream,cudaEventRecordExternal),"record start");const int n=static_cast<int>(count),nh=2*n+2;
    seeds_kernel<<<n,32,0,p.stream>>>(p.camera,p.device_observed,p.device_field,n,p.size,p.work);check(cudaGetLastError(),"initialize tag seeds");
    global_planar_kernel<<<1,1,0,p.stream>>>(p.camera,p.device_observed,p.device_field,n,p.size,p.work);check(cudaGetLastError(),"initialize joint plane");
    score_kernel<<<nh,32,0,p.stream>>>(p.camera,p.device_observed,p.device_field,n,threshold,0,nh,p.work);check(cudaGetLastError(),"score seeds");
    choose_robust_kernel<<<1,1,0,p.stream>>>(n,p.work);check(cudaGetLastError(),"select robust starts");
    refine_kernel<<<4,64,0,p.stream>>>(p.camera,p.device_observed,p.device_field,n,threshold,iterations,true,p.work);check(cudaGetLastError(),"refine robust starts");
    score_kernel<<<4,32,0,p.stream>>>(p.camera,p.device_observed,p.device_field,n,threshold,nh,4,p.work);check(cudaGetLastError(),"score robust starts");
    final_seed_kernel<<<1,1,0,p.stream>>>(p.camera,p.device_observed,p.device_field,n,p.size,p.work);check(cudaGetLastError(),"initialize accepted set");
    refine_kernel<<<nh+4,64,0,p.stream>>>(p.camera,p.device_observed,p.device_field,n,threshold,iterations,false,p.work);check(cudaGetLastError(),"refine accepted set");
    finalize_kernel<<<1,1,0,p.stream>>>(p.camera,p.device_observed,p.device_field,n,threshold,p.work,p.device_result);check(cudaGetLastError(),"finalize joint pose");
    check(cudaEventRecordWithFlags(p.end,p.stream,cudaEventRecordExternal),"record end");
    if(!p.mapped){check(cudaMemcpyAsync(p.host_result,p.device_result,sizeof(vision_multitag::Result),cudaMemcpyDeviceToHost,p.stream),"download result");check(cudaEventRecordWithFlags(p.done,p.stream,cudaEventRecordExternal),"record completion");}
}
cudaGraphExec_t CudaMultiTag::Impl::executable(std::size_t count,double threshold,int iterations){
    for(auto& graph:graphs)if(graph.exec&&graph.count==count&&graph.threshold==threshold&&graph.iterations==iterations){
        graph.used=++clock;return graph.exec;
    }
    auto* slot=&graphs[0];
    for(auto& graph:graphs)if(graph.used<slot->used)slot=&graph;
    cudaGraph_t captured=nullptr;cudaGraphExec_t executable=nullptr;bool capturing=false;
    try{
        // Thread-local capture permits independent camera threads to launch or
        // synchronize their own streams while this instance records its graph.
        check(cudaStreamBeginCapture(stream,cudaStreamCaptureModeThreadLocal),"begin solve capture");capturing=true;
        enqueue(count,threshold,iterations);
        const auto status=cudaStreamEndCapture(stream,&captured);capturing=false;check(status,"end solve capture");
        check(cudaGraphInstantiate(&executable,captured,0),"instantiate solve graph");
        check(cudaGraphDestroy(captured),"release captured graph");captured=nullptr;
    }catch(...){
        if(capturing)cudaStreamEndCapture(stream,&captured);
        if(captured)cudaGraphDestroy(captured);
        if(executable)cudaGraphExecDestroy(executable);
        throw;
    }
    // All prior calls finished before reaching here; cached graphs are never
    // evicted while executing. The owning detector also holds its mutex.
    if(slot->exec)cudaGraphExecDestroy(slot->exec);
    *slot=Graph{executable,count,threshold,iterations,++clock};return executable;
}
CudaMultiTag::CudaMultiTag(const Camera& camera,double size):impl_(new Impl(camera,size)){
    if(!vision_pose::finite(size)||size<=0)throw std::invalid_argument("CUDA MultiTag tag size must be positive");
    check(cudaSetDevice(0),"select device");auto& p=*impl_;cudaDeviceProp prop{};check(cudaGetDeviceProperties(&prop,0),"query device");p.mapped=prop.integrated&&prop.canMapHostMemory;
    check(cudaStreamCreateWithFlags(&p.stream,cudaStreamNonBlocking),"create stream");
    check(cudaHostAlloc(reinterpret_cast<void**>(&p.host_observed),max_tags*sizeof(Input),cudaHostAllocDefault),"allocate observations");
    check(cudaHostAlloc(reinterpret_cast<void**>(&p.host_field),max_tags*12*sizeof(double),cudaHostAllocDefault),"allocate field corners");
    check(cudaHostAlloc(reinterpret_cast<void**>(&p.host_result),sizeof(vision_multitag::Result),p.mapped?cudaHostAllocMapped:cudaHostAllocDefault),"allocate result");
    // Unlike single-tag initialization, joint fitting reuses these inputs in
    // many hypotheses and iterations. Orin pinned host memory is GPU-uncached;
    // one bounded upload into persistent device allocations enables GPU caching.
    check(cudaMalloc(reinterpret_cast<void**>(&p.device_observed),max_tags*sizeof(Input)),"allocate cached observations");
    check(cudaMalloc(reinterpret_cast<void**>(&p.device_field),max_tags*12*sizeof(double)),"allocate cached field corners");
    if(p.mapped)check(cudaHostGetDevicePointer(reinterpret_cast<void**>(&p.device_result),p.host_result,0),"map result");
    else {check(cudaMalloc(reinterpret_cast<void**>(&p.device_result),sizeof(vision_multitag::Result)),"allocate device result");
        check(cudaEventCreateWithFlags(&p.done,cudaEventDisableTiming),"create completion event");}
    check(cudaMalloc(reinterpret_cast<void**>(&p.work),sizeof(Workspace)),"allocate persistent workspace");
    check(cudaEventCreate(&p.begin),"create start event");check(cudaEventCreate(&p.end),"create end event");
}
CudaMultiTag::~CudaMultiTag()=default;
const vision_multitag::Result* CudaMultiTag::solve(const Input* observed,const double* field,std::size_t count,double threshold,int iterations){
    if(count<2||count>max_tags)throw std::invalid_argument("CUDA MultiTag requires 2..256 complete tags");
    if(!observed||!field)throw std::invalid_argument("CUDA MultiTag inputs must not be null");
    if(!vision_pose::finite(threshold)||threshold<=0||iterations<1||iterations>100)throw std::invalid_argument("CUDA MultiTag requires positive error threshold and 1..100 iterations");
    auto& p=*impl_;p.timing=0;check(cudaSetDevice(0),"select device");
    std::memcpy(p.host_observed,observed,count*sizeof(Input));std::memcpy(p.host_field,field,count*12*sizeof(double));
    try{
        // Replay the same transfers, kernels, quality checks and timing events
        // with one host launch. No hypothesis or refinement work is removed.
        check(cudaGraphLaunch(p.executable(count,threshold,iterations),p.stream),"launch solve graph");
        check(cudaEventSynchronize(p.mapped?p.end:p.done),"finish solve");check(cudaEventElapsedTime(&p.timing,p.begin,p.end),"read timing");
    }catch(...){cudaStreamSynchronize(p.stream);throw;}
    return p.host_result;
}
float CudaMultiTag::kernel_ms()const{return impl_->timing;}
