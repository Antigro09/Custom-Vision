#include "pnp_math.hpp"
extern "C" int vision_pose_solve(const double* camera,const double* uv,double size,int count,double* out) {
    vision_pose::Camera c{camera[0],camera[1],camera[2],camera[3],{}};
    for(int i=0;i<8;++i)c.d[i]=camera[4+i];
    vision_pose::Input input{};for(int i=0;i<8;++i)input.uv[i]=uv[i];
    auto r=vision_pose::solve(c,input,size,count);
    for(int i=0;i<9;++i)out[i]=r.best.R[i];
    for(int i=0;i<3;++i)out[9+i]=r.best.t[i];
    out[12]=r.best.error;out[13]=r.ambiguity;out[14]=r.initial_error;out[15]=r.initial_alternate_error;
    out[16]=r.iterations;out[17]=r.has_alternate;
    for(int i=0;i<9;++i)out[18+i]=r.alternate.R[i];
    for(int i=0;i<3;++i)out[27+i]=r.alternate.t[i];
    return r.valid;
}
extern "C" int vision_pose_project(const double* camera,const double* R,const double* t,const double* p,double* uv,double* J) {
    vision_pose::Camera c{camera[0],camera[1],camera[2],camera[3],{}};for(int i=0;i<8;++i)c.d[i]=camera[4+i];
    vision_pose::Candidate pose{};for(int i=0;i<9;++i)pose.R[i]=R[i];for(int i=0;i<3;++i)pose.t[i]=t[i];
    return vision_pose::project(c,pose,p,uv,J);
}
extern "C" int vision_pose_homography(const double* normalized_corners,double* H) {
    return vision_pose::square_homography(normalized_corners,H);
}
