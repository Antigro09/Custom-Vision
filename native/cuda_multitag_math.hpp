#pragma once
#include "pnp_math.hpp"
#ifdef __CUDACC__
#define MT_HD __host__ __device__
#else
#define MT_HD
#endif
namespace vision_multitag_math {
using namespace vision_pose;
MT_HD inline bool field_basis(const double* p,double size,double* basis,double* center){
    for(int k=0;k<12;++k)if(!vision_pose::finite(p[k]))return false;
    double x[3],y[3],z[3];
    for(int k=0;k<3;++k){center[k]=.25*(p[k]+p[3+k]+p[6+k]+p[9+k]);x[k]=(p[3+k]-p[k])/size;y[k]=(p[k]-p[9+k])/size;}
    if(::fabs(dot(x,x,3)-1)>1e-5||::fabs(dot(y,y,3)-1)>1e-5||::fabs(dot(x,y,3))>1e-5)return false;
    cross(x,y,z);
    for(int r=0;r<3;++r){basis[r*3]=x[r];basis[r*3+1]=y[r];basis[r*3+2]=z[r];}
    for(int i=0;i<4;++i){double q[3];object_point(i,size,q);for(int r=0;r<3;++r)
        if(::fabs(center[r]+dot(basis+3*r,q,3)-p[3*i+r])>1e-5*size)return false;}
    return true;
}
MT_HD inline Candidate world_pose(const Candidate& local,const double* basis,const double* center){
    Candidate world=local;
    for(int r=0;r<3;++r)for(int c=0;c<3;++c)world.R[3*r+c]=dot(local.R+3*r,basis+3*c,3);
    for(int r=0;r<3;++r)world.t[r]=local.t[r]-dot(world.R+3*r,center,3);
    return world;
}
MT_HD inline double tag_squared_error(const Camera& camera,const Candidate& pose,const Input& pixels,const double* xyz){
    double sum=0;for(int i=0;i<4;++i){double uv[2];if(!project(camera,pose,xyz+3*i,uv))return HUGE_VAL;
        const double dx=uv[0]-pixels.uv[2*i],dy=uv[1]-pixels.uv[2*i+1];sum+=dx*dx+dy*dy;}
    return sum*.25;
}
MT_HD inline double joint_error(const Camera& camera,const Candidate& pose,const Input* pixels,const double* xyz,int count,const unsigned char* mask){
    double sum=0;int accepted=0;for(int i=0;i<count;++i)if(mask[i]){const double e=tag_squared_error(camera,pose,pixels[i],xyz+12*i);if(!vision_pose::finite(e))return HUGE_VAL;sum+=e;++accepted;}
    return accepted?::sqrt(sum/accepted):HUGE_VAL;
}
// IPPE rotation reconstruction using the Jacobian of a centered metric plane.
// Same Collins/Bartoli formulation and attribution as pnp_math.hpp.
MT_HD inline bool rotations_from_homography(const double* H,double scale,Candidate* poses){
    const double p=H[2],q=H[5],j00=(H[0]-p*H[6])/scale,j01=(H[1]-p*H[7])/scale;
    const double j10=(H[3]-q*H[6])/scale,j11=(H[4]-q*H[7])/scale;
    const double norm=::sqrt(p*p+q*q+1),vx=p/norm,vy=q/norm,vz=1/norm,s=1/(1+vz);
    const double Rv[9]={1-vx*vx*s,-vx*vy*s,vx,-vx*vy*s,1-vy*vy*s,vy,-vx,-vy,vz};
    const double b00=Rv[0]-p*Rv[6],b01=Rv[1]-p*Rv[7],b10=Rv[3]-q*Rv[6],b11=Rv[4]-q*Rv[7];
    const double det=b00*b11-b01*b10;if(!vision_pose::finite(det)||::fabs(det)<1e-15)return false;
    const double a00=(b11*j00-b01*j10)/det,a01=(b11*j01-b01*j11)/det;
    const double a10=(-b10*j00+b00*j10)/det,a11=(-b10*j01+b00*j11)/det;
    const double aa=a00*a00+a01*a01,ab=a00*a10+a01*a11,bb=a10*a10+a11*a11;
    const double gamma=::sqrt(.5*(aa+bb+::sqrt((aa-bb)*(aa-bb)+4*ab*ab)));
    if(!vision_pose::finite(gamma)||gamma<1e-12)return false;
    const double a=a00/gamma,b=a01/gamma,c=a10/gamma,d=a11/gamma;
    const double z0=::sqrt(maxv(0,1-a*a-c*c));double z1=::sqrt(maxv(0,1-b*b-d*d));if(-a*b-c*d<0)z1=-z1;
    for(int branch=0;branch<2;++branch){const double sign=branch?-1:1,col0[3]={a,c,sign*z0},col1[3]={b,d,sign*z1};double col2[3];cross(col0,col1,col2);
        for(int r=0;r<3;++r){poses[branch].R[3*r]=dot(Rv+3*r,col0,3);poses[branch].R[3*r+1]=dot(Rv+3*r,col1,3);poses[branch].R[3*r+2]=dot(Rv+3*r,col2,3);}}
    return true;
}
// Joint planar fit across all corners of complete accepted tags. Its two raw
// hypotheses are kept independent for ambiguity; LM must not collapse them.
MT_HD inline bool planar_initialize(const Camera& camera,const Input* pixels,const double* rays,const double* xyz,int count,
                                    const unsigned char* mask,double size,Candidate* poses,int& planar){
    planar=0;int first=-1,ntags=0;double center[3]={0,0,0},basis[9],unused[3];
    for(int t=0;t<count;++t)if(mask[t]){if(first<0)first=t;++ntags;for(int j=0;j<4;++j)for(int k=0;k<3;++k)center[k]+=xyz[12*t+3*j+k];}
    if(ntags<2||!field_basis(xyz+12*first,size,basis,unused))return false;
    for(int k=0;k<3;++k)center[k]/=4*ntags;
    double spread=0;
    for(int t=0;t<count;++t)if(mask[t])for(int j=0;j<4;++j){double delta[3];for(int k=0;k<3;++k)delta[k]=xyz[12*t+3*j+k]-center[k];
        const double z=delta[0]*basis[2]+delta[1]*basis[5]+delta[2]*basis[8];if(::fabs(z)>1e-7)return false;
        spread+=dot(delta,delta,3);}
    planar=1;const double scale=::sqrt(spread/(4*ntags));if(!vision_pose::finite(scale)||scale<1e-12)return false;
    double A[64]={0},b[8]={0},H[9]={0},mean_u=0,mean_v=0;
    for(int t=0;t<count;++t)if(mask[t])for(int j=0;j<4;++j){double delta[3];for(int k=0;k<3;++k)delta[k]=xyz[12*t+3*j+k]-center[k];
        const double x=(delta[0]*basis[0]+delta[1]*basis[3]+delta[2]*basis[6])/scale;
        const double y=(delta[0]*basis[1]+delta[1]*basis[4]+delta[2]*basis[7])/scale;
        const double u=rays[8*t+2*j],v=rays[8*t+2*j+1];if(!vision_pose::finite(u)||!vision_pose::finite(v))return false;
        mean_u+=u;mean_v+=v;
        const double rows[16]={x,y,1,0,0,0,-u*x,-u*y,0,0,0,x,y,1,-v*x,-v*y};
        for(int axis=0;axis<2;++axis)for(int r=0;r<8;++r){const double value=rows[axis*8+r];b[r]+=value*(axis?v:u);
            for(int c=0;c<8;++c)A[r*8+c]+=value*rows[axis*8+c];}}
    if(!linear<8>(A,b,H))return false;H[8]=1;
    if(!rotations_from_homography(H,scale,poses))return false;
    mean_u/=4*ntags;mean_v/=4*ntags;
    for(int branch=0;branch<2;++branch){double sx=0,sy=0,numerator=0,denominator=0;
        for(int t=0;t<count;++t)if(mask[t])for(int j=0;j<4;++j){double delta[3];for(int k=0;k<3;++k)delta[k]=xyz[12*t+3*j+k]-center[k];
            const double p[3]={delta[0]*basis[0]+delta[1]*basis[3]+delta[2]*basis[6],delta[0]*basis[1]+delta[1]*basis[4]+delta[2]*basis[7],0};
            double rotated[3];for(int r=0;r<3;++r)rotated[r]=dot(poses[branch].R+3*r,p,3);
            const double u=rays[8*t+2*j],v=rays[8*t+2*j+1],du=u-mean_u,dv=v-mean_v;
            const double rx=u*rotated[2]-rotated[0],ry=v*rotated[2]-rotated[1];sx+=rx;sy+=ry;numerator-=du*rx+dv*ry;denominator+=du*du+dv*dv;}
        if(!vision_pose::finite(denominator)||denominator<1e-30)return false;
        poses[branch].t[2]=numerator/denominator;poses[branch].t[0]=sx/(4*ntags)+mean_u*poses[branch].t[2];poses[branch].t[1]=sy/(4*ntags)+mean_v*poses[branch].t[2];
        poses[branch]=world_pose(poses[branch],basis,center);poses[branch].error=joint_error(camera,poses[branch],pixels,xyz,count,mask);}
    return vision_pose::finite(poses[0].error)||vision_pose::finite(poses[1].error);
}
MT_HD inline bool same_pose(const Candidate& a,const Candidate& b){
    double rotation_distance=0;for(int k=0;k<9;++k){const double d=a.R[k]-b.R[k];rotation_distance+=d*d;}
    double center_distance=0;for(int k=0;k<3;++k){double ca=0,cb=0;for(int r=0;r<3;++r){ca-=a.R[3*r+k]*a.t[r];cb-=b.R[3*r+k]*b.t[r];}const double d=ca-cb;center_distance+=d*d;}
    return rotation_distance<2e-8&&center_distance<1e-8;
}
}
#undef MT_HD
