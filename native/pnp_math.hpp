// Small planar PnP math shared by the CUDA kernel and CPU reference tests.
// IPPE rotation reconstruction follows Collins/Bartoli and OpenCV 4.10's
// calib3d/src/ippe.cpp. See IPPE_LICENSE.txt for attribution and BSD terms.
// No OpenCV calls, dynamic allocation, or CPU seed is used in solve().
#pragma once
#include <cmath>
#include <cstddef>
#ifdef __CUDACC__
#define VP_HD __host__ __device__
#else
#define VP_HD
#endif
namespace vision_pose {
struct Camera { double fx, fy, cx, cy, d[8]; };
struct Input { double uv[8]; };
struct Candidate { double R[9], t[3], error; };
struct Result {
    Candidate best, alternate;
    double ambiguity, initial_error, initial_alternate_error;
    int valid, has_alternate, iterations, reason;
};
VP_HD inline bool finite(double x) {
#ifdef __CUDA_ARCH__
    return ::isfinite(x);
#else
    return std::isfinite(x);
#endif
}
VP_HD inline double minv(double a, double b) { return a < b ? a : b; }
VP_HD inline double maxv(double a, double b) { return a > b ? a : b; }
VP_HD inline double dot(const double* a, const double* b, int n) {
    double s=0; for (int i=0;i<n;++i) s+=a[i]*b[i]; return s;
}
VP_HD inline void cross(const double* a,const double* b,double* c) {
    c[0]=a[1]*b[2]-a[2]*b[1]; c[1]=a[2]*b[0]-a[0]*b[2]; c[2]=a[0]*b[1]-a[1]*b[0];
}
// Pivoted elimination. A/b are scratch, x receives the solution.
template<int N> VP_HD inline bool linear(double* A,double* b,double* x) {
    for(int k=0;k<N;++k) {
        int pivot=k; for(int i=k+1;i<N;++i) if(::fabs(A[i*N+k])>::fabs(A[pivot*N+k])) pivot=i;
        if(!finite(A[pivot*N+k]) || ::fabs(A[pivot*N+k])<1e-15) return false;
        if(pivot!=k) {
            for(int j=k;j<N;++j) { double v=A[k*N+j]; A[k*N+j]=A[pivot*N+j]; A[pivot*N+j]=v; }
            double v=b[k];b[k]=b[pivot];b[pivot]=v;
        }
        for(int i=k+1;i<N;++i) {
            double f=A[i*N+k]/A[k*N+k];
            for(int j=k+1;j<N;++j) { A[i*N+j]-=f*A[k*N+j]; }
            b[i]-=f*b[k];
        }
    }
    for(int i=N-1;i>=0;--i) { double v=b[i];for(int j=i+1;j<N;++j)v-=A[i*N+j]*x[j];x[i]=v/A[i*N+i];if(!finite(x[i]))return false; }
    return true;
}
// OpenCV rational Brown-Conrady: k1,k2,p1,p2,k3,k4,k5,k6.
// Optional 2x2 Jacobian of distorted normalized coordinates.
VP_HD inline bool distort(const Camera& c,double x,double y,double* q,double* J=nullptr) {
    const double r=x*x+y*y,r4=r*r;
    const double num=1+c.d[0]*r+c.d[1]*r4+c.d[4]*r4*r;
    const double den=1+c.d[5]*r+c.d[6]*r4+c.d[7]*r4*r;
    if(!finite(num)||!finite(den)||::fabs(den)<1e-12) return false;
    const double a=num/den;
    q[0]=x*a+2*c.d[2]*x*y+c.d[3]*(r+2*x*x);
    q[1]=y*a+c.d[2]*(r+2*y*y)+2*c.d[3]*x*y;
    if(J) {
        const double da=((c.d[0]+2*c.d[1]*r+3*c.d[4]*r4)*den-num*(c.d[5]+2*c.d[6]*r+3*c.d[7]*r4))/(den*den);
        J[0]=a+2*x*x*da+2*c.d[2]*y+6*c.d[3]*x;
        J[1]=2*x*y*da+2*c.d[2]*x+2*c.d[3]*y;
        J[2]=J[1];J[3]=a+2*y*y*da+6*c.d[2]*y+2*c.d[3]*x;
    }
    return finite(q[0])&&finite(q[1]);
}
VP_HD inline bool undistort(const Camera& c,double u,double v,double* q) {
    const double xd=(u-c.cx)/c.fx,yd=(v-c.cy)/c.fy;
    q[0]=xd;q[1]=yd;
    for(int i=0;i<20;++i) {
        double predicted[2],J[4]; if(!distort(c,q[0],q[1],predicted,J))return false;
        double e0=predicted[0]-xd,e1=predicted[1]-yd;
        if(maxv(::fabs(e0),::fabs(e1))<1e-12) return true;
        const double det=J[0]*J[3]-J[1]*J[2];if(!finite(det)||::fabs(det)<1e-15)return false;
        const double dx=(J[3]*e0-J[1]*e1)/det,dy=(-J[2]*e0+J[0]*e1)/det;
        // Backtracking is important for mildly nonmonotonic distortion fits.
        bool accepted=false; double scale=1;
        for(int trial=0;trial<8;++trial,scale*=.5) {
            double next[2];const double nx=q[0]-scale*dx,ny=q[1]-scale*dy;
            if(distort(c,nx,ny,next) && (next[0]-xd)*(next[0]-xd)+(next[1]-yd)*(next[1]-yd)<e0*e0+e1*e1) {
                q[0]=nx;q[1]=ny;accepted=true;break;
            }
        }
        if(!accepted)return false;
    }
    double p[2];return distort(c,q[0],q[1],p)&&::fabs(p[0]-xd)<1e-9&&::fabs(p[1]-yd)<1e-9;
}
VP_HD inline void object_point(int i,double size,double* p) {
    p[0]=(i==0||i==3)?-size*.5:size*.5;
    p[1]=i<2?size*.5:-size*.5;p[2]=0;
}
VP_HD inline bool project(const Camera& c,const Candidate& pose,const double* p,double* uv,double* J=nullptr) {
    double rotated[3],P[3];for(int r=0;r<3;++r){rotated[r]=dot(pose.R+r*3,p,3);P[r]=rotated[r]+pose.t[r];}
    if(!finite(P[2])||P[2]<=1e-9)return false;
    const double x=P[0]/P[2],y=P[1]/P[2],iz=1/P[2];
    double q[2],D[4];if(!distort(c,x,y,q,J?D:nullptr))return false;
    uv[0]=c.fx*q[0]+c.cx;uv[1]=c.fy*q[1]+c.cy;
    if(J) {
        const double projection[6]={c.fx*D[0]*iz,c.fx*D[1]*iz,-c.fx*(D[0]*x+D[1]*y)*iz,
                                    c.fy*D[2]*iz,c.fy*D[3]*iz,-c.fy*(D[2]*x+D[3]*y)*iz};
        // Left SO(3) increment of R; t has independent additive increments.
        const double B[9]={0,rotated[2],-rotated[1],-rotated[2],0,rotated[0],rotated[1],-rotated[0],0};
        for(int r=0;r<2;++r)for(int k=0;k<3;++k){
            J[r*6+k]=0;for(int j=0;j<3;++j)J[r*6+k]+=projection[r*3+j]*B[j*3+k];
            J[r*6+k+3]=projection[r*3+k];
        }
    }
    return finite(uv[0])&&finite(uv[1]);
}
VP_HD inline double error(const Camera& c,const Input& input,double size,const Candidate& pose) {
    double sum=0;for(int i=0;i<4;++i){double p[3],uv[2];object_point(i,size,p);if(!project(c,pose,p,uv))return HUGE_VAL;
        for(int j=0;j<2;++j){double e=uv[j]-input.uv[2*i+j];sum+=e*e;}}
    return ::sqrt(sum/4);
}
// Exact projective map from (-1,+1),(+1,+1),(+1,-1),(-1,-1).
// Solving the square's two projective coefficients directly removes the serial
// 8x8 elimination and its 64-double scratch array from every GPU initialization.
VP_HD inline bool square_homography(const double* q,double* H) {
    const double dx1=q[2]-q[4],dx2=q[6]-q[4],dx3=(q[0]-q[2])+(q[4]-q[6]);
    const double dy1=q[3]-q[5],dy2=q[7]-q[5],dy3=(q[1]-q[3])+(q[5]-q[7]);
    const double det=dx1*dy2-dx2*dy1;
    if(!finite(det)||::fabs(det)<1e-30)return false;
    const double g=(dx3*dy2-dx2*dy3)/det,h=(dx1*dy3-dx3*dy1)/det;
    const double a=q[2]-q[0]+g*q[2],b=q[6]-q[0]+h*q[6];
    const double d=q[3]-q[1]+g*q[3],e=q[7]-q[1]+h*q[7];
    const double scale=1+.5*(g+h);
    if(!finite(scale)||::fabs(scale)<1e-15)return false;
    // Convert the [0,1]^2 map to our centered square and normalize H[8].
    H[0]=.5*a/scale;H[1]=-.5*b/scale;H[2]=(.5*(a+b)+q[0])/scale;
    H[3]=.5*d/scale;H[4]=-.5*e/scale;H[5]=(.5*(d+e)+q[1])/scale;
    H[6]=.5*g/scale;H[7]=-.5*h/scale;H[8]=1;
    for(int i=0;i<8;++i)if(!finite(H[i]))return false;
    return true;
}
VP_HD inline bool initialize(const Camera& c,const Input& input,double size,Candidate* poses,const double* normalized=nullptr) {
    // Homography on unit-square points avoids poor scaling for very small tags.
    double q[8],H[9];
    if(normalized){for(int i=0;i<8;++i)q[i]=normalized[i];}
    else for(int i=0;i<4;++i)
        if(!undistort(c,input.uv[2*i],input.uv[2*i+1],q+2*i))return false;
    if(!square_homography(q,H))return false;
    const double p=H[2],qv=H[5],invh=2/size;
    const double j00=(H[0]-p*H[6])*invh,j01=(H[1]-p*H[7])*invh;
    const double j10=(H[3]-qv*H[6])*invh,j11=(H[4]-qv*H[7])*invh;
    const double norm=::sqrt(p*p+qv*qv+1),vx=p/norm,vy=qv/norm,vz=1/norm,s=1/(1+vz);
    // Rv maps +Z to the ray through the homography origin.
    const double Rv[9]={1-vx*vx*s,-vx*vy*s,vx,-vx*vy*s,1-vy*vy*s,vy,-vx,-vy,vz};
    const double b00=Rv[0]-p*Rv[6],b01=Rv[1]-p*Rv[7],b10=Rv[3]-qv*Rv[6],b11=Rv[4]-qv*Rv[7];
    const double det=b00*b11-b01*b10;if(::fabs(det)<1e-15)return false;
    const double a00=(b11*j00-b01*j10)/det,a01=(b11*j01-b01*j11)/det;
    const double a10=(-b10*j00+b00*j10)/det,a11=(-b10*j01+b00*j11)/det;
    const double aa=a00*a00+a01*a01,ab=a00*a10+a01*a11,bb=a10*a10+a11*a11;
    const double gamma=::sqrt(.5*(aa+bb+::sqrt((aa-bb)*(aa-bb)+4*ab*ab)));
    if(!finite(gamma)||gamma<1e-12)return false;
    const double a=a00/gamma,bv=a01/gamma,cc=a10/gamma,d=a11/gamma;
    const double z0=::sqrt(maxv(0,1-a*a-cc*cc));
    double z1=::sqrt(maxv(0,1-bv*bv-d*d));if(-a*bv-cc*d<0)z1=-z1;
    const double mean_u=.25*(q[0]+q[2]+q[4]+q[6]),mean_v=.25*(q[1]+q[3]+q[5]+q[7]);
    double centered[8],spread=0;
    for(int i=0;i<4;++i){centered[2*i]=q[2*i]-mean_u;centered[2*i+1]=q[2*i+1]-mean_v;
        spread+=centered[2*i]*centered[2*i]+centered[2*i+1]*centered[2*i+1];}
    if(!finite(spread)||spread<1e-30)return false;
    for(int branch=0;branch<2;++branch) {
        const double sign=branch?-1:1,col0[3]={a,cc,sign*z0},col1[3]={bv,d,sign*z1};double col2[3];cross(col0,col1,col2);
        auto& pose=poses[branch];for(int r=0;r<3;++r){pose.R[r*3]=dot(Rv+r*3,col0,3);pose.R[r*3+1]=dot(Rv+r*3,col1,3);pose.R[r*3+2]=dot(Rv+r*3,col2,3);}
        double sum_x=0,sum_y=0,numerator=0;
        for(int i=0;i<4;++i) {
            double obj[3],rotated[3];object_point(i,size,obj);for(int r=0;r<3;++r)rotated[r]=dot(pose.R+r*3,obj,3);
            const double rhs_x=q[2*i]*rotated[2]-rotated[0],rhs_y=q[2*i+1]*rotated[2]-rotated[1];
            sum_x+=rhs_x;sum_y+=rhs_y;numerator-=centered[2*i]*rhs_x+centered[2*i+1]*rhs_y;
        }
        // Schur complement of the translation least-squares equations. Computing
        // the denominator from centered rays avoids subtracting large squares.
        pose.t[2]=numerator/spread;pose.t[0]=.25*sum_x+mean_u*pose.t[2];pose.t[1]=.25*sum_y+mean_v*pose.t[2];
        pose.error=error(c,input,size,pose);
    }
    return true;
}
VP_HD inline Candidate increment(const Candidate& old,const double* step) {
    Candidate next=old;double theta2=dot(step,step,3),a,b;
    if(theta2<1e-12){a=1-theta2/6;b=.5-theta2/24;}
    else {double theta=::sqrt(theta2);a=::sin(theta)/theta;b=(1-::cos(theta))/theta2;}
    const double K[9]={0,-step[2],step[1],step[2],0,-step[0],-step[1],step[0],0};
    double E[9];for(int r=0;r<3;++r)for(int k=0;k<3;++k){
        double kk=0;for(int j=0;j<3;++j)kk+=K[r*3+j]*K[j*3+k];E[r*3+k]=(r==k?1.:0.)+a*K[r*3+k]+b*kk;}
    for(int r=0;r<3;++r){next.t[r]+=step[3+r];for(int k=0;k<3;++k){next.R[r*3+k]=0;for(int j=0;j<3;++j)next.R[r*3+k]+=E[r*3+j]*old.R[j*3+k];}}
    return next;
}
VP_HD inline int refine(const Camera& c,const Input& input,double size,Candidate& pose,int max_iterations) {
    double lambda=1e-3;int iterations=0;
    while(iterations<max_iterations) {
        ++iterations;
        double A[36]={0},g[6]={0};bool ok=true;
        for(int i=0;i<4;++i){double p[3],uv[2],J[12];object_point(i,size,p);if(!project(c,pose,p,uv,J)){ok=false;break;}
            for(int axis=0;axis<2;++axis){const double residual=input.uv[2*i+axis]-uv[axis];
                for(int r=0;r<6;++r){g[r]+=J[axis*6+r]*residual;for(int k=0;k<6;++k)A[r*6+k]+=J[axis*6+r]*J[axis*6+k];}}}
        if(!ok)break;
        for(int i=0;i<6;++i)A[i*6+i]+=lambda*maxv(A[i*6+i],1e-12);
        double step[6]={0};if(!linear<6>(A,g,step))break;
        if(dot(step,step,6)<1e-20)break;
        Candidate next=increment(pose,step);next.error=error(c,input,size,next);
        if(finite(next.error)&&next.error<pose.error){double improvement=pose.error-next.error;pose=next;lambda=maxv(1e-12,lambda*.3);if(improvement<1e-10)break;}
        else {lambda*=10;if(lambda>1e12)break;}
    }
    return iterations;
}
// normalized is an optional array of rays computed by parallel CUDA lanes. It
// is internal to the GPU solver; CPU callers invert distortion here normally.
VP_HD inline Result solve(const Camera& c,const Input& input,double size,int max_iterations=30,const double* normalized=nullptr) {
    Result result{};result.reason=1;result.ambiguity=1;result.best.error=result.alternate.error=HUGE_VAL;
    if(!finite(c.fx)||!finite(c.fy)||c.fx<=0||c.fy<=0||!finite(c.cx)||!finite(c.cy)||!finite(size)||size<=0)return result;
    for(int i=0;i<8;++i)if(!finite(input.uv[i])||!finite(c.d[i]))return result;
    // Convex, non-collapsed quadrilaterals only; no crossing corner ordering.
    double sign=0;for(int i=0;i<4;++i){int j=(i+1)%4,k=(i+2)%4;
        double cr=(input.uv[2*j]-input.uv[2*i])*(input.uv[2*k+1]-input.uv[2*j+1])-(input.uv[2*j+1]-input.uv[2*i+1])*(input.uv[2*k]-input.uv[2*j]);
        if(!finite(cr)||::fabs(cr)<1e-10||(i&&cr*sign<=0))return result;
        sign=cr;}
    Candidate seeds[2];result.reason=2;if(!initialize(c,input,size,seeds,normalized))return result;
    if(seeds[1].error<seeds[0].error){Candidate temp=seeds[0];seeds[0]=seeds[1];seeds[1]=temp;}
    if(!finite(seeds[0].error))return result;
    result.best=seeds[0];result.alternate=seeds[1];result.has_alternate=finite(seeds[1].error);
    result.initial_error=seeds[0].error;result.initial_alternate_error=seeds[1].error;
    if(result.has_alternate)result.ambiguity=seeds[1].error<1e-9?1:minv(1,seeds[0].error/seeds[1].error);
    result.iterations=refine(c,input,size,result.best,max_iterations);
    result.valid=finite(result.best.error);result.reason=result.valid?0:3;
    return result;
}
} // namespace vision_pose
#undef VP_HD
