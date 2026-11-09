#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
extern "C" int r1_dynamic_distance(int n,int m,const double* xs,const double* ys,const double* heights,const uint8_t* scope,const double* times,double age,const int* starts,const double* vx,const double* vy,const double* zlo,const double* zhi,const double* rate,const double* bounds,double* dist,double* nx,double* ny,double* support) {
 for(int p=0;p<n;p++){
  double best=std::numeric_limits<double>::infinity(), ox=0,oy=0,os=0,x=xs[p],y=ys[p];
  for(int i=0;i<m;i++){
   double margin=std::max(0.,rate[i])*(times[p]+age);
   if((i==0&&scope[p]==2)||(i>0&&scope[p]==1)||(!std::isnan(heights[p])&&(heights[p]<zlo[i]-margin||heights[p]>zhi[i]+margin)))continue;
   double dx=std::max({bounds[i*4]-x,x-bounds[i*4+1],0.});
   double dy=std::max({bounds[i*4+2]-y,y-bounds[i*4+3],0.});
   double box=std::max(0.,std::hypot(dx,dy)-margin);
   if((best>=0.&&box>best)||(best<0.&&box>0.)) continue;
   bool inside=true;double min_sq=std::numeric_limits<double>::infinity(), ax=0,ay=0,bx=0,by=0,ex=0,ey=0;
   for(int j=starts[i];j<starts[i+1];j++){
    int k=j+1==starts[i+1]?starts[i]:j+1;
    double a=vx[j],b=vy[j],u=vx[k]-a,v=vy[k]-b,len2=u*u+v*v;
    double invlen=1./std::sqrt(len2), onx=v*invlen,ony=-u*invlen;
    if((x-a)*onx+(y-b)*ony>1.e-10)inside=false;
    double t=std::max(0.,std::min(1.,((x-a)*u+(y-b)*v)/len2));
    double px=a+t*u,py=b+t*v, ddx=x-px,ddy=y-py,sq=ddx*ddx+ddy*ddy;
    if(sq<min_sq){min_sq=sq;ax=px;ay=py;bx=ddx;by=ddy;ex=onx;ey=ony;}
   }
   double d=std::sqrt(min_sq),sd=(inside?-d:d)-margin;
   if(sd<best){
    if(d>1.e-12){ox=bx/d;oy=by/d;if(inside){ox=-ox;oy=-oy;}}else{ox=ex;oy=ey;}
    best=sd;os=ox*ax+oy*ay+margin;
   }
  }
  dist[p]=best;nx[p]=ox;ny[p]=oy;support[p]=os;
 }
 return 0;
}
