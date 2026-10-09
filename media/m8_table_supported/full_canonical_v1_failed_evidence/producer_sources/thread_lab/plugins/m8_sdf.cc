// Geometry-only ISO 60-degree metric thread MuJoCo SDF plugin.
// Registration API follows google-deepmind/mujoco/plugin/sdf (Apache-2.0).
// No forces, joint coupling, insertion state or engagement state in this plugin.
#include <mujoco/mujoco.h>
#include <mujoco/mjplugin.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>

namespace {
constexpr int N = 8;
constexpr const char* names[N] = {"diameter", "pitch", "length", "pitch_diameter", "af", "chamfer", "female", "phase"};
constexpr mjtNum defaults[N] = {.008, .00125, .020, .007100, .013, .00065, 0, 0};
struct Params { mjtNum a[N]; };
// Point-to-profile distance in the (helical phase, cylindrical radius) plane.
// beta includes the local helical metric.  Unlike dividing an implicit radial
// field by its local gradient, closest-point distances remain valid through
// crest/root transitions and cannot report deep false contact in the bore.
mjtNum segment_distance(mjtNum u,mjtNum r,mjtNum beta,mjtNum u0,mjtNum r0,mjtNum u1,mjtNum r1) {
  const mjtNum x=(u-u0)*beta, y=r-r0;
  const mjtNum dx=(u1-u0)*beta, dy=r1-r0;
  const mjtNum t=std::clamp((x*dx+y*dy)/(dx*dx+dy*dy),mjtNum(0),mjtNum(1));
  return std::hypot(x-t*dx,y-t*dy);
}
mjtNum arc_distance(mjtNum u,mjtNum r,mjtNum beta,mjtNum uc,mjtNum rc,mjtNum R,mjtNum halfwidth) {
  const mjtNum x=u-uc,y=r-rc;
  auto at=[&](mjtNum v) {return std::hypot(beta*(v-x),-std::sqrt(R*R-v*v)-y);};
  mjtNum best=std::min(at(-halfwidth),at(halfwidth));
  // Upper queries are closest to one of the endpoints of this lower arc.
  if(y>=0) return best;
  mjtNum v=std::clamp(R*x/std::hypot(x,y),-halfwidth,halfwidth);
  // The local ellipse metric differs from a circle by <0.2% for M8.
  // Newton refinement yields the closest point on that bounded ellipse arc.
  for(int j=0;j<5;j++) {
    const mjtNum c=std::sqrt(R*R-v*v);
    const mjtNum e=-c-y;
    const mjtNum dy=v/c;
    const mjtNum f=beta*beta*(v-x)+e*dy;
    const mjtNum df=beta*beta+dy*dy+e*R*R/(c*c*c);
    if(df<=0) break;
    v=std::clamp(v-f/df,-halfwidth,halfwidth);
  }
  return std::min(best,at(v));
}
mjtNum profile_distance(mjtNum u,mjtNum r,mjtNum pitch,mjtNum r2,bool female) {
  const mjtNum H=std::sqrt(3.)*pitch*.5,major=r2+3*H/8,minor=r2-H/4;
  const mjtNum beta=1/std::sqrt(1+std::pow(pitch/(2*mjPI*std::max(r,r2*.5)),2));
  // u is wrapped to [-P/2,P/2].  Its closest periodic feature is the current
  // crest, either flank, or either neighbouring valley.  Farther copies have
  // identical radius with a greater phase separation and cannot be closest.
  mjtNum best=segment_distance(u,r,beta,-pitch/16,major,pitch/16,major);
  best=std::min(best,segment_distance(u,r,beta,pitch/16,major,3*pitch/8,minor));
  best=std::min(best,segment_distance(u,r,beta,-pitch/16,major,-3*pitch/8,minor));
  if(female) {
    best=std::min(best,segment_distance(u,r,beta,3*pitch/8,minor,5*pitch/8,minor));
    best=std::min(best,segment_distance(u,r,beta,-5*pitch/8,minor,-3*pitch/8,minor));
  } else {
    best=std::min(best,arc_distance(u,r,beta,pitch/2,r2-H/6,H/6,pitch/8));
    best=std::min(best,arc_distance(u,r,beta,-pitch/2,r2-H/6,H/6,pitch/8));
  }
  return best;
}
void read(mjtNum* a, const char* const* key, const char* const* val) {
  for (int i=0;i<N;i++) {
    a[i]=defaults[i];
    for(int j=0;j<N;j++) if(key[j] && !std::strcmp(key[j], names[i]) && val[j] && val[j][0]) a[i]=std::strtod(val[j], nullptr);
  }
}
mjtNum distance(const mjtNum p[3], const mjtNum* a) {
  const mjtNum radius = std::hypot(p[0],p[1]);
  const mjtNum pitch = a[1];
  const mjtNum H = std::sqrt(3.)*pitch*.5;
  const mjtNum r2 = a[3]*.5;
  const mjtNum major = r2+3*H/8;
  // Outside the female thread's major radius, the bore field is bounded
  // above by -(radius-major). If it and the exact chamfer field are below
  // the outer body, that body determines the result without a profile query.
  // Otherwise the original kernel below runs unchanged.
  if(radius>major && a[6] >= .5) {
    const mjtNum half=a[2]*.5;
    mjtNum hex=-mjMAXVAL;
    for(int k=0;k<6;k++) hex=std::max(hex,p[0]*std::cos(k*mjPI/3)+p[1]*std::sin(k*mjPI/3)-a[4]*.5);
    const mjtNum body=std::max(hex,std::abs(p[2])-half);
    const mjtNum bevel=major+a[5]-(half-std::abs(p[2]));
    const mjtNum hole_upper=std::max(-(radius-major),(bevel-radius)/std::sqrt(2.));
    if(hole_upper<=body) return body;
  }
  const mjtNum theta = std::atan2(p[1],p[0]);
  mjtNum phase = p[2] - pitch*theta/(2*mjPI) + a[7];
  phase -= pitch*std::floor(phase/pitch+.5);
  // ISO metric basic straight flank has an included angle of 60 degrees.
  // Pitch diameter is the dimensional fit parameter; 6g bolt / 6H nut are
  // independently specified, never inferred from thread engagement state.
  mjtNum profile = r2+std::sqrt(3.)*(pitch/4-std::abs(phase));
  mjtNum slope=std::sqrt(3.);
  if (a[6] < .5) {
    // External crest P/8 wide and tangent ISO root fillet R=H/6.
    if(std::abs(phase)<=pitch/16) {profile=major;slope=0;}
    else if(std::abs(phase)>=3*pitch/8) {
      const mjtNum R=H/6;
      const mjtNum v=pitch/2-std::abs(phase);
      const mjtNum c=std::sqrt(std::max(R*R-v*v,mjtNum(1e-30)));
      profile=r2-H/2+2*R-c;
      slope=std::abs(v/c);
    }
    // Bolt spans z=0..length, with a 45-degree thread runout at its tip.
    const mjtNum tip = major+(a[2]-a[5])-p[2];
    const mjtNum thread=std::copysign(profile_distance(phase,radius,pitch,r2,false),radius-profile);
    return std::max({thread, (radius-tip)/std::sqrt(2.), -p[2], p[2]-a[2]});
  }
  // Internal crest truncated at H/4 below pitch radius and root at 3H/8
  // above it.  Female D2 exceeds male d2 to supply explicit radial clearance.
  const mjtNum minor=r2-H/4;
  if(profile<minor || profile>major) slope=0;
  profile=std::clamp(profile,minor,major);
  const mjtNum half=a[2]*.5;
  // 45-degree entry/exit chamfer: expands cavity only near either face.
  // At a face it extends chamfer beyond the thread's major radius.
  const mjtNum bevel=major+a[5]-(half-std::abs(p[2]));
  mjtNum hole=std::copysign(profile_distance(phase,radius,pitch,r2,true),profile-radius);
  hole=std::max(hole,(bevel-radius)/std::sqrt(2.));
  mjtNum hex=-mjMAXVAL;
  for(int k=0;k<6;k++) hex=std::max(hex,p[0]*std::cos(k*mjPI/3)+p[1]*std::sin(k*mjPI/3)-a[4]*.5);
  const mjtNum body=std::max(hex,std::abs(p[2])-half);
  return std::max(body,hole);
}
void register_plugin() {
  mjpPlugin p; mjp_defaultPlugin(&p);
  p.name="astra.m8_thread"; p.capabilityflags=mjPLUGIN_SDF;
  p.nattribute=N; p.attributes=names;
  p.nstate=[](const mjModel*, int){ return 0; };
  p.init=[](const mjModel* m,mjData* d,int i){
    auto* data=new Params;
    for(int k=0;k<N;k++) {
      const char* v=mj_getPluginConfig(m,i,names[k]);
      data->a[k]=(v && v[0])?std::strtod(v,nullptr):defaults[k];
    }
    d->plugin_data[i]=reinterpret_cast<uintptr_t>(data); return 0;
  };
  p.destroy=[](mjData* d,int i){ delete reinterpret_cast<Params*>(d->plugin_data[i]); d->plugin_data[i]=0; };
  p.copy=[](mjData* dst,const mjModel*,const mjData* src,int i){ *reinterpret_cast<Params*>(dst->plugin_data[i])=*reinterpret_cast<const Params*>(src->plugin_data[i]); };
  p.reset=[](const mjModel*,mjtNum*,void*,int){};
  p.compute=[](const mjModel*,mjData*,int,int){};
  p.sdf_distance=[](const mjtNum* point,const mjData* d,int i){ return distance(point,reinterpret_cast<const Params*>(d->plugin_data[i])->a); };
  p.sdf_gradient=[](mjtNum* g,const mjtNum* point,const mjData* d,int i){
    const auto* a=reinterpret_cast<const Params*>(d->plugin_data[i])->a;
    constexpr mjtNum e=1e-8;
    for(int k=0;k<3;k++) { mjtNum lo[3]={point[0],point[1],point[2]}, hi[3]={point[0],point[1],point[2]}; lo[k]-=e;hi[k]+=e;g[k]=(distance(hi,a)-distance(lo,a))/(2*e); }
  };
  p.sdf_staticdistance=distance;
  p.sdf_attribute=[](mjtNum* a,const char** k,const char** v){read(a,k,v);};
  p.sdf_aabb=[](mjtNum* b,const mjtNum* a){
    b[0]=b[1]=0;
    if(a[6]>.5) { b[2]=0;b[3]=b[4]=a[4]/std::sqrt(3.)*1.01;b[5]=a[2]*.51; }
    else { b[2]=a[2]*.5;b[3]=b[4]=(a[3]*.5+3*std::sqrt(3.)*a[1]/16)*1.01;b[5]=a[2]*.505; }
  };
  mjp_registerPlugin(&p);
}
} // namespace
mjPLUGIN_LIB_INIT(metric_thread) { register_plugin(); }

// Diagnostic geometry sampling for tests; no dynamic force API is exposed.
extern "C" double astra_thread_distance(const double* point,const double* attrs) {
  return distance(point,attrs);
}
