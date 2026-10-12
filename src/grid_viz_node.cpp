/**
 * grid_viz_node.cpp  —  Cylindrical Occupancy Grid
 * =================================================
 *
 * WHAT IT DOES
 * ------------
 * 1. Auto-discovers every sensor_msgs/PointCloud2 topic whose TF chain reaches
 *    `base_link` (or whatever ~frame_id is set to).
 * 2. Subscribes to all of them and bins each incoming point into a cylindrical
 *    (r, θ, z) voxel grid stored as a flat array of atomic uint32 counters.
 * 3. A background decay thread continuously multiplies all counters by a
 *    configurable decay factor so stale readings fade out.
 * 4. Publishes the static wireframe MarkerArray for structural visualisation.
 * 5. When ~debug:=true, additionally publishes:
 *      • sensor_msgs/PointCloud2  on  ~/cylindrical_grid/density_cloud
 *      • visualization_msgs/MarkerArray  on  ~/cylindrical_grid/density_markers
 *        (now using curved sector geometry instead of cubes)
 *
 * PARAMETERS (all have defaults)
 * ----------------------------------------------------------
 *  ~frame_id           string   "base_link"
 *  ~max_radius         double   8.0   m
 *  ~min_radius         double   0.15  m
 *  ~height             double   3.0   m
 *  ~z_offset           double  -0.5   m
 *  ~n_radial           int      8
 *  ~n_angular          int      36
 *  ~n_vertical         int      6
 *  ~log_radial         bool     true
 *  ~decay_rate         double   0.95
 *  ~decay_period_s     double   0.5   s
 *  ~discovery_rate     double   2.0   Hz
 *  ~publish_rate       double   10.0  Hz
 *  ~line_width         double   0.02  m
 *  ~alpha              double   0.6
 *  ~debug              bool     false
 *  ~debug_min_density  double   0.01
 *  ~density_arc_segs   int      8     (segments per arc for density sectors)
 */

#include <ros/ros.h>
#include <ros/master.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <atomic>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_set>
#include <thread>
#include <memory>
#include <algorithm>
#include <functional>

// ============================================================================
//  Constants
// ============================================================================

static constexpr int ATAN2_LUT_SIZE = 1024;
static constexpr int ARC_SEGS       = 12;

// ============================================================================
//  Utility helpers
// ============================================================================

static geometry_msgs::Point makePoint(double x, double y, double z)
{ geometry_msgs::Point p; p.x=x; p.y=y; p.z=z; return p; }

static std_msgs::ColorRGBA makeColor(float r, float g, float b, float a)
{ std_msgs::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=a; return c; }

static std_msgs::ColorRGBA hsvToRgba(float h, float s, float v, float a)
{
    float r=0,g=0,b=0;
    int i=static_cast<int>(h*6.f);
    float f=h*6.f-i, p=v*(1-s), q=v*(1-f*s), t=v*(1-(1-f)*s);
    switch(i%6){
      case 0:r=v;g=t;b=p;break; case 1:r=q;g=v;b=p;break;
      case 2:r=p;g=v;b=t;break; case 3:r=p;g=q;b=v;break;
      case 4:r=t;g=p;b=v;break; case 5:r=v;g=p;b=q;break;
    }
    return makeColor(r,g,b,a);
}

static std_msgs::ColorRGBA densityColor(float t, float alpha)
{
    static const float RR[]={0.f,0.f,0.f,1.f,1.f};
    static const float GG[]={0.f,1.f,1.f,1.f,0.f};
    static const float BB[]={1.f,1.f,0.f,0.f,0.f};
    t = std::max(0.f, std::min(1.f, t));
    float s = t*4.f;
    int i = std::min(3, static_cast<int>(s));
    float f = s-i;
    return makeColor(RR[i]+(RR[i+1]-RR[i])*f,
                     GG[i]+(GG[i+1]-GG[i])*f,
                     BB[i]+(BB[i+1]-BB[i])*f, alpha);
}

// ============================================================================
//  GridGeometry
// ============================================================================



struct GridGeometry
{
    std::vector<double>  r_bounds;
    std::vector<double>  r2_bounds;
    std::vector<double>  th_bounds;
    std::vector<double>  z_bounds;
    std::vector<double>  r_centres;
    std::vector<double>  th_centres;
    std::vector<double>  z_centres;

    int n_radial, n_angular, n_vertical, total_bins;

    std::vector<int16_t> atan2_lut;

    void build(double r_min, double r_max,
               double z_bot, double z_top,
               int nr, int nth, int nz, bool log_r)
    {
        n_radial=nr; n_angular=nth; n_vertical=nz;
        total_bins=nr*nth*nz;

        r_bounds.resize(nr+1);
        if(log_r){
            double lmin=std::log(r_min), lmax=std::log(r_max);
            for(int i=0;i<=nr;++i)
                r_bounds[i]=std::exp(lmin+i*(lmax-lmin)/nr);
        } else {
            for(int i=0;i<=nr;++i)
                r_bounds[i]=r_min+i*(r_max-r_min)/nr;
        }
        r2_bounds.resize(nr+1);
        for(int i=0;i<=nr;++i) r2_bounds[i]=r_bounds[i]*r_bounds[i];
        r_centres.resize(nr);
        for(int i=0;i<nr;++i) r_centres[i]=0.5*(r_bounds[i]+r_bounds[i+1]);

        th_bounds.resize(nth+1);
        for(int i=0;i<=nth;++i) th_bounds[i]=i*2.0*M_PI/nth;
        th_centres.resize(nth);
        for(int i=0;i<nth;++i) th_centres[i]=0.5*(th_bounds[i]+th_bounds[i+1]);

        z_bounds.resize(nz+1);
        for(int i=0;i<=nz;++i) z_bounds[i]=z_bot+i*(z_top-z_bot)/nz;
        z_centres.resize(nz);
        for(int i=0;i<nz;++i) z_centres[i]=0.5*(z_bounds[i]+z_bounds[i+1]);

        const int N=ATAN2_LUT_SIZE;
        atan2_lut.resize(N*N);
        double two_pi_inv=nth/(2.0*M_PI);
        for(int iy=0;iy<N;++iy){
            double y=(iy-N/2+0.5)/(N/2);
            for(int ix=0;ix<N;++ix){
                double x=(ix-N/2+0.5)/(N/2);
                double angle=std::atan2(y,x);
                if(angle<0) angle+=2.0*M_PI;
                int bin=static_cast<int>(angle*two_pi_inv);
                if(bin>=nth) bin=nth-1;
                atan2_lut[iy*N+ix]=static_cast<int16_t>(bin);
            }
        }
    }

    inline int binIndex(int ir, int ith, int iz) const
    { return ir*(n_angular*n_vertical)+ith*n_vertical+iz; }

    inline bool lookupBin(float x, float y, float z_val,
                          int& out_idx) const
    {
        double zd=static_cast<double>(z_val);
        if(zd < z_bounds.front() || zd >= z_bounds.back()) return false;

        double r2=static_cast<double>(x)*x+static_cast<double>(y)*y;
        if(r2 < r2_bounds.front() || r2 >= r2_bounds.back()) return false;

        int ir=static_cast<int>(
            std::upper_bound(r2_bounds.begin(),r2_bounds.end(),r2)
            -r2_bounds.begin())-1;
        if(ir<0||ir>=n_radial) return false;

        double r=std::sqrt(r2);
        double inv_r=1.0/r;
        const int N=ATAN2_LUT_SIZE;
        int ix=static_cast<int>((x*inv_r*0.9999+1.0)*(N/2));
        int iy=static_cast<int>((y*inv_r*0.9999+1.0)*(N/2));
        if(ix<0)ix=0; else if(ix>=N)ix=N-1;
        if(iy<0)iy=0; else if(iy>=N)iy=N-1;
        int ith=atan2_lut[iy*N+ix];

        int iz=static_cast<int>(
            std::upper_bound(z_bounds.begin(),z_bounds.end(),zd)
            -z_bounds.begin())-1;
        if(iz<0||iz>=n_vertical) return false;

        out_idx=binIndex(ir,ith,iz);
        return true;
    }
};

// ============================================================================
//  DensityGrid  —  raw unique_ptr array of atomics (vector<atomic> won't
//                  compile on GCC 9 because atomic is non-movable and the
//                  vector internals instantiate move paths at compile time)
// ============================================================================

struct DensityGrid
{
    // Use a raw heap array via unique_ptr — avoids ALL vector reallocation
    // machinery that chokes on non-movable types in GCC 9.
    std::unique_ptr<std::atomic<uint32_t>[]> counts;
    int total_bins{0};

    void init(int n)
    {
        total_bins = n;
        // Allocate fresh array; new[] value-initialises each atomic to 0.
        counts.reset(new std::atomic<uint32_t>[n]);
        for(int i = 0; i < n; ++i)
            counts[i].store(0, std::memory_order_relaxed);
    }

    inline void increment(int idx)
    {
        if(counts[idx].load(std::memory_order_relaxed) < 65535u)
            counts[idx].fetch_add(1, std::memory_order_relaxed);
    }

    void decay(float factor)
    {
        for(int i = 0; i < total_bins; ++i){
            uint32_t v = counts[i].load(std::memory_order_relaxed);
            if(v > 0)
                counts[i].store(
                    static_cast<uint32_t>(v * factor),
                    std::memory_order_relaxed);
        }
    }

    uint32_t maxCount() const
    {
        uint32_t m = 0;
        for(int i = 0; i < total_bins; ++i)
            m = std::max(m, counts[i].load(std::memory_order_relaxed));
        return m;
    }
};

// ============================================================================
//  Wireframe builder
// ============================================================================

static void addArc(visualization_msgs::Marker& mk,
                   double r, double z, double th0, double th1, int segs)
{
    for(int s=0;s<segs;++s){
        double a0=th0+s*(th1-th0)/segs, a1=th0+(s+1)*(th1-th0)/segs;
        mk.points.push_back(makePoint(r*std::cos(a0),r*std::sin(a0),z));
        mk.points.push_back(makePoint(r*std::cos(a1),r*std::sin(a1),z));
    }
}
static void addVLine(visualization_msgs::Marker& mk,
                     double r, double th, double z0, double z1)
{
    mk.points.push_back(makePoint(r*std::cos(th),r*std::sin(th),z0));
    mk.points.push_back(makePoint(r*std::cos(th),r*std::sin(th),z1));
}
static void addRLine(visualization_msgs::Marker& mk,
                     double r0, double r1, double th, double z)
{
    mk.points.push_back(makePoint(r0*std::cos(th),r0*std::sin(th),z));
    mk.points.push_back(makePoint(r1*std::cos(th),r1*std::sin(th),z));
}
static visualization_msgs::Marker makeLineMarker(
    const std::string& frame_id, const ros::Time& t,
    int id, const std::string& ns, double lw,
    const std_msgs::ColorRGBA& col)
{
    visualization_msgs::Marker mk;
    mk.header.frame_id=frame_id; mk.header.stamp=t;
    mk.ns=ns; mk.id=id;
    mk.type=visualization_msgs::Marker::LINE_LIST;
    mk.action=visualization_msgs::Marker::ADD;
    mk.scale.x=lw; mk.color=col;
    mk.pose.orientation.w=1.0;
    mk.lifetime=ros::Duration(0);
    return mk;
}

static visualization_msgs::MarkerArray buildWireframe(
    const GridGeometry& g, const std::string& frame_id,
    double lw, float alpha)
{
    visualization_msgs::MarkerArray ma;
    ros::Time now=ros::Time::now();
    for(int ir=0;ir<g.n_radial;++ir){
        float hue=static_cast<float>(ir)/g.n_radial;
        auto mk=makeLineMarker(frame_id,now,ir,"cylindrical_bins",lw,
                               hsvToRgba(hue,0.85f,0.95f,alpha));
        double ri=g.r_bounds[ir], ro=g.r_bounds[ir+1];
        for(int iz=0;iz<g.n_vertical;++iz){
            double zb=g.z_bounds[iz],zt=g.z_bounds[iz+1];
            for(int ith=0;ith<g.n_angular;++ith){
                double th0=g.th_bounds[ith],th1=g.th_bounds[ith+1];
                addArc(mk,ri,zb,th0,th1,ARC_SEGS); addArc(mk,ro,zb,th0,th1,ARC_SEGS);
                if(iz==0){addRLine(mk,ri,ro,th0,zb); addRLine(mk,ri,ro,th1,zb);}
                addArc(mk,ri,zt,th0,th1,ARC_SEGS); addArc(mk,ro,zt,th0,th1,ARC_SEGS);
                addRLine(mk,ri,ro,th0,zt); addRLine(mk,ri,ro,th1,zt);
                addVLine(mk,ri,th0,zb,zt); addVLine(mk,ro,th0,zb,zt);
                if(ith==g.n_angular-1){
                    addVLine(mk,ri,th1,zb,zt); addVLine(mk,ro,th1,zb,zt);
                }
            }
        }
        ma.markers.push_back(mk);
    }
    {
        auto excl=makeLineMarker(frame_id,now,g.n_radial,"exclusion_zone",
                                  lw*1.5,makeColor(1,.3f,.3f,alpha*.8f));
        double r=g.r_bounds[0];
        for(int iz=0;iz<=g.n_vertical;++iz)
            addArc(excl,r,g.z_bounds[iz],0,2*M_PI,g.n_angular*ARC_SEGS);
        ma.markers.push_back(excl);
    }
    return ma;
}

// ============================================================================
//  Debug output builders
// ============================================================================

static sensor_msgs::PointCloud2 buildDensityCloud(
    const GridGeometry& g, const DensityGrid& grid,
    const std::string& frame_id, float min_frac)
{
    uint32_t max_cnt=grid.maxCount();
    if(max_cnt==0) max_cnt=1;

    std::vector<float> xs,ys,zs,ins;
    xs.reserve(g.total_bins);

    for(int ir=0;ir<g.n_radial;++ir){
        double r=g.r_centres[ir];
        for(int ith=0;ith<g.n_angular;++ith){
            double th=g.th_centres[ith];
            float cx=static_cast<float>(r*std::cos(th));
            float cy=static_cast<float>(r*std::sin(th));
            for(int iz=0;iz<g.n_vertical;++iz){
                float norm=static_cast<float>(
                    grid.counts[g.binIndex(ir,ith,iz)]
                    .load(std::memory_order_relaxed))
                    /max_cnt;
                if(norm<min_frac) continue;
                xs.push_back(cx); ys.push_back(cy);
                zs.push_back(static_cast<float>(g.z_centres[iz]));
                ins.push_back(norm);
            }
        }
    }

    int N=static_cast<int>(xs.size());
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id=frame_id;
    cloud.header.stamp=ros::Time::now();
    cloud.height=1; cloud.width=N;
    cloud.is_dense=true; cloud.is_bigendian=false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(4,
        "x",1,sensor_msgs::PointField::FLOAT32,
        "y",1,sensor_msgs::PointField::FLOAT32,
        "z",1,sensor_msgs::PointField::FLOAT32,
        "intensity",1,sensor_msgs::PointField::FLOAT32);
    mod.resize(N);

    sensor_msgs::PointCloud2Iterator<float> ix(cloud,"x"),iy(cloud,"y"),
                                             iz(cloud,"z"),ii(cloud,"intensity");
    for(int i=0;i<N;++i,++ix,++iy,++iz,++ii){
        *ix=xs[i]; *iy=ys[i]; *iz=zs[i]; *ii=ins[i];
    }
    return cloud;
}

// New function: Build curved sector mesh for a single bin
static void addSectorMesh(visualization_msgs::Marker& mk,
                          double r_inner, double r_outer,
                          double th0, double th1,
                          double z_bottom, double z_top,
                          int arc_segs,
                          const std_msgs::ColorRGBA& color)
{
    // Add triangles for the sector (like a curved wedge)
    // We'll build it as a solid using TRIANGLE_LIST
    
    // Bottom face
    for(int s=0; s<arc_segs; ++s){
        double a0 = th0 + s*(th1-th0)/arc_segs;
        double a1 = th0 + (s+1)*(th1-th0)/arc_segs;
        
        // Triangle 1
        mk.points.push_back(makePoint(r_inner*std::cos(a0), r_inner*std::sin(a0), z_bottom));
        mk.points.push_back(makePoint(r_outer*std::cos(a0), r_outer*std::sin(a0), z_bottom));
        mk.points.push_back(makePoint(r_outer*std::cos(a1), r_outer*std::sin(a1), z_bottom));
        
        // Triangle 2
        mk.points.push_back(makePoint(r_inner*std::cos(a0), r_inner*std::sin(a0), z_bottom));
        mk.points.push_back(makePoint(r_outer*std::cos(a1), r_outer*std::sin(a1), z_bottom));
        mk.points.push_back(makePoint(r_inner*std::cos(a1), r_inner*std::sin(a1), z_bottom));
        
        // Add colors for each vertex
        for(int i=0; i<6; ++i) mk.colors.push_back(color);
    }
    
    // Top face
    for(int s=0; s<arc_segs; ++s){
        double a0 = th0 + s*(th1-th0)/arc_segs;
        double a1 = th0 + (s+1)*(th1-th0)/arc_segs;
        
        // Triangle 1 (reverse order for proper normal)
        mk.points.push_back(makePoint(r_inner*std::cos(a0), r_inner*std::sin(a0), z_top));
        mk.points.push_back(makePoint(r_outer*std::cos(a1), r_outer*std::sin(a1), z_top));
        mk.points.push_back(makePoint(r_outer*std::cos(a0), r_outer*std::sin(a0), z_top));
        
        // Triangle 2
        mk.points.push_back(makePoint(r_inner*std::cos(a0), r_inner*std::sin(a0), z_top));
        mk.points.push_back(makePoint(r_inner*std::cos(a1), r_inner*std::sin(a1), z_top));
        mk.points.push_back(makePoint(r_outer*std::cos(a1), r_outer*std::sin(a1), z_top));
        
        for(int i=0; i<6; ++i) mk.colors.push_back(color);
    }
    
    // Outer curved wall
    for(int s=0; s<arc_segs; ++s){
        double a0 = th0 + s*(th1-th0)/arc_segs;
        double a1 = th0 + (s+1)*(th1-th0)/arc_segs;
        
        // Triangle 1
        mk.points.push_back(makePoint(r_outer*std::cos(a0), r_outer*std::sin(a0), z_bottom));
        mk.points.push_back(makePoint(r_outer*std::cos(a0), r_outer*std::sin(a0), z_top));
        mk.points.push_back(makePoint(r_outer*std::cos(a1), r_outer*std::sin(a1), z_top));
        
        // Triangle 2
        mk.points.push_back(makePoint(r_outer*std::cos(a0), r_outer*std::sin(a0), z_bottom));
        mk.points.push_back(makePoint(r_outer*std::cos(a1), r_outer*std::sin(a1), z_top));
        mk.points.push_back(makePoint(r_outer*std::cos(a1), r_outer*std::sin(a1), z_bottom));
        
        for(int i=0; i<6; ++i) mk.colors.push_back(color);
    }
    
    // Inner curved wall
    for(int s=0; s<arc_segs; ++s){
        double a0 = th0 + s*(th1-th0)/arc_segs;
        double a1 = th0 + (s+1)*(th1-th0)/arc_segs;
        
        // Triangle 1 (reverse order for proper normal)
        mk.points.push_back(makePoint(r_inner*std::cos(a0), r_inner*std::sin(a0), z_bottom));
        mk.points.push_back(makePoint(r_inner*std::cos(a1), r_inner*std::sin(a1), z_top));
        mk.points.push_back(makePoint(r_inner*std::cos(a0), r_inner*std::sin(a0), z_top));
        
        // Triangle 2
        mk.points.push_back(makePoint(r_inner*std::cos(a0), r_inner*std::sin(a0), z_bottom));
        mk.points.push_back(makePoint(r_inner*std::cos(a1), r_inner*std::sin(a1), z_bottom));
        mk.points.push_back(makePoint(r_inner*std::cos(a1), r_inner*std::sin(a1), z_top));
        
        for(int i=0; i<6; ++i) mk.colors.push_back(color);
    }
    
    // Side walls at th0 and th1
    // Wall at th0
    mk.points.push_back(makePoint(r_inner*std::cos(th0), r_inner*std::sin(th0), z_bottom));
    mk.points.push_back(makePoint(r_outer*std::cos(th0), r_outer*std::sin(th0), z_bottom));
    mk.points.push_back(makePoint(r_outer*std::cos(th0), r_outer*std::sin(th0), z_top));
    
    mk.points.push_back(makePoint(r_inner*std::cos(th0), r_inner*std::sin(th0), z_bottom));
    mk.points.push_back(makePoint(r_outer*std::cos(th0), r_outer*std::sin(th0), z_top));
    mk.points.push_back(makePoint(r_inner*std::cos(th0), r_inner*std::sin(th0), z_top));
    
    for(int i=0; i<6; ++i) mk.colors.push_back(color);
    
    // Wall at th1
    mk.points.push_back(makePoint(r_inner*std::cos(th1), r_inner*std::sin(th1), z_bottom));
    mk.points.push_back(makePoint(r_outer*std::cos(th1), r_outer*std::sin(th1), z_top));
    mk.points.push_back(makePoint(r_outer*std::cos(th1), r_outer*std::sin(th1), z_bottom));
    
    mk.points.push_back(makePoint(r_inner*std::cos(th1), r_inner*std::sin(th1), z_bottom));
    mk.points.push_back(makePoint(r_inner*std::cos(th1), r_inner*std::sin(th1), z_top));
    mk.points.push_back(makePoint(r_outer*std::cos(th1), r_outer*std::sin(th1), z_top));
    
    for(int i=0; i<6; ++i) mk.colors.push_back(color);
}

static visualization_msgs::MarkerArray buildDensityMarkers(
    const GridGeometry& g, const DensityGrid& grid,
    const std::string& frame_id, float min_frac, float alpha, int arc_segs)
{
    visualization_msgs::MarkerArray ma;
    ros::Time now=ros::Time::now();
    uint32_t max_cnt=grid.maxCount();
    if(max_cnt==0) max_cnt=1;

    // Create one marker for all density sectors
    visualization_msgs::Marker mk;
    mk.header.frame_id = frame_id;
    mk.header.stamp = now;
    mk.ns = "density_sectors";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::TRIANGLE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.lifetime = ros::Duration(0);
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 1.0;  // Not used for TRIANGLE_LIST
    mk.scale.y = 1.0;
    mk.scale.z = 1.0;

    // Pre-allocate approximate space
    int max_triangles = g.total_bins * (arc_segs * 8 + 12);  // Rough estimate
    mk.points.reserve(max_triangles * 3);
    mk.colors.reserve(max_triangles * 3);

    // Add sectors for bins with density > min_frac
    for(int ir=0; ir<g.n_radial; ++ir){
        double r_inner = g.r_bounds[ir];
        double r_outer = g.r_bounds[ir+1];
        
        for(int ith=0; ith<g.n_angular; ++ith){
            double th0 = g.th_bounds[ith];
            double th1 = g.th_bounds[ith+1];
            
            for(int iz=0; iz<g.n_vertical; ++iz){
                float norm = static_cast<float>(
                    grid.counts[g.binIndex(ir,ith,iz)]
                    .load(std::memory_order_relaxed))
                    / max_cnt;
                
                if(norm < min_frac) continue;
                
                double z_bottom = g.z_bounds[iz];
                double z_top = g.z_bounds[iz+1];
                
                auto color = densityColor(norm, alpha);
                addSectorMesh(mk, r_inner, r_outer, th0, th1, 
                             z_bottom, z_top, arc_segs, color);
            }
        }
    }

    if(!mk.points.empty()){
        ma.markers.push_back(mk);
    } else {
        // Delete marker if no sectors to show
        visualization_msgs::Marker del;
        del.header.frame_id = frame_id;
        del.header.stamp = now;
        del.ns = "density_sectors";
        del.id = 0;
        del.action = visualization_msgs::Marker::DELETE;
        ma.markers.push_back(del);
    }
    
    return ma;
}

// ============================================================================
//  Main node class
// ============================================================================

class CylindricalGridNode
{
public:
    CylindricalGridNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_)
    {
        loadParams();

        geom_.build(min_radius_, max_radius_,
                    z_offset_, z_offset_+height_,
                    n_radial_, n_angular_, n_vertical_, log_radial_);
        density_.init(geom_.total_bins);

        ROS_INFO("[CylGrid] Total bins: %d  (r=%d × θ=%d × z=%d)  frame: %s",
                 geom_.total_bins, n_radial_, n_angular_, n_vertical_,
                 frame_id_.c_str());

        wireframe_ = buildWireframe(geom_, frame_id_, line_width_,
                                    static_cast<float>(alpha_));

        wire_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "cylindrical_grid/markers", 1, /*latch=*/true);

        if(debug_){
            cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
                "cylindrical_grid/density_cloud", 5);
            mkarray_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
                "cylindrical_grid/density_markers", 5);
            ROS_INFO("[CylGrid] Debug ON — "
                     "density_cloud (Foxglove) + density_markers (curved sectors)");
            ROS_INFO("[CylGrid] Foxglove: PointCloud → topic above → "
                     "Color by: intensity → Turbo colormap → point_size ~0.4m");
            ROS_INFO("[CylGrid] Density sectors arc segments: %d", density_arc_segs_);
        }

        if (publish_topdown_image_) {
    image_pub_ = nh_.advertise<sensor_msgs::Image>(
        "cylindrical_grid/topdown_image", 5);

    ROS_INFO("[CylGrid] Top-down image enabled: cylindrical_grid/topdown_image");
}

        decay_thread_ = std::thread([this]{ decayLoop(); });

        double disc_rate, pub_rate;
        pnh_.param("discovery_rate", disc_rate, 2.0);
        pnh_.param("publish_rate",   pub_rate, 10.0);

        discovery_timer_ = nh_.createTimer(
            ros::Duration(1.0/disc_rate),
            &CylindricalGridNode::onDiscovery, this);

        publish_timer_ = nh_.createTimer(
            ros::Duration(1.0/pub_rate),
            &CylindricalGridNode::onPublish, this);

        
    }

   static sensor_msgs::Image buildTopDownImage(
    const GridGeometry& g,
    const DensityGrid& grid,
    const std::string& frame_id,
    int res)
{
    uint32_t max_cnt = grid.maxCount();
    if (max_cnt == 0) max_cnt = 1;

    const int W = res;
    const int H = res;

    std::vector<float> rbuf(W * H, 0.0f);
    std::vector<float> gbuf(W * H, 0.0f);
    std::vector<float> bbuf(W * H, 0.0f);

    double cx = W * 0.5;
    double cy = H * 0.5;

    double r_max = g.r_bounds.back();
    double scale = (W * 0.5) / (r_max * 1.05);

    auto toImg = [&](double x, double y) -> std::pair<int,int> {
        int ix = static_cast<int>(cx + x * scale);
        int iy = static_cast<int>(cy - y * scale);
        return {ix, iy};
    };

    auto densityColor = [&](float t, float &r, float &g, float &b)
    {
        // same palette style as your markers
        t = std::max(0.0f, std::min(t, 1.0f));

        r = std::min(1.0f, 2.0f * t);
        g = std::min(1.0f, 2.0f * (1.0f - std::fabs(t - 0.5f) * 2.0f));
        b = std::max(0.0f, 1.0f - 2.0f * t);
    };

    auto fillQuad = [&](std::array<std::pair<int,int>,4> pts,
                        float rr, float gg, float bb)
    {
        // very simple scan conversion (good enough + fast enough for bins)

        int minx = W, maxx = 0, miny = H, maxy = 0;

        for (auto &p : pts) {
            minx = std::min(minx, p.first);
            maxx = std::max(maxx, p.first);
            miny = std::min(miny, p.second);
            maxy = std::max(maxy, p.second);
        }

        minx = std::max(0, std::min(minx, W-1));
        maxx = std::max(0, std::min(maxx, W - 1));
miny = std::max(0, std::min(miny, H - 1));
maxy = std::max(0, std::min(maxy, H - 1));
        // barycentric-ish fill (cheap & stable)
        auto sign = [](std::pair<int,int> a, std::pair<int,int> b, std::pair<int,int> c){
            return (a.first - c.first) * (b.second - c.second) -
                   (b.first - c.first) * (a.second - c.second);
        };

        for (int y = miny; y <= maxy; ++y) {
            for (int x = minx; x <= maxx; ++x) {

                std::pair<int,int> p{x,y};

                bool b1 = sign(p, pts[0], pts[1]) < 0;
                bool b2 = sign(p, pts[1], pts[2]) < 0;
                bool b3 = sign(p, pts[2], pts[3]) < 0;
                bool b4 = sign(p, pts[3], pts[0]) < 0;

                if ((b1 == b2) && (b2 == b3) && (b3 == b4)) {
                    int idx = y * W + x;
                    rbuf[idx] = std::max(rbuf[idx], rr);
                    gbuf[idx] = std::max(gbuf[idx], gg);
                    bbuf[idx] = std::max(bbuf[idx], bb);
                }
            }
        }
    };

    // =========================
    // MAIN RENDER LOOP
    // =========================
    for (int ir = 0; ir < g.n_radial; ++ir) {

        double r0 = g.r_bounds[ir];
        double r1 = g.r_bounds[ir + 1];

        for (int ith = 0; ith < g.n_angular; ++ith) {

            double th0 = g.th_bounds[ith];
            double th1 = g.th_bounds[ith + 1];

            float best = 0.0f;

            for (int iz = 0; iz < g.n_vertical; ++iz) {
                float norm = static_cast<float>(
                    grid.counts[g.binIndex(ir, ith, iz)]
                        .load(std::memory_order_relaxed)
                ) / max_cnt;

                best = std::max(best, norm);
            }

            if (best <= 0.0f) continue;

            float rr, gg, bb;
            densityColor(best, rr, gg, bb);

            // 4 corners of sector
            std::array<std::pair<int,int>,4> pts;

            pts[0] = toImg(r0 * std::cos(th0), r0 * std::sin(th0));
            pts[1] = toImg(r1 * std::cos(th0), r1 * std::sin(th0));
            pts[2] = toImg(r1 * std::cos(th1), r1 * std::sin(th1));
            pts[3] = toImg(r0 * std::cos(th1), r0 * std::sin(th1));

            fillQuad(pts, rr, gg, bb);
        }
    }

    // =========================
    // PACK INTO ROS IMAGE
    // =========================
    sensor_msgs::Image img;
    img.header.frame_id = frame_id;
    img.header.stamp = ros::Time::now();

    img.height = H;
    img.width = W;
    img.encoding = "rgb8";
    img.step = W * 3;

    img.data.resize(W * H * 3);

    for (int i = 0; i < W * H; ++i) {
        img.data[i*3 + 0] = static_cast<uint8_t>(std::max(0.0f, std::min(rbuf[i], 1.0f)) * 255);
img.data[i*3 + 1] = static_cast<uint8_t>(std::max(0.0f, std::min(gbuf[i], 1.0f)) * 255);
img.data[i*3 + 2] = static_cast<uint8_t>(std::max(0.0f, std::min(bbuf[i], 1.0f)) * 255);
    }

    return img;
}

    ~CylindricalGridNode(){
        shutdown_=true;
        if(decay_thread_.joinable()) decay_thread_.join();
    }

private:
    void loadParams()
    {
        pnh_.param<std::string>("frame_id", frame_id_, "base_link");
        pnh_.param("max_radius",     max_radius_,     8.0);
        pnh_.param("min_radius",     min_radius_,     0.15);
        pnh_.param("height",         height_,         3.0);
        pnh_.param("z_offset",       z_offset_,      -0.5);
        pnh_.param("n_radial",       n_radial_,       8);
        pnh_.param("n_angular",      n_angular_,      36);
        pnh_.param("n_vertical",     n_vertical_,     6);
        pnh_.param("log_radial",     log_radial_,     true);
        pnh_.param("decay_rate",     decay_rate_,     0.95);
        pnh_.param("decay_period_s", decay_period_s_, 0.5);
        pnh_.param("line_width",     line_width_,     0.02);
        pnh_.param("alpha",          alpha_,          0.6);
        pnh_.param("debug",          debug_,          false);
        pnh_.param("debug_min_density", debug_min_density_, 0.01);
        pnh_.param("density_arc_segs", density_arc_segs_, 8);
        pnh_.param("publish_topdown_image", publish_topdown_image_, true);
        pnh_.param("topdown_res", topdown_res_, 512);
pnh_.param("topdown_padding", topdown_padding_, 1.0);

        if(min_radius_<=0){ ROS_WARN("min_radius<=0, clamping to 0.05"); min_radius_=0.05; }
        if(max_radius_<=min_radius_){ ROS_FATAL("max_radius must be > min_radius"); ros::shutdown(); }
    }

    void onCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
    {
        sensor_msgs::PointCloud2 transformed;
        try {
            geometry_msgs::TransformStamped tf =
                tf_buffer_.lookupTransform(
                    frame_id_, msg->header.frame_id,
                    msg->header.stamp, ros::Duration(0.05));
            tf2::doTransform(*msg, transformed, tf);
        } catch(const tf2::TransformException&) {
            return;
        }

        sensor_msgs::PointCloud2ConstIterator<float>
            it_x(transformed,"x"), it_y(transformed,"y"), it_z(transformed,"z");

        for(; it_x!=it_x.end(); ++it_x,++it_y,++it_z){
            int idx;
            if(geom_.lookupBin(*it_x,*it_y,*it_z,idx))
                density_.increment(idx);
        }
    }

    void onDiscovery(const ros::TimerEvent&)
    {
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);

        for(auto& t : topics){
            if(t.datatype!="sensor_msgs/PointCloud2") continue;
            if(subscribed_.count(t.name)) continue;

            ROS_INFO("[CylGrid] Found new PointCloud2 topic: %s", t.name.c_str());

            ros::Subscriber sub = nh_.subscribe<sensor_msgs::PointCloud2>(
                t.name, 5,
                boost::bind(&CylindricalGridNode::onCloud, this, _1));
            subs_.push_back(sub);
            subscribed_.insert(t.name);
        }
    }

    void onPublish(const ros::TimerEvent&)
    {
        ros::Time now=ros::Time::now();
        for(auto& mk : wireframe_.markers) mk.header.stamp=now;
        wire_pub_.publish(wireframe_);

        if (publish_topdown_image_) {
    image_pub_.publish(
    buildTopDownImage(geom_, density_, frame_id_, topdown_res_));
}

        if(debug_){
            cloud_pub_.publish(
                buildDensityCloud(geom_,density_,frame_id_,
                                  static_cast<float>(debug_min_density_)));
            mkarray_pub_.publish(
                buildDensityMarkers(geom_,density_,frame_id_,
                                    static_cast<float>(debug_min_density_),
                                    static_cast<float>(alpha_),
                                    density_arc_segs_));
        }
    }

    void decayLoop()
    {
        ros::Rate rate(1.0/decay_period_s_);
        while(!shutdown_ && ros::ok()){
            density_.decay(static_cast<float>(decay_rate_));
            rate.sleep();
        }
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;

    tf2_ros::Buffer            tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    GridGeometry geom_;
    DensityGrid  density_;

    std::string frame_id_;
    double max_radius_, min_radius_, height_, z_offset_;
    int    n_radial_, n_angular_, n_vertical_;
    bool   log_radial_;
    double decay_rate_, decay_period_s_;
    double line_width_, alpha_;
    bool   debug_;
    double debug_min_density_;
    int    density_arc_segs_;

    ros::Publisher wire_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher mkarray_pub_;

    ros::Publisher image_pub_;
bool publish_topdown_image_{true};
int topdown_res_{512};
double topdown_padding_{1.0};

    ros::Timer discovery_timer_;
    ros::Timer publish_timer_;

    std::vector<ros::Subscriber>    subs_;
    std::unordered_set<std::string> subscribed_;

    std::thread       decay_thread_;
    std::atomic<bool> shutdown_{false};

    visualization_msgs::MarkerArray wireframe_;
};

// ============================================================================
//  main
// ============================================================================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cylindrical_grid_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    int threads=std::max(2,static_cast<int>(std::thread::hardware_concurrency()));
    ros::AsyncSpinner spinner(threads);
    spinner.start();

    CylindricalGridNode node(nh, pnh);
    ros::waitForShutdown();
    return 0;
}