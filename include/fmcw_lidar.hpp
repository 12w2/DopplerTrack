#ifndef __FMCW_POINT_DEFINE_H__
#define __FMCW_POINT_DEFINE_H__

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <unordered_set>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <math.h>

// Custom point type definition
struct _FMCWLidar_9D_PointT
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;

    union
    {
        struct
        {
            float    velocity0;
            uint32_t ring;
            float    time;
            float    intensity;
        };
        uint32_t data_c[4];
    };

    union
    {
        struct
        {
            float velocity;
            float radius;
            float theta;
            float phi;
        };
        float data_d[4];
    };

    union
    {
        struct
        {
            float velocity_direction;
            float acceleration;
            float acceleration_direction;
            float reconstruct_velocity;
        };
        float data_f[4];
    };

    inline _FMCWLidar_9D_PointT()
    {
        x = y = z = velocity0 = time = intensity = 0.0f;
        ring = 0;
        rgba = 0;
        velocity = radius = theta = phi = 0.0f;
        velocity_direction = acceleration = acceleration_direction = reconstruct_velocity = 0.0f;
    }

    inline _FMCWLidar_9D_PointT(const _FMCWLidar_9D_PointT& a)
    {
        x = a.x; y = a.y; z = a.z;
        rgba = a.rgba;
        velocity0 = a.velocity0; ring = a.ring; time = a.time; intensity = a.intensity;
        velocity = a.velocity; radius = a.radius; theta = a.theta; phi = a.phi;
        velocity_direction = a.velocity_direction;
        acceleration = a.acceleration;
        acceleration_direction = a.acceleration_direction;
        reconstruct_velocity = a.reconstruct_velocity;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

} EIGEN_ALIGN16;

// Register custom point type
POINT_CLOUD_REGISTER_POINT_STRUCT(_FMCWLidar_9D_PointT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, rgba, rgba)
    (float, velocity0, velocity0)
    (uint32_t, ring, ring)
    (float, time, time)
    (float, intensity, intensity)
    (float, velocity, velocity)
    (float, radius, radius)
    (float, theta, theta)
    (float, phi, phi)
    (float, velocity_direction, velocity_direction)
    (float, acceleration, acceleration)
    (float, acceleration_direction, acceleration_direction)
    (float, reconstruct_velocity, reconstruct_velocity)
)


typedef _FMCWLidar_9D_PointT                  Aq_9d_PointT;
typedef pcl::PointCloud<Aq_9d_PointT>         Aq_9d_PCT;
typedef pcl::PointCloud<Aq_9d_PointT>::Ptr    Aq_9d_PCT_Ptr;

typedef Aq_9d_PointT                          AQ_PointT;
typedef pcl::PointCloud<AQ_PointT>            AQ_PCloudT;
typedef pcl::PointCloud<AQ_PointT>::Ptr       AQ_PCloudTPtr;
typedef pcl::PointCloud<AQ_PointT>::ConstPtr  AQ_PCloudTcPtr;

#endif // __FMCW_POINT_DEFINE_H__
