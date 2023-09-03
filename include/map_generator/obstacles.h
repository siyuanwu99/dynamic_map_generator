/**
 * @file obstacles.h
 * @author Siyuan Wu (siyuanwu99@gmail.com)
 * @brief
 * @version 1.0
 * @date 2023-09-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#ifndef __OBSTACLES_H__
#define __OBSTACLES_H__
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <vector>

/**
 * @class Obstacle
 * @brief base class for all obstacles
 * @details
 * The obstacle is defined as a position, a velocity and a size
 *
 */
class Obstacle {
 public:
  Obstacle(const Eigen::Vector3f &p, const Eigen::Vector3f &v, const Eigen::Vector3f &s)
      : cloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>())
      , position(p)
      , velocity(v)
      , size(s)
      , q(Eigen::Quaternionf::Identity()) {}
  virtual ~Obstacle() {}
  virtual void render(float resolution) const = 0;
  void         update(float dt,
                      float x_l = -10,
                      float x_h = 10,
                      float y_l = -10,
                      float y_h = 10,
                      float z_l = -10,
                      float z_h = 10) {
    position += velocity * dt;
    if (position.z() < z_l) {
      position.z() = z_l;
      velocity.z() = -velocity.z();
    }
    if (position.z() > z_h) {
      position.z() = z_h;
      velocity.z() = -velocity.z();
    }
    if (position.x() < x_l) {
      position.x() = x_l;
      velocity.x() = -velocity.x();
    }
    if (position.x() > x_h) {
      position.x() = x_h;
      velocity.x() = -velocity.x();
    }
    if (position.y() < y_l) {
      position.y() = y_l;
      velocity.y() = -velocity.y();
    }
    if (position.y() > y_h) {
      position.y() = y_h;
      velocity.y() = -velocity.y();
    }
  }

  void getCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) const {
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << position.x(), position.y(), position.z();
    transform.rotate(q);
    pcl::transformPointCloud(*cloud, *cloud_ptr, transform);
  }

  Eigen::Vector3f getPosition() const { return position; }
  Eigen::Vector3f getVelocity() const { return velocity; }
  Eigen::Vector3f getBBox() const { return size; }

  Eigen::Quaternionf getQuaternion() const { return q; }

 protected:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  Eigen::Vector3f    position;
  Eigen::Vector3f    velocity;
  Eigen::Vector3f    size;
  Eigen::Quaternionf q;
};

/**
 * @class AABB
 * @brief axis-aligned bounding box obstacle
 *
 */
class AABB : public Obstacle {
 public:
  AABB(const Eigen::Vector3f &p, const Eigen::Vector3f &v, const Eigen::Vector3f &s)
      : Obstacle(p, v, s) {}
  void render(float resolution) const override {
    Eigen::Vector3f min = -size / 2;
    Eigen::Vector3f max = size / 2;
    for (float x = min.x(); x <= max.x(); x += resolution) {
      for (float y = min.y(); y <= max.y(); y += resolution) {
        for (float z = min.z(); z <= max.z(); z += resolution) {
          cloud->push_back(pcl::PointXYZ(x, y, z));
        }
      }
    }
    cloud->width    = cloud->points.size();
    cloud->height   = 1;
    cloud->is_dense = true;
  }
};
/**
 * @class Cylinder
 * @brief cylinder obstacle
 * @details
 * The cylinder is defined by a circle and a height
 *
 */
class Cylinder : public Obstacle {
 public:
  Cylinder(const Eigen::Vector3f &p,
           const Eigen::Vector3f &v,
           const float           &radius,
           const float           &height)
      : Obstacle(p, v, Eigen::Vector3f(2 * radius, 2 * radius, height))
      , radius(radius)
      , height(height) {}
  void render(float resolution) const override {
    Eigen::Vector3f min = -size / 2;
    Eigen::Vector3f max = size / 2;
    for (float x = min.x(); x <= max.x(); x += resolution) {
      for (float y = min.y(); y <= max.y(); y += resolution) {
        for (float z = min.z(); z <= max.z(); z += resolution) {
          if (x * x + y * y <= radius * radius) {
            cloud->push_back(pcl::PointXYZ(x, y, z));
          }
        }
      }
    }
    cloud->width    = cloud->points.size();
    cloud->height   = 1;
    cloud->is_dense = true;
  }

 protected:
  float radius;
  float height;
};

/**
 * @class CircleGate
 * @brief a circle gate which robot can fly through
 *
 * @details
 * The circle gate is defined by a circle, a thickness and an angle
 * The theta is defined as the theta between the normal of the circle and the x axis
 * The radius is the radius of the outer circle
 *
 */
class CircleGate : public Obstacle {
 public:
  CircleGate(const Eigen::Vector3f &p,
             const Eigen::Vector3f &v,
             const float           &radius,
             const float           &thickness,
             const float           &alpha,
             const float           &theta)
      : Obstacle(
            p, v, Eigen::Vector3f(2 * radius * sin(theta), 2 * radius * cos(theta), 2 * radius))
      , radius(radius)
      , thickness(thickness)
      , alpha(alpha)
      , theta(theta) {
    q = Eigen::Quaternionf(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
  }
  void render(float resolution) const override {
    float da = resolution / radius;
    for (float a = 0; a <= 2 * M_PI; a += da) {
      for (float r = radius - alpha; r <= radius; r += resolution) {
        for (float d = -thickness / 2; d <= thickness / 2; d += resolution) {
          float x = d;
          float y = r * cos(a);
          float z = r * sin(a);
          cloud->push_back(pcl::PointXYZ(x, y, z));
        }
      }
    }
    cloud->width    = cloud->points.size();
    cloud->height   = 1;
    cloud->is_dense = true;
  };

  float getAlpha() const { return alpha; }
  float getTheta() const { return theta; }

 protected:
  float radius;
  float thickness; /* thickness of the circle */
  float alpha;     /* distance between inner and outer circle */
  float theta;
};

/* ----- Parameters ----- */
struct ObstacleConfig {
  int   num;
  float size_x_min;
  float size_x_max;
  float size_y_min;
  float size_y_max;
  float size_z_min;
  float size_z_max;
};

struct AABBConfig {
  int   num{0};
  float size_x_min{0};
  float size_x_max{0};
  float size_y_min{0};
  float size_y_max{0};
  float size_z_min{0};
  float size_z_max{0};
};

struct CylinderConfig {
  int   num{0};
  float radius_min{0};
  float radius_max{0};
  float height_min{0};
  float height_max{0};
};

struct CircleGateConfig {
  int   num{0};
  float radius_min{0};
  float radius_max{0};
  float thickness_max{0};
  float alpha_min{0};
  float alpha_max{0};
  float theta_max{0};
};

#endif  // __OBSTACLES_H__
