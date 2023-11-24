#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/surface/convex_hull.h>

#include <control_core/types.h>

#include <visualization_msgs/Marker.h>

namespace skin_visualizer
{
  inline Eigen::Vector3f calculate_normals(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3)
  {
    Eigen::Vector3f a = v2 - v1;
    Eigen::Vector3f b = v3 - v1;
    return a.cross(b).normalized();
  }

  inline Eigen::Vector3f calculate_center(pcl::PointCloud<pcl::PointXYZ>& points)
  {
    Eigen::Vector3f c = Eigen::Vector3f::Zero();
    for(size_t i = 0; i < points.size(); ++i)
      c = c.array() + points[i].getArray3fMap();
    return c / float(points.size());
  }

  inline bool is_facing_inwards(const Eigen::Vector3f& normal, const Eigen::Vector3f& direction)
  {
    return normal.dot(direction) > 0.0;
  }

  class WrenchConeVisualizer
  {
    public:
      typedef pcl::PointXYZ Point;
      typedef pcl::PointCloud<Point> Cloud;

    public:
      WrenchConeVisualizer(cc::Scalar scaling=0.1) :
        scaling_(scaling)
      {
        marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
        marker_.scale.x = 1.0;
        marker_.scale.y = 1.0;
        marker_.scale.z = 1.0;
      }

      virtual ~WrenchConeVisualizer()
      {
      }

      void compute(const cc::SkinPatch& patch, cc::Scalar mu)
      {
        compute(patch.pose(), patch.force().hull(), mu);
      }

      void compute(const cc::Contact& contact)
      {
        compute(contact.pose(), contact.hull(), contact.friction());
      }

      void compute(const cc::CartesianPosition& pose, const cc::PolygonShape geometry, cc::Scalar mu)
      {
        Cloud::Ptr cloud;
        cloud.reset(new Cloud);

        size_t n = geometry.size();
        const auto& points = geometry.vertices();

        // collect all vertices in local frame
        cc::MatrixX vertices(n*5,3);
        cc::Vector3 p;
        for(size_t i = 0; i < n; ++i)
        {
          p << points.col(i), 0;
          vertices.block(i*5,0,5,3) = cc::pointContactVertices(mu, p, scaling_);
        }

        // transform them into joint frame
        cc::Vector3 point_global, point_local;
        for(size_t i = 0; i < n*5; ++i)
        {
          point_local = vertices.row(i).transpose();
          point_global = pose*point_local;
          cloud->push_back(pcl::PointXYZ(point_global[0], point_global[1], point_global[2]));
        }

        // find hull
        compute(cloud);
      }

      void compute(Cloud::Ptr cloud)
      {
        if(cloud->empty())
          return;

        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cloud);

        Cloud vertices;
        std::vector<pcl::Vertices> triangles;
        chull.reconstruct(vertices, triangles);

        Eigen::Vector3f center = calculate_center(vertices);
        
        marker_.points.clear();
        geometry_msgs::Point point;
        for(std::size_t i = 0; i < triangles.size(); ++i)
        {
          const pcl::Vertices& face = triangles[i];
          const auto& v1 = vertices[face.vertices[0]].getArray3fMap();
          const auto& v2 = vertices[face.vertices[1]].getArray3fMap();
          const auto& v3 = vertices[face.vertices[2]].getArray3fMap();
          Eigen::Vector3f n = calculate_normals(v1, v2, v3);
          Eigen::Vector3f d = center.array() - (v1 + v2 + v3)/3.;

          if(is_facing_inwards(n, d))
          {
            for(int j = 0; j < 3; ++j)
            {
              point.x = vertices[face.vertices[j]].x;
              point.y = vertices[face.vertices[j]].y;
              point.z = vertices[face.vertices[j]].z;
              marker_.points.push_back(point);
            }
          }
          else
          {
            for(int j = 2; j >= 0; --j)
            {
              point.x = vertices[face.vertices[j]].x;
              point.y = vertices[face.vertices[j]].y;
              point.z = vertices[face.vertices[j]].z;
              marker_.points.push_back(point);
            }
          }
        }
      }

      virtual visualization_msgs::Marker toMarkerMsg(
        double r = 0.0, 
        double g = 0.0, 
        double b = 0.0, 
        double a = 1.0)
      {
        marker_.color.r = r;
        marker_.color.g = g;
        marker_.color.b = b;
        marker_.color.a = a;
        return marker_;
      }

    private:
      cc::Scalar scaling_;
      visualization_msgs::Marker marker_;
  };


};