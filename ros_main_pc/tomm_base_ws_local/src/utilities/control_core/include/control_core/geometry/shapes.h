/*! \file
 *
 * \author Simon Armleder
 *
 * \copyright Copyright 2020 Institute for Cognitive Systems (ICS),
 *    Technical University of Munich (TUM)
 *
 * #### Licence
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#ifndef CONTROL_CORE_GEOMETRY_SHAPES_H
#define CONTROL_CORE_GEOMETRY_SHAPES_H

#include <control_core/geometry/i_shape.h>
#include <control_core/geometry/geometry_2d.h>

#include <geometry_msgs/Polygon.h>

// the namespace for the project
namespace cc
{
  /**
   * @brief Point Shape
   * 
   */
  class PointShape : public ShapeBase
  {
  public:
    PointShape() : ShapeBase(),
                   point_(Point2d::Zero())
    {
    }

    PointShape(Scalar x, Scalar y) : 
      ShapeBase(),
      point_((Point2d() << x, y).finished())
    {
    }

    PointShape(const Point2d &point) : 
      ShapeBase(), 
      point_(point)
    {
    }

    virtual ~PointShape()
    {
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param point 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    bool checkCollision(const Point2d &point, Scalar min_dist) const
    {
      return distance_points(point_, point) < min_dist;
    }

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param point 
     * @return Scalar 
     */
    Scalar minimumDistance(const Point2d &point) const
    {
      return distance_points(point_, point);
    }

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return Scalar 
     */
    Scalar minimumDistance(const Points2d &polygon) const
    {
      return distance_point_polygon(point_, polygon);
    }

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const Point2d 
     */
    const Point2d closestPoint(const Point2d &point) const
    {
      return point_;
    }

    /**
     * @brief get the position
     * 
     * @return const Point2d& 
     */
    const Point2d &position() const
    {
      return point_;
    }

    /**
     * @brief convert to a ros visualization maker msg
     * 
     * @return visualization_msgs::Marker 
     */
    virtual visualization_msgs::Marker toMarkerMsg(
      Scalar r=0.0, 
      Scalar g = 0.0, 
      Scalar b = 0.0, 
      Scalar a = 1.0,
      Scalar scale = 0.05) const
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::POINTS;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.scale.x = scale;
      marker.scale.y = scale;

      geometry_msgs::Point point;
      point.x = point_.x();
      point.y = point_.y();
      marker.points.push_back(point);

      return marker;
    }

  private:
    Point2d point_;
  };

  /**
   * @brief line like obstacle
   * 
   */
  class LineShape : public ShapeBase
  {
  public:
    LineShape() : ShapeBase(),
                  s_(Point2d::Zero()),
                  e_(Point2d::Zero())
    {
    }

    /**
     * @brief Construct a new Line Shape object
     * 
     * @param s 
     * @param e 
     */
    LineShape(const Point2d &s, const Point2d &e) : ShapeBase(),
                                                            s_(s),
                                                            e_(e)
    {
    }

    /**
     * @brief Construct a new Line Shape object
     * 
     * @param line 
     */
    LineShape(const LineShape &line) : ShapeBase(),
                                       s_(line.s_),
                                       e_(line.e_)
    {
    }

    virtual ~LineShape()
    {
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param position 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    bool checkCollision(const Point2d &point, Scalar min_dist) const
    {
      return distance_point_line_segment(point, s_, e_) < min_dist;
    }

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param point 
     * @return Scalar 
     */
    Scalar minimumDistance(const Point2d &point) const
    {
      return distance_point_line_segment(point, s_, e_);
    }

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return Scalar 
     */
    Scalar minimumDistance(const Points2d &polygon) const
    {
      return distance_line_polygon(s_, e_, polygon);
    }

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const Point2d 
     */
    const Point2d closestPoint(const Point2d &position) const
    {
      return closest_point_on_line_segment(position, s_, e_);
    }

    /**
     * @brief get the start position
     * 
     * @return const Point2d& 
     */
    const Point2d &start() const
    {
      return s_;
    }

    /**
     * @brief get the end position
     * 
     * @return const Point2d& 
     */
    const Point2d &end() const
    {
      return e_;
    }

    /**
   * @brief Calculate line segment length
   * 
   * @return const Sclar
   */
    Scalar length() const
    {
      return (e_ - s_).norm();
    }

    /**
     * @brief fit a line segment through points using least square fitting
     * See: https://mathworld.wolfram.com/LeastSquaresFitting.html
     * 
     * @param points 
     */
    void fit(const Points2d& points)
    {
      // fit line y = a*x + b 
      int n = points.cols(); // number of points
      Vector2 sum = points.rowwise().sum(); // [sum_x, sum_y]
      Vector2 sum_sq = points.array().square().rowwise().sum(); // [sum_xsq, sum_ysq]
      Scalar sum_xy = (points.row(0).cwiseProduct(points.row(1))).sum();

      Scalar denom = n * sum_sq[0] - sum[0] * sum[0];
      Scalar a = (n * sum_xy - sum[0] * sum[1]) / denom;
      Scalar b = (sum[1] * sum_sq[0] - sum[0] * sum_xy) / denom;

      // y = a*x + b --> w1*x + w2*y + b = 0; parameter vector w = [b w1 w2]
      Vector3 w{1, a/b, -1/b};    // optional: normalize the constant to 0

      // fit segment: project start and end points on the fitted line
      s_ = closest_point_on_line(points.leftCols(1), w);
      e_ = closest_point_on_line(points.rightCols(1), w);
    }

    /**
     * @brief convert to a ros visualization maker msg
     * 
     * @return visualization_msgs::Marker 
     */
    virtual visualization_msgs::Marker toMarkerMsg(
      Scalar r = 0.0, 
      Scalar g = 0.0, 
      Scalar b = 0.0, 
      Scalar a = 1.0,
      Scalar scale = 0.05) const
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.scale.x = scale;

      geometry_msgs::Point point;
      point.x = s_.x();
      point.y = s_.y();
      marker.points.push_back(point);
      point.x = e_.x();
      point.y = e_.y();
      marker.points.push_back(point);

      return marker;
    }

  private:
    Point2d s_;
    Point2d e_;
  };

  /**
   * @brief Circle Shape
   * 
   */
  class CircleShape : public ShapeBase
  {
  public:
    /**
     * @brief Construct a new Circle Shape object
     * 
     */
    CircleShape() : ShapeBase(),
                    center_(Point2d::Zero()),
                    radius_(0.0)
    {
    }

    /**
     * @brief Construct a new Circle Shape object
     * 
     * @param center 
     * @param radius 
     */
    CircleShape(const Point2d &center, Scalar radius) : ShapeBase(),
                                                                center_(center),
                                                                radius_(radius)
    {
    }

    /**
     * @brief Construct a new Circle Shape object
     * 
     * @param circle 
     */
    CircleShape(const CircleShape &circle) : ShapeBase(),
                                             center_(circle.center_),
                                             radius_(circle.radius_)
    {
    }

    /**
     * @brief Destroy the Circle Shape object
     * 
     */
    virtual ~CircleShape()
    {
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param point 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    bool checkCollision(const Point2d &point, Scalar min_dist) const
    {
      return ((point - center_).norm() - radius_) < min_dist;
    }

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param point 
     * @return Scalar 
     */
    Scalar minimumDistance(const Point2d &point) const
    {
      return distance_points(center_, point) - radius_;
    }

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return Scalar 
     */
    Scalar minimumDistance(const Points2d &polygon) const
    {
      return distance_point_polygon(center_, polygon) - radius_;
    }

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const Point2d 
     */
    const Point2d closestPoint(const Point2d &point) const
    {
      return center_ + radius_ * (point - center_).normalized();
    }

    /**
     * @brief get the radius
     * 
     * @return Scalar 
     */
    Scalar r() const
    {
      return radius_;
    }

    /**
     * @brief return the center
     * 
     * @return const Point2d& 
     */
    const Point2d &c() const
    {
      return center_;
    }

    /**
     * @brief convert to a ros visualization maker msg
     * 
     * @return visualization_msgs::Marker 
     */
    virtual visualization_msgs::Marker toMarkerMsg(
      Scalar r = 0.0, 
      Scalar g = 0.0, 
      Scalar b = 0.0, 
      Scalar a = 1.0,
      Scalar scale = 0.05) const
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.scale.x = scale;

      geometry_msgs::Point point;
      for(size_t i = 0; i < 30; ++i)
      {
        point.x = radius_*cos(2*M_PI*Scalar(i)/30.) + center_.x();
        point.y = radius_*sin(2*M_PI*Scalar(i)/30.) + center_.y();
        marker.points.push_back(point);
      }
      marker.points.push_back(marker.points.front());
      return marker;
    }

  private:
    Point2d center_;
    Scalar radius_;
  };

  /**
   * @brief Polygon Shape
   * 
   */
  class PolygonShape : public ShapeBase
  {
  public:
    typedef std::pair<MatrixX, VectorX> HalfSpace;

  public:
    /**
     * @brief Construct a new empty Polygon Shape object
     * 
     * @param vertices 
     */
    PolygonShape() : ShapeBase()
    {
    }

    /**
     * @brief Construct a new Polygon Shape object
     * 
     * @param vertices 
     */
    PolygonShape(const Points2d &vertices) : ShapeBase(), vertices_(vertices)
    {
    }

    /**
     * @brief Construct a new Polygon Shape object
     * 
     * @param vertices 
     */
    PolygonShape(Scalar x_min, Scalar x_max, Scalar y_min, Scalar y_max) :
      vertices_((cc::Points2d(2,4)<<
        x_min, x_max, x_max, x_min,
        y_min, y_min, y_max, y_max).finished())
    {
    }

    /**
     * @brief Construct a new Polygon Shape object
     * 
     * @param vertices 
     */
    PolygonShape(Scalar x_half, Scalar y_half) :
      vertices_((cc::Points2d(2,4)<<
        -x_half, +x_half, +x_half, -x_half,
        -y_half, -y_half, y_half, y_half).finished())
    {
    }

    /**
     * @brief Construct a new Polygon Shape object
     * 
     * @param polygon 
     */
    PolygonShape(const PolygonShape &polygon) : ShapeBase(),
                                                vertices_(polygon.vertices_)
    {
    }

    /**
     * @brief Destroy the Polygon Shape object
     * 
     */
    virtual ~PolygonShape()
    {
    }

    size_t size() const
    {
      return size_t(vertices_.cols());
    }

    /**
     * @brief check if polygon is of rectangular shape (four sided + 90 deg angles)
     * 
     * @return true 
     * @return false 
     */
    bool isRectangular()
    {
      if(vertices_.cols() != 4)
        return false;

      // check relative angles
      Point2d e1 = vertices_.col(1) - vertices_.col(0);
      Point2d e2;
      for (size_t i = 2; i < vertices_.cols(); ++i)
      {
        e2 = vertices_.col(i) - vertices_.col(i-1);
        if(e1.dot(e2) > 1e-6)
          return false;
        e1 = e2;
      }
      return true;
    }

    /**
     * @brief check the collision between shape and point
     * 
     * @param point 
     * @param min_dist 
     * @return true 
     * @return false 
     */
    bool checkCollision(const Point2d &point, Scalar min_dist) const
    {
      // inside
      if(point_polygon_intersect(vertices_, point))
      {
        return true;
      }

      if (min_dist == 0)
      {
        // on the edge
        return false;
      }

      // outside
      return minimumDistance(point) < min_dist;
    };

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param point 
     * @return Scalar 
     */
    Scalar minimumDistance(const Point2d &point) const
    {
      return distance_point_polygon(point, vertices_);
    };

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return Scalar 
     */
    Scalar minimumDistance(const Points2d &polygon) const
    {
      return distance_polygon_polygon(polygon, vertices_);
    }

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const Point2d 
     */
    const Point2d closestPoint(const Point2d &point) const
    {
      // a point
      if (vertices_.cols() == 1)
      {
        return vertices_.col(0);
      }

      // a line
      if (vertices_.cols() > 1)
      {
        Point2d pt = closest_point_on_line_segment(point, vertices_.col(0), vertices_.rightCols(1));

        // a polygon
        if (vertices_.cols() > 2)
        {
          Scalar dmin = (pt - point).norm();
          Point2d pt_min = pt;

          for (size_t i = 1; i < vertices_.cols() - 1; ++i)
          {
            pt = closest_point_on_line_segment(point, vertices_.col(i), vertices_.col(i-1));
            Scalar d = (pt - point).norm();
            if (d < dmin)
            {
              dmin = d;
              pt_min = pt;
            }
          }
          return pt_min;
        }
        return pt;
      }

      // error case
      return Point2d::Zero();
    };

    /**
     * @brief Get the vertices
     * 
     * @return const std::vector<Point2d>& 
     */
    const Points2d &vertices() const
    {
      return vertices_;
    }

    /**
     * @brief Get the vertices
     * 
     * @return std::vector<Point2d>& 
     */
    Points2d &vertices()
    {
      return vertices_;
    }

    /**
     * @brief compute the controid point
     * 
     * @return const Points2d 
     */
    Point2d centroid() const
    {
      return vertices_.rowwise().mean();
    }

    /**
     * @brief compute the maximum vertex
     * 
     * @return Point2d 
     */
    Point2d maxVertex() const 
    {
      return vertices_.rowwise().maxCoeff();
    }

    /**
     * @brief compute the minimum vertex
     * 
     * @return Point2d 
     */
    Point2d minVertex() const
    {
      return vertices_.rowwise().minCoeff();
    }

    /**
     * @brief return half extends of outer axis aligned rectangle
     * 
     * @return Point2d 
     */
    Point2d halfExtends() const
    {
      return 0.5*(maxVertex() - minVertex());
    }

    /**
     * @brief compute half space representation of polygon
     * Inequality A*x < b fullfilled if point inside 
     * 
     * @return HalfSpace 
     * 
     */
    HalfSpace halfSpace() const
    {
      HalfSpace half_space;
      MatrixX& A = half_space.first;
      VectorX& b = half_space.second;
      compute_half_space(vertices_, A, b);
      return half_space; 
    }

    /**
     * @brief clamp a point on hull
     * 
     * @param point 
     * @return Point2d 
     */
    Point2d clamp(const Point2d &point) const
    {
      if(!checkCollision(point, 0.0))
        return closestPoint(point);
      return point;
    }

    /**
     * @brief convert to a ros visualization maker msg
     * 
     * @return visualization_msgs::Marker 
     */
    virtual visualization_msgs::Marker toMarkerMsg(
      Scalar r = 0.0, 
      Scalar g = 0.0, 
      Scalar b = 0.0, 
      Scalar a = 1.0,
      Scalar scale = 0.05) const
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.color.r = r;
      marker.color.g = g;
      marker.color.b = b;
      marker.color.a = a;
      marker.scale.x = scale;

      geometry_msgs::Point point;
      for(int i = 0; i < vertices_.cols(); ++i) 
      {
          point.x = vertices_.col(i).x();
          point.y = vertices_.col(i).y();
          marker.points.push_back(point);
      }
      if (vertices_.size() > 2) 
      {
          point.x = vertices_.col(0).x();
          point.y = vertices_.col(0).y();
          marker.points.push_back(point);
      }
      return marker;
    }

    /*!
    * \brief Assignment of geometry_msgs::Polygon.
    */
    PolygonShape& operator=(const geometry_msgs::Polygon& msg)
    {
      vertices_.resize(2, msg.points.size());
      for(size_t i = 0; i < msg.points.size(); ++i)
      {
        vertices_.col(i) << msg.points[i].x, msg.points[i].y;
      }
      return *this;
    }

    /*!
    * \brief Conversion to geometry_msgs::Polygon.
    */
    operator geometry_msgs::Polygon() const
    {
      geometry_msgs::Polygon msg;
      msg.points.resize(vertices_.cols());
      for(size_t i = 0; i < msg.points.size(); ++i)
      {
        msg.points[i].x = vertices_.col(i).x();
        msg.points[i].y = vertices_.col(i).y();
        msg.points[i].z = 0.0;
      }
      return msg;
    }  

    /*!
    * \brief Conversion to geometry_msgs::Polygon.
    */
    geometry_msgs::Polygon toPolygonMsg() const
    {
      return static_cast<geometry_msgs::Polygon>(*this);
    }

  private:
    Points2d vertices_;
  };

} // namespace ow

#endif