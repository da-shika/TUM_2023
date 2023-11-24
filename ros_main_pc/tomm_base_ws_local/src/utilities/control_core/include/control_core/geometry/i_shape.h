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

#ifndef CONTROL_CORE_GEOMETRY_ISHAPES_H
#define CONTROL_CORE_GEOMETRY_ISHAPES_H

#include <control_core/primitive_types.h>
#include <visualization_msgs/Marker.h>

// the namespace for the project
namespace cc
{
  /**
   * @brief The Base class for all shapes
   * 
   */
  class ShapeBase
  {
  public:
    /**
     * @brief Construct a new Shape Base object
     * 
     */
    ShapeBase()
    {
    }

    /**
     * @brief Destroy the Shape Base object
     * 
     */
    virtual ~ShapeBase()
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
    virtual bool checkCollision(const Vector2 &point, Scalar min_dist) const = 0;

    /**
     * @brief return the minimum distance between shape and point
     * 
     * @param position 
     * @return Scalar 
     */
    virtual Scalar minimumDistance(const Vector2 &position) const = 0;

    /**
     * @brief return the minimum distance between shape and polygon
     * 
     * @param polygon 
     * @return Scalar 
     */
    virtual Scalar minimumDistance(const Points2d &polygon) const = 0;

    /**
     * @brief return the closest point on the shape
     * 
     * @param point 
     * @return const Vector2 
     */
    virtual const Vector2 closestPoint(const Vector2 &point) const = 0;

    /**
     * @brief fit the shape to given set of points
     * 
     * @param points 
     */
    virtual void fit(const Points2d& points)
    {
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
      Scalar scale = 0.05) const = 0; 
  };

}

#endif