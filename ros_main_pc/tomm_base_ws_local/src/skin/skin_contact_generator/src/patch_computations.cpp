#include <skin_contact_generator/patch_computations.h>

////////////////////////////////////////////////////////////////////////////////
// pcl includes
////////////////////////////////////////////////////////////////////////////////
#include <pcl/common/centroid.h>

////////////////////////////////////////////////////////////////////////////////
// opencv2 includes
////////////////////////////////////////////////////////////////////////////////
#include <opencv2/imgproc.hpp>
#include <tf_conversions/tf_eigen.h>

////////////////////////////////////////////////////////////////////////////////
// opencv2 includes
////////////////////////////////////////////////////////////////////////////////
#include <control_core/geometry/shapes.h>


namespace skin_contact_generator
{

  void computeContactFrame(
    const skin_client::SkinPatchDataContainer::CellCloud& cells, 
    const skin_client::SkinPatchDataContainer::Indices& indices,
    cc::CartesianPosition& pose)
  {
    // cenroid
    Eigen::Vector4f centroid_b;
    pcl::compute3DCentroid(cells, indices, centroid_b);

    // take normal vector from skin configuration
    Eigen::Vector3f n_average(0,0,0);
    for(auto idx : indices)
      n_average += cells[idx].getNormalVector3fMap();
    
    // build a rotation with z-axis pointing in cell normal direction
    cc::AngularPosition Q_n = cc::AngularPosition::FromTwoVectors(
      cc::Vector3(0,0,1), n_average.cast<double>());

    // transformation contact wrt joint
    cc::CartesianPosition X_c_j;
    X_c_j.pos() = centroid_b.head(3).cast<double>();
    X_c_j.angular() = Q_n;

    // transformation joint wrt contact
    cc::CartesianPosition X_j_c = X_c_j.inverse();

    // compute the 2d covariance matrix of active contact cloud
    cc::Vector3 x_c;
    cc::Matrix2 Cov = cc::Matrix2::Zero();
    for(auto idx : indices)
    {
      x_c = X_j_c.angular() * cells[idx].getArray3fMap().cast<double>() + X_j_c.pos();
      Cov += x_c.head(2)*x_c.head(2).transpose();
    }
    Cov /= cc::Scalar(indices.size());
    
    // build the 2d roation matrix
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(Cov, Eigen::ComputeEigenvectors);
    Eigen::Rotation2Dd R2(eigen_solver.eigenvectors());

    // build the 3d roation matrix
    pose.pos() = X_c_j.pos();
    pose.angular() = X_c_j.angular() * Eigen::AngleAxisd(R2.smallestPositiveAngle(), Eigen::Vector3d::UnitZ());
  }

  void computeContactWrench(
    const skin_client::SkinPatchDataContainer::CellCloud& cells, 
    const skin_client::SkinPatchDataContainer::Indices& indices, 
    const skin_client::SkinPatchDataContainer::Measurement& force, 
    const skin_client::SkinPatchDataContainer::Measurement& proximity,
    const skin_client::SkinPatchDataContainer::Measurement& distance,
    const Frame& frame,
    cc::SkinPatch& patch)
  {
    // reset state
    patch.force().setZero();
    patch.proximity().setZero();
    patch.minDist() = 1.0;
    patch.maxDist() = 0.0;
    patch.force().min() = 1.0;
    patch.force().max() = 0.0;
    patch.proximity().min() = 1.0;
    patch.proximity().max() = 0.0;

    // transformation in contact frame
    auto X_joint_contact = patch.pose().inverse();

    cc::Wrench& W_f = patch.force().wrench();
    cc::Wrench& W_p = patch.proximity().wrench();

    cc::LinearPosition& p_f = patch.force().centerOfPressure();
    cc::LinearPosition& p_p = patch.proximity().centerOfPressure();

    cc::Scalar& n_f = patch.force().area();
    cc::Scalar& n_p = patch.proximity().area();

    cc::Scalar& p_min = patch.proximity().min();
    cc::Scalar& p_max = patch.proximity().max();
    cc::Scalar& f_min = patch.force().min();
    cc::Scalar& f_max = patch.force().max();

    double f_cell, p_cell, d_cell;
    double f_accumulated = 0, p_accumulated = 0;
    cc::Vector3 n, x;
    for(auto idx : indices)
    {
      if(frame == Frame::CONTACT) 
      {
        // cell pos and normal in contact frame
        x = X_joint_contact.angular() * cells[idx].getArray3fMap().cast<double>() + X_joint_contact.pos();
        n = X_joint_contact.angular() * cells[idx].getNormalVector3fMap().cast<double>();
      }
      else
      {
        // cell pos and normal in joint frame
        x = cells[idx].getArray3fMap().cast<double>();
        n = cells[idx].getNormalVector3fMap().cast<double>();
      }

      // data
      f_cell = force[idx];
      p_cell = proximity[idx];
      d_cell = distance[idx];

      // force
      W_f.force() += f_cell*n;
      W_f.moment() += f_cell*x.cross(n);
      p_f += f_cell*x;
      f_accumulated += f_cell;
      n_f += 1;

      // proximity
      W_p.force() += p_cell*n;
      W_p.moment() += p_cell*x.cross(n);
      p_p += p_cell*x;
      p_accumulated += p_cell;
      n_p += 1;

      // force range
      if(f_min > f_cell)
        f_min = f_cell;
      if(f_max < f_cell)
        f_max = f_cell;

      // distance, proximity range
      if(patch.minDist() > d_cell) {
        patch.minDist() = d_cell;
        p_max = p_cell;
      }
      if(patch.maxDist() < d_cell) {
        patch.maxDist() = d_cell;
        p_min = p_cell;
      }
    }

    if(n_f > 0 && f_accumulated > 0.0)
    {
      p_f = p_f/f_accumulated;
      W_f = W_f/cc::Scalar(n_f);
    }
    if(n_p > 0 && p_accumulated > 0.0)
    {
      p_p = p_p/p_accumulated;
      W_p = W_p/cc::Scalar(n_p);
    }
  }

  void computeContactHull(
    const skin_client::SkinPatchDataContainer::CellCloud& cells, 
    const skin_client::SkinPatchDataContainer::Indices& indices,
    const Frame& frame,
    cc::SkinPatch& patch)
  {
    int n = indices.size();
    std::vector<cv::Point2f> points(n);
    std::vector<cv::Point2f> hull, approx;

    // transform into contact frame and project to 2d
    cc::CartesianPosition X_j_c = patch.pose().inverse();
    cc::Vector3 x_c; int idx;
    for(size_t i = 0; i < n; ++i)
    {
      idx = indices[i];
      if(frame == CONTACT)
        x_c = X_j_c.angular() * cells[idx].getArray3fMap().cast<double>() + X_j_c.pos();
      else
        x_c = cells[idx].getArray3fMap().cast<double>();                        // TODO this only works on flat!
      points[i].x = x_c.x(); points[i].y = x_c.y();
    }

    // build simplifed form
    cv::convexHull(points, hull);
    cv::approxPolyDP(hull, approx, 0.015, true);

    // store the points in side contact hull
    auto& vertices = patch.force().hull().vertices();
    vertices.resize(2, approx.size());
    for(size_t i = 0; i < approx.size(); ++i)
    {
      vertices.col(i) << approx[i].x, approx[i].y;
    }

    // for now, lets assume same shape for proximity                            // TODO split active force, active proximity
    patch.proximity().hull().vertices() = vertices;
  }

};