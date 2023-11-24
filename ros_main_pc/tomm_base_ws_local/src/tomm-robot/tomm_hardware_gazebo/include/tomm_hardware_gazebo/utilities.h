#include <math.h>
#include <angles/angles.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <urdf_parser/urdf_parser.h>

#include <ros/ros.h>

#include <regex>

typedef Eigen::Vector3d eVector3;
typedef Eigen::Isometry3d eMatrixHom;
typedef Eigen::Matrix3d eMatrixRot;
typedef Eigen::Quaternion<double> eQuaternion;

namespace gazebo_ros_control
{

  /**
   * @brief extract pose from gzpose string
   * 
   * e.g. "-x 1.0 -y 0.0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0"
   * 
   * @param gzpose 
   * @return eVector3 
   */
  eVector3 extract_pose_gzpose(const std::string& gzpose)
  {
    eVector3 pose = eVector3::Zero();
    std::regex pattern("-?\\d+(?:\\.\\d+)?");
    std::smatch matches;
    std::string str = gzpose;
    ROS_WARN_STREAM(gzpose);
    while (regex_search(str, matches, pattern)) {
        std::string match = matches.prefix();
        match = std::regex_replace(match,std::regex("\\s"),"");
        if (match == "-x") {
            pose.x() = stof(matches[0]);
        } else if (match == "-y") {
            pose.y() = stof(matches[0]);
        } else if (match == "-Y") {
            pose.z() = stof(matches[0]);
        }
        str = matches.suffix();
    }
    return pose;
  }

  //////////////////////////////////////////////////////////////////////////////
  // utility functions
  //////////////////////////////////////////////////////////////////////////////

  namespace xh
  {
    class XmlrpcHelperException : public ros::Exception
    {
    public:
      XmlrpcHelperException(const std::string &what) : ros::Exception(what)
      {
      }
    };

    typedef XmlRpc::XmlRpcValue Struct;
    typedef XmlRpc::XmlRpcValue Array;

    template <class T>
    void fetchParam(ros::NodeHandle nh, const std::string &param_name, T &output)
    {
      XmlRpc::XmlRpcValue val;
      bool ok = false;
      try
      {
        ok = nh.getParam(param_name, val);
      }
      catch (const ros::InvalidNameException &e)
      {
      }

      if (!ok)
      {
        std::ostringstream err_msg;
        err_msg << "could not load parameter '" << param_name
                << "'. (namespace: " << nh.getNamespace() << ")";
        ROS_ERROR("fetchParam: %s", err_msg.str().c_str());
        throw XmlrpcHelperException(err_msg.str());
      }

      output = static_cast<T>(val);
    }
  }

  inline double shortest_angle_diff(double theta_goal, double theta)
  {
    return std::fmod(std::fabs(theta_goal - theta) + M_PI, 2 * M_PI) - M_PI;
  }

  inline std::vector<std::string> getIds(const ros::NodeHandle &nh, const std::string &key)
  {
    using std::string;
    using std::vector;

    xh::Struct xh_st;
    try
    {
      xh::fetchParam(nh, key, xh_st);
    }
    catch (const xh::XmlrpcHelperException &)
    {
      ROS_DEBUG_STREAM("Requested data found in the parameter server (namespace "
                       << nh.getNamespace() + "/" + key << ").");
      return vector<string>();
    }

    vector<string> out;
    for (xh::Struct::iterator it = xh_st.begin(); it != xh_st.end(); ++it)
    {
      out.push_back(it->first);
    }
    return out;
  }

  void convert(const urdf::Vector3 &in, eVector3 &out)
  {
    out = eVector3(in.x, in.y, in.z);
  }

  void convert(const urdf::Rotation &in, eMatrixRot &out)
  {
    out = eQuaternion(in.w, in.x, in.y, in.z);
  }

  inline eMatrixHom createMatrix(eMatrixRot const &rot, eVector3 const &trans)
  {
    eMatrixHom temp;
    temp.setIdentity();
    temp = (rot);
    temp.translation() = trans;
    return temp;
  }

  void convert(const urdf::Pose &in, eMatrixHom &out)
  {
    eVector3 r;
    convert(in.position, r);
    eMatrixRot E;
    convert(in.rotation, E);
    out = createMatrix(E, r);
  }

  template <typename T>
  Eigen::Matrix<T, 3, 3> skew(const Eigen::Matrix<T, 3, 1> &vec)
  {
    return (Eigen::Matrix<T, 3, 3>() << T(0), -vec(2), vec(1), vec(2), T(0), -vec(0),
            -vec(1), vec(0), T(0))
        .finished();
  }

}