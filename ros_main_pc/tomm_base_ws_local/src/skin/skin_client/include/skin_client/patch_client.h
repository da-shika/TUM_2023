#ifndef SKIN_CLIENT_PATCH_CLIENT_H_
#define SKIN_CLIENT_PATCH_CLIENT_H_

#include <skin_client/patch_base.h>

#include <pcl/point_types.h>
#include <pcl/cloud_iterator.h>
#include <std_msgs/Header.h>

namespace skin_client
{

  struct SkinPatchDataContainer : public SkinPatchDataContainerBase
  {
    // types
    typedef pcl::PointNormal CellPoint;                     // x,y,z,nx,ny,nz
    typedef pcl::PointXYZRGB DistancePoint;                 // x,y,z,r,g,b
    typedef pcl::PointCloud<CellPoint> CellCloud;
    typedef pcl::PointCloud<DistancePoint> DistanceCloud;
    typedef std::vector<double> Measurement;

    // sensor data (updated through callback)
    Measurement proximity;                                  // proximity measurment 0-1
    Measurement force;                                      // force measurment 0-1
    Measurement distance;                                   // distance between 0-max distance 
    
    // pointclouds (updated through functions)
    DistanceCloud::Ptr distance_cloud;                        // pcl pointcloud of proximity information (pos + distance)
    CellCloud::Ptr cell_cloud;                                // pcl pointcloud of cell positions  

    // active measurement (updated through functions)
    Indices active_id;
    Measurement active_proximity;
    Measurement active_force;
    Measurement active_distance;    
    CellCloud::Ptr active_cell_cloud;
  };

  /**
   * @brief SkinPatchClient Class
   * 
   * This class subscribes to the changing skin information of a single patch.
   * Data is stored in SkinPatchDataContainer and can be accesed with data().
   * 
   * Note: In the callback we copy the chaning skin information (force, prox).
   * The rest is only updated after calling updateAll().
   */
  class SkinPatchClient : public SkinPatchBase<SkinPatchDataContainer>
  {
  public:
    typedef std::shared_ptr<SkinPatchClient> Ptr;
    typedef SkinPatchDataContainer Data;
    typedef SkinPatchBase<Data> Base;
    typedef Eigen::Vector3i VectorRGB;

  protected:
    bool has_connection_;
    ros::Subscriber sub_;

    static VectorRGB min_dist_color_;
    static VectorRGB max_dist_color_;

  public:
    SkinPatchClient();
    virtual ~SkinPatchClient();

    ////////////////////////////////////////////////////////////////////////////
    // connection
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief check if valid connection to server
     * 
     */
    bool hasConnection() const;

    /**
     * @brief connect to server
     *
     */
    virtual bool enableDataConnection(ros::NodeHandle& nh) override;

    ////////////////////////////////////////////////////////////////////////////
    // computations
    ////////////////////////////////////////////////////////////////////////////

    /**
     * @brief compute proximimty pointcloud and store it in the field
     * proximity_cloud
     * 
     */
    void computePointCloud();

    /**
     * @brief compute the active cell indices based on signal
     * 
     */
    Data::Indices computeActiveDistanceIndices(double thresh = 0.02);
    Data::Indices computeActiveProximityIndices(double thresh = 0.1);
    Data::Indices computeActiveForceIndices(double thresh = 0.1);

    /**
     * @brief fill the active filds of the data structure for given indices
     */
    void extractActiveInformation(const Data::Indices& active);

  protected:
    bool load(
      double min_range=0.0, double max_range=1.0, bool verbose=false) override;

  private:
    /**
     * @brief server callback
     * 
     * @param msg 
     */
    void callback(const skin_client::SkinPatchDataConstPtr& msg);

  };

}

#endif