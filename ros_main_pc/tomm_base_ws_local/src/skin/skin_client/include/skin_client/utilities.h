#ifndef SKIN_CLIENT_UTILITIES_H_
#define SKIN_CLIENT_UTILITIES_H_

#include <skin_client/pugixml/pugixml.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <numeric>

namespace skin_client
{

  //////////////////////////////////////////////////////////////////////////////
  // utility functions
  //////////////////////////////////////////////////////////////////////////////

  inline bool endsWith(const std::string& str, const std::string& suffix)
  {
    return str.size() >= suffix.size() && 0 == str.compare(str.size()-suffix.size(), suffix.size(), suffix);
  }

  inline bool startsWith(const std::string& str, const std::string& prefix)
  {
    return str.size() >= prefix.size() && 0 == str.compare(0, prefix.size(), prefix);
  }

  inline std::vector<std::string> split(const std::string& s, char delimiter)
  {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream token_stream(s);
    while (std::getline(token_stream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
  }

  template <typename T>
  inline std::vector<size_t> sort_indexes(const std::vector<T> &v) 
  {
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::stable_sort(idx.begin(), idx.end(),
        [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});
    return idx;
  }

  /**
   * @brief extract the path name from a string holding the path to a file
   * 
   * Extracts from 
   *  /src/h1_robot/tum_ics_h1_configs/SkinConfigs/p101e.xml
   * The patch name
   *  p101e
   * 
   * @param path 
   * @return std::string 
   */
  inline std::string patch_name_from_file_url(const std::string& path)
  {
    std::string name = path;
    name = split(name, '/').back();
    name = split(name, '.').front();
    return name;
  }

  /**
   * @brief sorts patches based on their names. This is useful to allways load
   * them in the same order.
   * 
   * @param names vector of patch names / patch urls
   * @return std::vector<std::string> sorted vector of patch names
   */
  inline std::vector<std::string> sort_patch_names(const std::vector<std::string>& names)
  {
    std::vector<std::string> patches;
    for(const auto& name : names)
      patches.push_back(patch_name_from_file_url(name));
    auto indices = sort_indexes(patches);

    std::vector<std::string> sorted(indices.size());
    for(size_t i = 0; i < indices.size(); ++i)
      sorted[i] = names[indices[i]];
    return sorted;
  }

  //////////////////////////////////////////////////////////////////////////////
  // skin config loading
  //////////////////////////////////////////////////////////////////////////////

  /**
   * @brief SkinCell configuration structure
   * 
   */
  struct SkinCellConfig
  {
    int id;
    Eigen::Affine3d T_cell_root;
    std::vector<int> neighbors;
  };

  /**
   * @brief SkinPatch configuration structure
   * 
   */
  struct SkinPatchConfig
  {
    int id;
    std::string base_frame;
    std::string topic_prefix;
    Eigen::Affine3d T_base_0;
    Eigen::Affine3d T_root_base;
    std::vector<SkinCellConfig> cells;
  };

  /**
   * @brief parse matrix node
   * 
   * @param mat_node 
   * @param mat 
   * @param desired_rows 
   * @param desired_cols 
   * @return true 
   * @return false 
   */
  inline bool parse_matrix(const pugi::xml_node& mat_node, Eigen::MatrixXd& mat, 
    int desired_rows=Eigen::Dynamic, int desired_cols=Eigen::Dynamic)
  {
    typedef Eigen::MatrixXd::Index Index;
    Index rows = mat_node.attribute("rows").as_int();
    Index cols = mat_node.attribute("cols").as_int();

    if(rows <= 0 || cols <= 0)
      return false;

    if(desired_rows == Eigen::Dynamic)
      desired_rows = rows;
    if(desired_cols == Eigen::Dynamic)
      desired_cols = cols;
    if(desired_rows != rows || desired_cols != cols)
      return false;

    mat.resize(rows, cols);
    for (pugi::xml_node elem : mat_node.children("element"))
    {
      Index i = elem.attribute("row").as_int();
      Index j = elem.attribute("col").as_int();
      if(i >= mat.rows() || j >= mat.rows())
        return false;
      mat(i,j) = std::stod(elem.child_value());
    }

    return true;
  }

  /**
   * @brief parse pos node
   * 
   * @param pos_node 
   * @param pos 
   * @return true 
   * @return false 
   */
  inline bool parse_pos(const pugi::xml_node& pos_node, Eigen::Vector3d& pos)
  {
    pugi::xml_node mat_node = pos_node.child("matrix");
    Eigen::MatrixXd mat;
    if(!parse_matrix(mat_node, mat, 3, 1))
      return false;
    pos = mat;
    return true;
  }

  /**
   * @brief parse rot node
   * 
   * @param rot_node 
   * @param rot 
   * @return true 
   * @return false 
   */
  inline bool parse_rot(const pugi::xml_node& rot_node, Eigen::Matrix3d& rot)
  {
    pugi::xml_node mat_node = rot_node.child("matrix");
    Eigen::MatrixXd mat;
    if(!parse_matrix(mat_node, mat, 3, 3))
      return false;
    rot = mat;
    return true;
  }

  /**
   * @brief parse transformation inside cell, tf_base_dh, tf_rootCell_base node
   * 
   * @param transf_node 
   * @param T 
   * @return true 
   * @return false 
   */
  inline bool parse_transf(const pugi::xml_node& transf_node, Eigen::Affine3d& T)
  {
    pugi::xml_node pos_node = transf_node.child("pos");
    pugi::xml_node rot_node = transf_node.child("rot");
    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
    if(!parse_pos(pos_node, pos) || !parse_rot(rot_node, rot))
      return false;

    T = Eigen::Translation3d(pos)*Eigen::AngleAxisd(rot);
    return true;
  }

  inline bool parse_neighbors(const pugi::xml_node& neighbors_node, std::vector<int>& ids)
  {
    int n = neighbors_node.attribute("size").as_int();
    if(n < 0)
      return false;

    for (pugi::xml_node elem : neighbors_node.children("neighbor"))
    {
      int id = elem.attribute("id").as_int();
      ids.push_back(id);
    }
    return true;
  }

  /**
   * @brief parse cell node
   * 
   * @param cell_node 
   * @param cell 
   * @return true 
   * @return false 
   */
  inline bool parse_cell(const pugi::xml_node& cell_node, SkinCellConfig& cell)
  {
    cell.id = cell_node.attribute("id").as_int();
    if(!parse_transf(cell_node, cell.T_cell_root))
      return false;
    if(!parse_neighbors(cell_node.child("neighbors"), cell.neighbors))
      return false;
    return true;
  }

  /**
   * @brief parse SkinConfig node
   * 
   * @param patch_node 
   * @param patch 
   * @return true 
   * @return false 
   */
  inline bool parse_patch(const pugi::xml_node& patch_node, SkinPatchConfig& patch)
  {
    double number_of_cells = patch_node.attribute("numberOfCells").as_double();
    patch.cells.reserve(number_of_cells);
    patch.id = patch_node.child("patchDriverSettings").attribute("patchId").as_int();
    patch.topic_prefix = patch_node.child("patchDriverSettings").attribute("prefix").as_string();
    patch.base_frame = patch_node.child("patchExtrinsics").attribute("baseFrame").as_string();

    // root cell transformations
    pugi::xml_node root_cell_base_node = patch_node.child("patchExtrinsics").child("tf_base_dh");
    if(!parse_transf(root_cell_base_node, patch.T_base_0))
      return false;
    pugi::xml_node root_cell_transf_node = patch_node.child("patchExtrinsics").child("tf_rootCell_base");
    if(!parse_transf(root_cell_transf_node, patch.T_root_base))
      return false;

    // patch cells
    for(pugi::xml_node cell_elem : patch_node.children("cell"))
    {
      SkinCellConfig cell;
      if(!parse_cell(cell_elem, cell))
        return false;
      patch.cells.push_back(cell);
    }
    return true;
  }

  /**
   * @brief search for index in cell array based on cell id
   * 
   */
  inline int id_to_idx(const std::vector<skin_client::SkinCellConfig>& cells, int id)
  {
    auto it = std::find_if(cells.begin(), cells.end(), [id] (const skin_client::SkinCellConfig& s) { 
      return s.id == id; } );
    return it == cells.end() ? -1 : std::distance(cells.begin(), it);
  }

}

#endif
