#include <wrench_cone/polyhedron.h>

namespace wrench_cone
{

  int Polyhedron::counter_ = 0;

  Polyhedron::Polyhedron() : 
    mat_ptr_{nullptr},
    polytope_{nullptr}
  {
    if(counter_ == 0)
      dd_set_global_constants();
  }

  Polyhedron::~Polyhedron()
  {
    counter_--;

    if(mat_ptr_)
      dd_FreeMatrix(mat_ptr_);
    if(polytope_)
      dd_FreePolyhedra(polytope_);

    if(counter_ == 0)
      dd_free_global_constants();
  }

  bool Polyhedron::setVRepresentation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
  {
    if(!hvrep(A,b,false))
      return false;
    return true;
  }

  bool Polyhedron::setHRepresentation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
  {
    if(!hvrep(A,b,true))
      return false;
    return true;
  }

  bool Polyhedron::vRepresentation(Eigen::MatrixXd& A, Eigen::VectorXd& b) const
  {
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    return ddfMatrix2EigenMatrix(mat, A, b, true);
  }

  bool Polyhedron::hRepresentation(Eigen::MatrixXd& A, Eigen::VectorXd& b) const
  {
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    return ddfMatrix2EigenMatrix(mat, A, b, false);
  }

  bool Polyhedron::hvrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool is_from_generators)
  {
    Eigen::MatrixXd cMat = concatenateMatrix(A, b, is_from_generators);
    return doubleDescription(cMat, is_from_generators);
  }

  void Polyhedron::initializeMatrixPtr(Eigen::Index rows, Eigen::Index cols, bool is_from_generators)
  {
    if(mat_ptr_)
      dd_FreeMatrix(mat_ptr_);
    mat_ptr_ = dd_CreateMatrix(rows, cols);
  }


  bool Polyhedron::doubleDescription(const Eigen::MatrixXd& matrix, bool is_from_generators)
  {
    initializeMatrixPtr(matrix.rows(), matrix.cols(), is_from_generators);

    for (auto row = 0; row < matrix.rows(); ++row)
      for (auto col = 0; col < matrix.cols(); ++col)
        mat_ptr_->matrix[row][col][0] = matrix(row, col);

    if (polytope_ != nullptr)
      dd_FreePolyhedra(polytope_);

    polytope_ = dd_DDMatrix2Poly(mat_ptr_, &err_);
    return (err_ == dd_NoError) ? true : false;
  }

  Eigen::MatrixXd Polyhedron::concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool is_from_generators)
  {
    double sign = (is_from_generators ? 1 : -1);
    Eigen::MatrixXd mat(A.rows(), A.cols() + 1);
    mat.col(0) = b;
    mat.block(0, 1, A.rows(), A.cols()).noalias() = sign * A;
    return mat;
  }

  bool Polyhedron::ddfMatrix2EigenMatrix(const dd_MatrixPtr mat, Eigen::MatrixXd& A, Eigen::VectorXd& b, bool is_output_vrep) const
  {
    if(mat == nullptr)
      return false;

    double sign = (is_output_vrep ? 1 : -1);
    auto rows = mat->rowsize;
    auto cols = mat->colsize;
    A.resize(rows, cols-1);
    b.resize(rows);
    for (auto row = 0; row < rows; ++row) {
      b(row) = mat->matrix[row][0][0];
      for (auto col = 1; col < cols; ++col)
        A(row, col - 1) = sign * mat->matrix[row][col][0];
    }
    return true;
  }

};