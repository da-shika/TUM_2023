#ifndef WRENCH_CONE_POLYHEDRON_H_
#define WRENCH_CONE_POLYHEDRON_H_

#include <Eigen/Dense>

#include <cddlib/setoper.h>
#include <cddlib/cdd.h>

namespace wrench_cone
{

  class Polyhedron
  {
    public:
      Polyhedron();
      ~Polyhedron();

      bool setVRepresentation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

      bool setHRepresentation(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

      bool vRepresentation(Eigen::MatrixXd& A, Eigen::VectorXd& b) const;

      bool hRepresentation(Eigen::MatrixXd& A, Eigen::VectorXd& b) const;

    private:
      bool hvrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool is_from_generators);
      
      void initializeMatrixPtr(Eigen::Index rows, Eigen::Index cols, bool is_from_generators);

      bool doubleDescription(const Eigen::MatrixXd& matrix, bool isFromGenerators);

      Eigen::MatrixXd concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool is_from_generators);

      bool ddfMatrix2EigenMatrix(const dd_MatrixPtr mat, Eigen::MatrixXd& A, Eigen::VectorXd& b, bool is_output_vrep) const;

    private:
      static int counter_;
      dd_MatrixPtr mat_ptr_;
      dd_PolyhedraPtr polytope_;
      dd_ErrorType err_;
  };

}

#endif