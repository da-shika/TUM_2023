#ifndef WRENCH_CONE_WRENCH_CONE_SOLVER_H
#define WRENCH_CONE_WRENCH_CONE_SOLVER_H

#include <wrench_cone/polyhedron.h>

#include <control_core/types.h>

namespace wrench_cone
{

  /**
   * @brief WrenchConeSolver
   * 
   * Computes the Wrench cone in V and H-Rep for given 
   * contact. This class fills the H,V,G matrices inside cc::Contact
   * based on the geometry of that contact.
   */
  class WrenchConeSolver
  {
    public:
      WrenchConeSolver();
      ~WrenchConeSolver();

      /**
       * @brief computes the V and H-Rep of a given contact.
       * 
       * @param contact 
       */
      void compute(cc::Contact& contact);

    private:
      Polyhedron poly_;
  };
}

#endif