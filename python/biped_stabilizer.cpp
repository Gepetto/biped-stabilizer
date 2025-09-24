#include <Eigen/Eigen>
#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>
#include <eigenpy/memory.hpp>

#include "biped-stabilizer/cop_stabilizer.hpp"
#include "biped-stabilizer/python.hpp"

BOOST_PYTHON_MODULE(biped_stabilizer_cpp) {
  // Enabling eigenpy support, i.e. numpy/eigen compatibility.
  eigenpy::enableEigenPy();
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::Vector2d);
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::Vector3d);
  ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::Matrix3d);
  eigenpy::exposeQuaternion();

  biped_stabilizer::python::exposeIsometry3d();
  biped_stabilizer::python::exposeCopStabilizer();
}
