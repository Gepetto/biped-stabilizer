#include <Eigen/Eigen>
#include <eigenpy/memory.hpp>
#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>

#include "biped_stabilizer/python.hpp"
#include "biped_stabilizer/cop_stabilizer.hpp"

BOOST_PYTHON_MODULE(biped_stabilizer) {
    // Enabling eigenpy support, i.e. numpy/eigen compatibility.
    eigenpy::enableEigenPy();
    eigenpy::exposeQuaternion();
    ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::Vector3d);
    ENABLE_SPECIFIC_MATRIX_TYPE(Eigen::Matrix3d);

    biped_stabilizer::python::exposeIsometry3d();   
    biped_stabilizer::python::exposeCopStabilizer();
}