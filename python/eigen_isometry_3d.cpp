/**
 * @file
 * @license unknown
 * @copyright Copyright (c) 2022, PAL, CNRS
 *
 * @brief Python bindings for the Isometry3d class.
 */

#include <Eigen/Eigen>
#include <boost/python/def.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>
#include <eigenpy/memory.hpp>
#include <iostream>
#include <sstream>

#include "biped-stabilizer/python.hpp"

namespace bp = boost::python;
namespace biped_stabilizer {
namespace python {
std::shared_ptr<Eigen::Isometry3d> wrapTransformConstructor() {
  return std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity());
}

Eigen::Vector3d getPoseTranslation(Eigen::Isometry3d& self) {
  return self.translation();
}

Eigen::Matrix3d getPoseRotation(Eigen::Isometry3d& self) {
  return self.rotation().matrix();
}

void setPoseTranslation(Eigen::Isometry3d& self,
                        const Eigen::Vector3d& translation) {
  self.translation() = translation;
}

void setPoseRotationQuaternion(Eigen::Isometry3d& self,
                               const Eigen::Quaterniond& quat) {
  const Eigen::Vector3d trans = self.translation();
  self = quat.matrix();
  self.translation() = trans;
}

void setPoseRotationMatrix(Eigen::Isometry3d& self,
                           const Eigen::Matrix3d& rot) {
  const Eigen::Vector3d trans = self.translation();
  self = rot;
  self.translation() = trans;
}

std::ostream& operator<<(std::ostream& os, Eigen::Isometry3d& pose) {
  os << "Translation : " << pose.translation().transpose() << std::endl
     << "Rotation : " << std::endl
     << pose.rotation().matrix();
  return os;
}

std::ostream& wrapperDisplayPose(std::ostream& os,
                                 const Eigen::Isometry3d& pose) {
  os << "Translation : " << pose.translation().transpose() << std::endl
     << "Rotation : " << std::endl
     << pose.rotation().matrix();
  return os;
}

void printPose(Eigen::Isometry3d& self) { std::cout << self << std::endl; }

std::string toString(Eigen::Isometry3d& self) {
  std::ostringstream oss;
  oss << self << std::endl;
  return oss.str();
}

void exposeIsometry3d() {
  boost::python::class_<Eigen::Isometry3d>("eMatrixHom")
      .def("__init__", make_constructor(&wrapTransformConstructor,
                                        boost::python::default_call_policies()))
      .def("display", &printPose, boost::python::args("self"))
      .def("__repr__", &toString, boost::python::args("self"))
      .def("__str__", &toString, boost::python::args("self"))
      .def("get_translation", &getPoseTranslation, boost::python::args("self"),
           "return the translation part of the transform")
      .def("get_rotation", &getPoseRotation, boost::python::args("self"),
           "return the rotation part of the transform")
      .def("set_translation", &setPoseTranslation,
           boost::python::args("self", "translation"),
           "set the translation as a vector 3d")
      .def("set_rotation", &setPoseRotationQuaternion,
           boost::python::args("self", "quaternion"),
           "set the rotation from a Quaternion")
      .def("set_rotation", &setPoseRotationMatrix,
           boost::python::args("self", "rotation"),
           "set the rotation from a 3d matrix");
}

}  // namespace python
}  // namespace biped_stabilizer
