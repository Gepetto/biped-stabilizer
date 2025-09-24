#include "biped-stabilizer/cop_stabilizer.hpp"
#include <iostream>

#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>

#include "biped-stabilizer/python.hpp"

namespace biped_stabilizer {
namespace python {

namespace bp = boost::python;

template <typename T>
inline void to_std_vector(const bp::object &iterable,
                          std::vector<T, Eigen::aligned_allocator<T>> &out) {
  out = std::vector<T, Eigen::aligned_allocator<T>>(
      bp::stl_input_iterator<T>(iterable), bp::stl_input_iterator<T>());
}

bp::tuple getStableCoMs(CopStabilizer &self, double height) {
  std::array<eVector3, 3> coms = self.getStableCoMs(height);
  return bp::make_tuple(coms[0], coms[1], coms[2]);
}

bp::tuple CopStabilizer_stabilize(CopStabilizer &self, bp::dict args) {
  eVector3 actual_com = bp::extract<eVector3>(args["actual_com"]);
  eVector3 actual_com_vel = bp::extract<eVector3>(args["actual_com_vel"]);
  eVector3 actual_com_acc = bp::extract<eVector3>(args["actual_com_acc"]);

  bool is_2d = false;
  eVector3 actual_cop_3d;
  eVector2 actual_cop_2d;
  bp::object shape_obj = args["actual_cop"].attr("shape");
  int size_of_cop = bp::extract<int>(shape_obj[0]);
  if (size_of_cop == 2) {
    // Try to extract as eVector2
    actual_cop_2d = bp::extract<eVector2>(args["actual_cop"]);
    is_2d = true;
  } else if (size_of_cop == 3) {
    // Try to extract as eVector3
    actual_cop_3d = bp::extract<eVector3>(args["actual_cop"]);
    is_2d = false;
  } else {
    throw std::runtime_error("actual_cop must be of size 2 or 3");
  }

  eVector3 reference_com = bp::extract<eVector3>(args["reference_com"]);
  eVector3 reference_com_vel = bp::extract<eVector3>(args["reference_com_vel"]);
  eVector3 reference_com_acc = bp::extract<eVector3>(args["reference_com_acc"]);
  eVector3 reference_com_jerk =
      bp::extract<eVector3>(args["reference_com_jerk"]);
  eVector3 desired_com;
  eVector3 desired_com_vel;
  eVector3 desired_com_acc;
  eVector3 desired_icp;
  eVector3 actual_icp;
  eVector3 desired_cop_reference_3d;
  eVector3 desired_cop_computed_3d;
  eVector2 desired_cop_reference_2d;
  eVector2 desired_cop_computed_2d;

  eMatrixHoms actual_stance_poses;
  to_std_vector(args["actual_stance_poses"], actual_stance_poses);
  if (is_2d) {
    self.stabilize(actual_com, actual_com_vel, actual_com_acc, actual_cop_2d,
                   actual_stance_poses, reference_com, reference_com_vel,
                   reference_com_acc, reference_com_jerk, desired_com,
                   desired_com_vel, desired_com_acc, desired_icp, actual_icp,
                   desired_cop_reference_2d, desired_cop_computed_2d);
    return bp::make_tuple(desired_com, desired_com_vel, desired_com_acc,
                          desired_icp, actual_icp, desired_cop_reference_2d,
                          desired_cop_computed_2d);
  } else {
    self.stabilize(actual_com, actual_com_vel, actual_com_acc, actual_cop_3d,
                   actual_stance_poses, reference_com, reference_com_vel,
                   reference_com_acc, reference_com_jerk, desired_com,
                   desired_com_vel, desired_com_acc, desired_icp, actual_icp,
                   desired_cop_reference_3d, desired_cop_computed_3d);
    return bp::make_tuple(desired_com, desired_com_vel, desired_com_acc,
                          desired_icp, actual_icp, desired_cop_reference_3d,
                          desired_cop_computed_3d);
  }
}

void exposeCopStabilizer() {
  bp::class_<CopStabilizerSettings>("CopStabilizerSettings")
      .def_readwrite("height", &CopStabilizerSettings::height)
      .def_readwrite("foot_length", &CopStabilizerSettings::foot_length)
      .def_readwrite("foot_width", &CopStabilizerSettings::foot_width)
      .def_readwrite("robot_mass", &CopStabilizerSettings::robot_mass)
      .def_readwrite("dt", &CopStabilizerSettings::dt)
      .def_readwrite("cop_x_gains", &CopStabilizerSettings::cop_x_gains)
      .def_readwrite("cop_y_gains", &CopStabilizerSettings::cop_y_gains)
      .def_readwrite("cop_p_cc_gain", &CopStabilizerSettings::cop_p_cc_gain)
      .def_readwrite("integral_gain", &CopStabilizerSettings::integral_gain)
      .def_readwrite("g", &CopStabilizerSettings::g)
      .def_readwrite("cop_control_type",
                     &CopStabilizerSettings::cop_control_type)
      .def_readwrite("saturate_cop", &CopStabilizerSettings::saturate_cop)
      .def_readwrite("use_rate_limited_dcm",
                     &CopStabilizerSettings::use_rate_limited_dcm)
      .def("__eq__", &CopStabilizerSettings::operator==)
      .def("__ne__", &CopStabilizerSettings::operator!=)
      .def("__repr__", &CopStabilizerSettings::to_string);

  bp::class_<CopStabilizer, boost::noncopyable>("CopStabilizer", bp::init<>())
      .def(bp::init<CopStabilizerSettings>())
      .def("configure", &CopStabilizer::configure)
      .def("stabilize", &CopStabilizer_stabilize)
      .def("get_stable_coms", &getStableCoMs)
      .def("get_settings", &CopStabilizer::getSettings,
           bp::return_internal_reference<>());
  return;
}
} // namespace python
} // namespace biped_stabilizer
