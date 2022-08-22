
#include <boost/python.hpp>
#include <boost/python/return_internal_reference.hpp>
#include "biped-stabilizer/python.hpp"
#include "biped-stabilizer/cop_stabilizer.hpp"

namespace biped_stabilizer {
namespace python {

namespace bp = boost::python;

template< typename T >
inline
void to_std_vector( const bp::object& iterable, std::vector< T, Eigen::aligned_allocator<T> >& out)
{
    out = std::vector< T , Eigen::aligned_allocator<T> >(
        bp::stl_input_iterator< T >( iterable ),
        bp::stl_input_iterator< T >( ) );
}


bp::tuple getStableCoMs(CopStabilizer& self, double height){

    std::array<eVector3, 3> coms = self.getStableCoMs(height);
    return bp::make_tuple(coms[0], coms[1], coms[2]);
}

bp::tuple CopStabilizer_stabilize(
    CopStabilizer& self, bp::dict args)
{
    eVector3 actual_com = bp::extract<eVector3>(args["actual_com"]);
    eVector3 actual_com_vel = bp::extract<eVector3>(args["actual_com_vel"]);
    eVector3 actual_com_acc = bp::extract<eVector3>(args["actual_com_acc"]);
    eVector3 actual_cop = bp::extract<eVector3>(args["actual_cop"]);
    eVector3 reference_com = bp::extract<eVector3>(args["reference_com"]);
    eVector3 reference_com_vel = bp::extract<eVector3>(args["reference_com_vel"]);
    eVector3 reference_com_acc = bp::extract<eVector3>(args["reference_com_acc"]);
    eVector3 reference_com_jerk = bp::extract<eVector3>(args["reference_com_jerk"]);
    eVector3 desired_com;
    eVector3 desired_com_vel;
    eVector3 desired_com_acc;
    eVector3 desired_icp;
    eVector3 actual_icp;
    eVector3 desired_cop_reference;
    eVector3 desired_cop_computed;

    eMatrixHoms actual_stance_poses;
    to_std_vector(args["actual_stance_poses"], actual_stance_poses);
    self.stabilize(actual_com,
                  actual_com_vel,
                  actual_com_acc,
                  actual_cop,
                  actual_stance_poses,
                  reference_com,
                  reference_com_vel,
                  reference_com_acc,
                  reference_com_jerk,
                  desired_com,
                  desired_com_vel,
                  desired_com_acc,
                  desired_icp,
                  actual_icp,
                  desired_cop_reference,
                  desired_cop_computed);
    return bp::make_tuple(desired_com,
                          desired_com_vel,
                          desired_com_acc,
                          desired_icp,
                          actual_icp,
                          desired_cop_reference,
                          desired_cop_computed);
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
      .def_readwrite("cop_control_type", &CopStabilizerSettings::cop_control_type)
      .def_readwrite("saturate_cop", &CopStabilizerSettings::saturate_cop)
      .def_readwrite("use_rate_limited_dcm", &CopStabilizerSettings::use_rate_limited_dcm)
      .def("__eq__", &CopStabilizerSettings::operator==)
      .def("__ne__", &CopStabilizerSettings::operator!=)
      .def("__repr__", &CopStabilizerSettings::to_string)
      ;

    bp::class_<CopStabilizer, boost::noncopyable>("CopStabilizer", bp::init<>())
      .def(bp::init<CopStabilizerSettings>())
      .def("configure", &CopStabilizer::configure)
      .def("stabilize", &CopStabilizer_stabilize)
      .def("get_stable_coms", &getStableCoMs)
      .def("get_settings", &CopStabilizer::getSettings, bp::return_internal_reference<>())
    ;
  return;


}
}  // namespace python
}  // namespace biped_stabilizer
