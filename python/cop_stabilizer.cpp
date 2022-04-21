
#include <boost/python.hpp>
#include "biped_stabilizer/python.hpp"
#include "biped_stabilizer/cop_stabilizer.hpp"

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

void ConfigureStabilizer(CopStabilizer& self, bp::dict settings)
{
  double height = bp::extract<double>(settings["height"]); // 0.0
  double foot_length = bp::extract<double>(settings["foot_length"]); //0.0;
  double foot_width = bp::extract<double>(settings["foot_width"]); //0.0;
  double robot_mass = bp::extract<double>(settings["robot_mass"]); //0.0;
  double dt = bp::extract<double>(settings["dt"]); //0.0;
  eVector3 cop_x_gains = bp::extract<eVector3>(settings["cop_x_gains"]); //eVector3::Zero();
  eVector3 cop_y_gains = bp::extract<eVector3>(settings["cop_y_gains"]); //eVector3::Zero();
  double cop_p_cc_gain = bp::extract<double>(settings["cop_p_cc_gain"]); //0.0;
  eVector2 integral_gain = bp::extract<eVector2>(settings["integral_gain"]); //eVector2::Zero();
  // Meaningfull defaults.
  double g = bp::extract<double>(settings["g"]); //9.81;
  std::string cop_control_type = bp::extract<std::string>(settings["cop_control_type"]); //"p_cc";
  bool saturate_cop = bp::extract<bool>(settings["saturate_cop"]); //true;
  bool use_rate_limited_dcm = bp::extract<bool>(settings["use_rate_limited_dcm"]); //false;

  CopStabilizerSettings setting;
  setting.height = height;
  setting.foot_width = foot_width;
  setting.foot_length = foot_length;
  setting.robot_mass = robot_mass;
  setting.dt = dt;
  setting.cop_x_gains = cop_x_gains;
  setting.cop_y_gains = cop_y_gains;
  setting.cop_p_cc_gain = cop_p_cc_gain;
  setting.integral_gain = integral_gain;
  setting.g = g;
  setting.cop_control_type = cop_control_type;
  setting.saturate_cop = saturate_cop;
  setting.use_rate_limited_dcm = use_rate_limited_dcm;

  self.configure(setting);
}

CopStabilizerSettings settings(CopStabilizer& self){ return self.get_settings();}

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
    eVector3 desired_com = bp::extract<eVector3>(args["desired_com"]);
    eVector3 desired_com_vel = bp::extract<eVector3>(args["desired_com_vel"]);
    eVector3 desired_com_acc = bp::extract<eVector3>(args["desired_com_acc"]);
    eVector3 desired_icp = bp::extract<eVector3>(args["desired_icp"]);
    eVector3 actual_icp = bp::extract<eVector3>(args["actual_icp"]);
    eVector3 desired_cop_reference = bp::extract<eVector3>(args["desired_cop_reference"]);
    eVector3 desired_cop_computed = bp::extract<eVector3>(args["desired_cop_computed"]);
    
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
      .add_property("height", &CopStabilizerSettings::height)
      .add_property("foot_length", &CopStabilizerSettings::foot_length)
      .add_property("foot_width", &CopStabilizerSettings::foot_width)
      .add_property("robot_mass", &CopStabilizerSettings::robot_mass)
      .add_property("dt", &CopStabilizerSettings::dt)
      .add_property("cop_x_gains", &CopStabilizerSettings::cop_x_gains)
      .add_property("cop_y_gains", &CopStabilizerSettings::cop_y_gains)
      .add_property("cop_p_cc_gain", &CopStabilizerSettings::cop_p_cc_gain)
      .add_property("integral_gain", &CopStabilizerSettings::integral_gain)
      .add_property("g", &CopStabilizerSettings::g)
      .add_property("cop_control_type", &CopStabilizerSettings::cop_control_type)
      .add_property("saturate_cop", &CopStabilizerSettings::saturate_cop)
      .add_property("use_rate_limited_dcm", &CopStabilizerSettings::use_rate_limited_dcm)
      ;

    bp::class_<CopStabilizer, boost::noncopyable>("CopStabilizer", bp::init<>())
      .def(bp::init<CopStabilizerSettings>())
      .def("stabilize", &CopStabilizer_stabilize)
      .def("get_stable_coms", &CopStabilizer::getStableCoMs)
      .def("configure", &ConfigureStabilizer)
      .add_property("settings", &settings)
    ;
  return;


}
}  // namespace python
}  // namespace biped_stabilizer