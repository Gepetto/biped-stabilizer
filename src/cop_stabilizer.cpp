#include "biped-stabilizer/cop_stabilizer.hpp"

#include "biped-stabilizer/third_party/wykobi/wykobi_algorithm.hpp"

namespace biped_stabilizer {

const double DT_DERIVATIVE = 1e-4;

CopStabilizer::CopStabilizer()
    : configured_(false), old_reference_com_acc_(eVector3::Zero()),
      jerk_ma_queue_(), errorSum_(Eigen::Vector2d::Zero()) {}

CopStabilizer::CopStabilizer(const CopStabilizerSettings &settings)
    : configured_(false), old_reference_com_acc_(eVector3::Zero()),
      jerk_ma_queue_(), errorSum_(Eigen::Vector2d::Zero()) {
  configure(settings);
}

CopStabilizer::~CopStabilizer() {}

void CopStabilizer::configure(const CopStabilizerSettings &settings) {
  settings_ = settings;
  const double &dt = settings_.dt;
  configured_ = true;
  first_iter_ = true;
  w2_ = settings_.g / settings_.height;
  w_ = std::sqrt(w2_);
  RotPi_2_ << 0, -1, 1, 0;
  // These system matrices correspond to "dP->CCP" state: c, c_dot, ccop and
  // input ccop_dot
  A_ << cosh(w_ * dt), sinh(w_ * dt) / w_, 1 - cosh(w_ * dt),
      w_ * sinh(w_ * dt), cosh(w_ * dt), -w_ * sinh(w_ * dt), 0, 0, 1;
  B_ << dt - sinh(w_ * dt) / w_, 1 - cosh(w_ * dt), dt;

  cx_gainK_ = settings_.cop_x_gains;
  cy_gainK_ = settings_.cop_y_gains;

  A22_ << cosh(w_ * dt), sinh(w_ * dt) / w_, w_ * sinh(w_ * dt), cosh(w_ * dt);
  B2_ << 1 - cosh(w_ * dt), -w_ * sinh(w_ * dt);

  Aj_ << 1, dt, dt * dt / 2, 0, 1, dt, 0, 0, 1;
  Bj_ << dt * dt * dt / 6, dt * dt / 2, dt;

  //  cx_gainK2_ << 3.36, 3.36/w;
  //  cy_gainK2_ << 3.36, 3.36/w;
  cx_gainK2_ << settings_.cop_p_cc_gain, settings_.cop_p_cc_gain / w_;
  cy_gainK2_ << settings_.cop_p_cc_gain, settings_.cop_p_cc_gain / w_;

  // values for J_CCC method
  S_coms_j_ << 1, dt - DT_DERIVATIVE, pow(dt - DT_DERIVATIVE, 2) / 2, 1, dt,
      pow(dt, 2) / 2, 1, dt + DT_DERIVATIVE, pow(dt + DT_DERIVATIVE, 2) / 2;
  U_coms_j_ << pow(dt - DT_DERIVATIVE, 3) / 6, pow(dt, 3) / 6,
      pow(dt + DT_DERIVATIVE, 3) / 6;

  // Values for P_CC method
  S_coms_ << cosh(w_ * (dt - DT_DERIVATIVE)),
      sinh(w_ * (dt - DT_DERIVATIVE)) / w_, cosh(w_ * dt), sinh(w_ * dt) / w_,
      cosh(w_ * (dt + DT_DERIVATIVE)), sinh(w_ * (dt + DT_DERIVATIVE)) / w_;
  U_coms_ << 1 - cosh(w_ * (dt - DT_DERIVATIVE)), 1 - cosh(w_ * dt),
      1 - cosh(w_ * (dt + DT_DERIVATIVE));

  target_com_.setZero();
  target_com_vel_.setZero();
  target_com_acc_.setZero();
  non_linear_.setZero();
  target_cop_.setZero();
  desired_uncampled_cop_.setZero();
  errorSum_.setZero();
  oldTrackingError2_x_.setZero();
  oldTrackingError2_y_.setZero();
  oldTrackingError_x_.setZero();
  oldTrackingError_y_.setZero();
  estimated_disturbance_.setZero();

  local_foot_description_.resize(4);
  local_foot_description_[0] =
      eVector3(settings_.foot_length / 2, settings_.foot_width / 2, 0.);
  local_foot_description_[1] =
      eVector3(settings_.foot_length / 2, -settings_.foot_width / 2, 0.);
  local_foot_description_[2] =
      eVector3(-settings_.foot_length / 2, settings_.foot_width / 2, 0.);
  local_foot_description_[3] =
      eVector3(-settings_.foot_length / 2, -settings_.foot_width / 2, 0.);
  foot_description_.clear();
  foot_description_.reserve(10 * 4);
  wykobi_foot_description_.clear();
  wykobi_foot_description_.reserve(10 * 4);
  jerk_ma_queue_.clear();

  // REGISTER_VARIABLE("/introspection_data", "StabilizerTargetCOMPosition_X",
  //                   &target_com_[0], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerTargetCOMPosition_Y",
  //                   &target_com_[1], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerTargetCOMPosition_Z",
  //                   &target_com_[2], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerTargetCOMVelocity_X",
  //                   &target_com_vel_[0], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerTargetCOMVelocity_Y",
  //                   &target_com_vel_[1], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerTargetCOMVelocity_Z",
  //                   &target_com_vel_[2], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data",
  // "StabilizerTargetCOMAcceleration_X",
  //                   &target_com_acc_[0], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data",
  // "StabilizerTargetCOMAcceleration_Y",
  //                   &target_com_acc_[1], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data",
  // "StabilizerTargetCOMAcceleration_Z",
  //                   &target_com_acc_[2], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerNonLinearPart_X",
  //                   &non_linear_[0], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerNonLinearPart_Y",
  //                   &non_linear_[1], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerNonLinearPart_Z",
  //                   &non_linear_[2], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerTargetCOP_X",
  //                   &target_cop_[0], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerTargetCOP_Y",
  //                   &target_cop_[1], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerDesiredUnclampedCOP_X",
  //                   &desired_uncampled_cop_[0], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerDesiredUnclampedCOP_Y",
  //                   &desired_uncampled_cop_[1], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerErrorSum_X",
  //                   &errorSum_[0], &registered_variables_);
  // REGISTER_VARIABLE("/introspection_data", "StabilizerErrorSum_Y",
  //                   &errorSum_[1], &registered_variables_);
}

void CopStabilizer::computeSupportPolygon(const eMatrixHoms &stance_poses,
                                          Polygon2D &convex_hull) {
  // Compute the global position of the feet edges.
  foot_description_.clear();
  for (size_t j = 0; j < stance_poses.size(); j++) {
    for (size_t i = 0; i < local_foot_description_.size(); ++i) {
      foot_description_.push_back(stance_poses[j] * local_foot_description_[i]);
    }
  }
  // Convert it to the wykobi type.
  wykobi_foot_description_.clear();
  for (size_t i = 0; i < foot_description_.size(); ++i) {
    wykobi_foot_description_.push_back(
        wykobi::make_point(foot_description_[i](0), foot_description_[i](1)));
  }
  // Compute the convex hull.
  convex_hull.clear();
  wykobi::algorithm::convex_hull_graham_scan<wykobi::point2d<double>>(
      wykobi_foot_description_.begin(), wykobi_foot_description_.end(),
      std::back_inserter(convex_hull));
}

void CopStabilizer::projectCOPinSupportPolygon(
    const eVector2 &target_cop_unclamped, const Polygon2D &polygon,
    eVector2 &target_cop) {
  wykobi_cop_unclamped_.x = target_cop_unclamped.x();
  wykobi_cop_unclamped_.y = target_cop_unclamped.y();
  if (wykobi::point_in_convex_polygon(wykobi_cop_unclamped_, polygon)) {
    target_cop = target_cop_unclamped;
  } else {
    // ROS_INFO("[com_control_utils] COP_unclamped not in the support polygon
    // !");
    wykobi_cop_clamped_ = wykobi::closest_point_on_polygon_from_point(
        polygon, wykobi_cop_unclamped_);
    target_cop.x() = wykobi_cop_clamped_.x;
    target_cop.y() = wykobi_cop_clamped_.y;
  }
}

void CopStabilizer::stabilize(
    const eVector3 &actual_com, const eVector3 &actual_com_vel,
    const eVector3 &actual_com_acc, const eVector3 &actual_cop,
    const eMatrixHoms &actual_stance_poses, const eVector3 &reference_com,
    const eVector3 &reference_com_vel, const eVector3 &reference_com_acc,
    const eVector3 &reference_com_jerk, eVector3 &desired_com,
    eVector3 &desired_com_vel, eVector3 &desired_com_acc,
    eVector3 &desired_icp,           // ???
    eVector3 &actual_icp,            // ???
    eVector3 &desired_cop_reference, // ???
    eVector3 &desired_cop_computed) {
  Polygon2D support_polygon;
  if (settings_.saturate_cop) {
    computeSupportPolygon(actual_stance_poses, support_polygon);
  } // else skip computation to save computation time
  return stabilize(actual_com, actual_com_vel, actual_com_acc, actual_cop,
                   support_polygon, reference_com, reference_com_vel,
                   reference_com_acc, reference_com_jerk, desired_com,
                   desired_com_vel, desired_com_acc, desired_icp, actual_icp,
                   desired_cop_reference, desired_cop_computed);
}

void CopStabilizer::stabilize(
    const eVector3 &actual_com, const eVector3 &actual_com_vel,
    const eVector3 &actual_com_acc, const eVector3 &actual_cop,
    const Polygon2D &support_polygon, const eVector3 &reference_com,
    const eVector3 &reference_com_vel, const eVector3 &reference_com_acc,
    const eVector3 &reference_com_jerk, eVector3 &desired_com,
    eVector3 &desired_com_vel, eVector3 &desired_com_acc,
    eVector3 &desired_icp,           // ???
    eVector3 &actual_icp,            // ???
    eVector3 &desired_cop_reference, // ???
    eVector3 &desired_cop_computed) {
  if (settings_.cop_control_type == "cop")
    return stabilizeCOP(actual_com, actual_com_vel, actual_com_acc, actual_cop,
                        support_polygon, reference_com, reference_com_vel,
                        reference_com_acc, desired_com, desired_com_vel,
                        desired_com_acc, desired_icp, actual_icp,
                        desired_cop_reference, desired_cop_computed);
  else if (settings_.cop_control_type == "p_cc")
    return stabilizeP_CC(actual_com, actual_com_vel, actual_com_acc, actual_cop,
                         support_polygon, reference_com, reference_com_vel,
                         reference_com_acc, desired_com, desired_com_vel,
                         desired_com_acc, desired_icp, actual_icp,
                         desired_cop_reference, desired_cop_computed);
  else if (settings_.cop_control_type == "approximated_acceleration")
    return stabilizeApproximateAcceleration(
        actual_com, actual_com_vel, actual_com_acc, actual_cop, support_polygon,
        reference_com, reference_com_vel, reference_com_acc, desired_com,
        desired_com_vel, desired_com_acc, desired_icp, actual_icp,
        desired_cop_reference, desired_cop_computed);
  else if (settings_.cop_control_type == "j_ccc")
    return stabilizeJerk(
        actual_com, actual_com_vel, actual_com_acc, actual_cop, support_polygon,
        reference_com, reference_com_vel, reference_com_acc, reference_com_jerk,
        desired_com, desired_com_vel, desired_com_acc, desired_icp, actual_icp,
        desired_cop_reference, desired_cop_computed);
  else
    throw std::runtime_error("Invalid cop control type, got : " +
                             settings_.cop_control_type);
}

void CopStabilizer::stabilizeCOP( // Not supported
    const eVector3 &actual_com, const eVector3 &actual_com_vel,
    const eVector3 &actual_com_acc, const eVector3 &actual_cop,
    const Polygon2D &support_polygon, const eVector3 &reference_com,
    const eVector3 &reference_com_vel, const eVector3 &reference_com_acc,
    eVector3 &desired_com, eVector3 &desired_com_vel, eVector3 &desired_com_acc,
    eVector3 &desired_icp,           // ???
    eVector3 &actual_icp,            // ???
    eVector3 &desired_cop_reference, // ???
    eVector3 &desired_cop_computed) {
  // ROS_INFO("Call stabilizeCOP");
  if (!configured_) {
    throw std::runtime_error(
        "Stabilizer not initialized, please call the constructor with "
        "arguments first");
  }
  const double &dt = settings_.dt;

  target_com_ = reference_com;
  target_com_vel_ = reference_com_vel;
  target_com_acc_ = reference_com_acc;

  // REAL values
  const Eigen::Vector3d actualState_x(actual_com.x(), actual_com_vel.x(),
                                      actual_cop.x());
  const Eigen::Vector3d actualState_y(actual_com.y(), actual_com_vel.y(),
                                      actual_cop.y());

  // Reference values
  const Eigen::Vector3d reference_com_jerky(
      (reference_com_acc - old_reference_com_acc_) / dt);
  const Eigen::Vector3d reference_com_jerk(
      movingAverage(reference_com_jerky, 5, jerk_ma_queue_));
  const Eigen::Vector2d referenceCCOP(reference_com.head<2>() -
                                      reference_com_acc.head<2>() / w2_);
  const Eigen::Vector2d referenceCCOP_vel(reference_com_vel.head<2>() -
                                          reference_com_jerk.head<2>() / w2_);

  const Eigen::Vector3d referenceState_x(
      reference_com.x(), reference_com_vel.x(), referenceCCOP.x());
  const Eigen::Vector3d referenceState_y(
      reference_com.y(), reference_com_vel.y(), referenceCCOP.y());

  // Tracking
  Eigen::Vector3d stateTrackingError_x(actualState_x - referenceState_x);
  Eigen::Vector3d stateTrackingError_y(actualState_y - referenceState_y);
  Eigen::Vector2d feedbackTerm;
  feedbackTerm << cx_gainK_.transpose() * stateTrackingError_x,
      cy_gainK_.transpose() * stateTrackingError_y;

  if (settings_.use_rate_limited_dcm) {
    /// @todo create a low pass filter on the DCM
    // feedbackTerm = rate_limiter->onlineFiltering(feedbackTerm);
  }
  Eigen::Vector2d commandedCCOP_vel(referenceCCOP_vel + feedbackTerm);
  Eigen::Vector3d nextState_x;
  Eigen::Vector3d nextState_y;

  nextState_x = A_ * actualState_x + B_ * commandedCCOP_vel.x();
  nextState_y = A_ * actualState_y + B_ * commandedCCOP_vel.y();

  getNonLinearPart(actual_com, actual_com_acc, actual_cop, non_linear_);
  if (settings_.saturate_cop) {
    Eigen::Vector2d COP_unclamped(nextState_x(2) + non_linear_(0),
                                  nextState_y(2) + non_linear_(1));

    if (!isPointInPolygon(COP_unclamped, support_polygon)) {
      eVector2 COP_clamped;
      projectCOPinSupportPolygon(COP_unclamped, support_polygon, COP_clamped);

      eVector3 nextRefState_x(A_ * referenceState_x +
                              B_ * referenceCCOP_vel.x());
      eVector3 nextRefState_y(A_ * referenceState_y +
                              B_ * referenceCCOP_vel.y());
      eVector2 nextRefCCOP(nextRefState_x(2), nextRefState_y(2));
      eVector2 LA_trErr_0(A_.block<1, 3>(2, 0) * stateTrackingError_x,
                          A_.block<1, 3>(2, 0) * stateTrackingError_y);

      eVector2 r =
          COP_clamped - nextRefCCOP - non_linear_.head<2>() - LA_trErr_0;

      eVector3 stateTrackingError_clamped_x =
          cx_gainK_ * r.x() / (cx_gainK_.dot(cx_gainK_) * B_(2));
      eVector3 stateTrackingError_clamped_y =
          cy_gainK_ * r.y() / (cy_gainK_.dot(cy_gainK_) * B_(2));

      eVector3 nextStateTrackingError_clamped_x =
          A_ * stateTrackingError_x +
          B_ * cx_gainK_.transpose() * stateTrackingError_clamped_x;
      eVector3 nextStateTrackingError_clamped_y =
          A_ * stateTrackingError_y +
          B_ * cy_gainK_.transpose() * stateTrackingError_clamped_y;

      nextState_x = nextRefState_x + nextStateTrackingError_clamped_x;
      nextState_y = nextRefState_y + nextStateTrackingError_clamped_y;
    }
  }

  desired_com << nextState_x(0), nextState_y(0), reference_com.z();
  desired_com_vel << nextState_x(1), nextState_y(1), reference_com_vel.z();
  desired_com_acc << w2_ * (nextState_x(0) - nextState_x(2)),
      w2_ * (nextState_y(0) - nextState_y(2)), reference_com_acc.z();
  desired_icp << nextState_x(0) + nextState_x(1) / w_,
      nextState_y(0) + nextState_y(1) / w_, 0.;
  actual_icp << actualState_x(0) + actualState_x(1) / w_,
      actualState_y(0) + actualState_y(1) / w_, 0.;
  desired_cop_reference << referenceState_x(2), referenceState_y(2), 0.;
  desired_cop_computed << nextState_x(2), nextState_y(2), 0.;

  // updating old_acc
  old_reference_com_acc_ = reference_com_acc;
}

void CopStabilizer::stabilizeApproximateAcceleration( // Not supported
    const eVector3 &actual_com, const eVector3 &actual_com_vel,
    const eVector3 &actual_com_acc, const eVector3 &actual_cop,
    const Polygon2D &support_polygon, const eVector3 &reference_com,
    const eVector3 &reference_com_vel, const eVector3 &reference_com_acc,
    eVector3 &desired_com, eVector3 &desired_com_vel, eVector3 &desired_com_acc,
    eVector3 &desired_icp,           // ???
    eVector3 &actual_icp,            // ???
    eVector3 &desired_cop_reference, // ???
    eVector3 &desired_cop_computed) {
  // ROS_INFO("Call stabilize approx acceleration");
  if (!configured_) {
    throw std::runtime_error(
        "Stabilizer not initialized, please call the constructor with "
        "arguments first");
  }
  const double &dt = settings_.dt;

  target_com_ = reference_com;
  target_com_vel_ = reference_com_vel;
  target_com_acc_ = reference_com_acc;

  // REFERENCE
  static Eigen::Vector3d old_reference_com_acc = reference_com_acc;
  Eigen::Vector3d reference_com_jerky(
      (reference_com_acc - old_reference_com_acc) / dt);
  Eigen::Vector3d reference_com_jerk(
      movingAverage(reference_com_jerky, 5, jerk_ma_queue_));
  Eigen::Vector2d referenceCCOP(reference_com.head<2>() -
                                reference_com_acc.head<2>() / w2_);
  Eigen::Vector2d referenceCCOP_vel(reference_com_vel.head<2>() -
                                    reference_com_jerk.head<2>() / w2_);
  Eigen::Vector3d referenceState_x(reference_com.x(), reference_com_vel.x(),
                                   referenceCCOP.x());
  Eigen::Vector3d referenceState_y(reference_com.y(), reference_com_vel.y(),
                                   referenceCCOP.y());

  // REAL
  Eigen::Vector3d actualState_x(actual_com.x(), actual_com_vel.x(),
                                actual_cop.x());
  Eigen::Vector3d actualState_y(actual_com.y(), actual_com_vel.y(),
                                actual_cop.y());

  // TRACKING
  Eigen::Vector3d stateTrackingError_x(actualState_x - referenceState_x);
  Eigen::Vector3d stateTrackingError_y(actualState_y - referenceState_y);
  Eigen::Vector2d feedbackTerm;
  feedbackTerm << cx_gainK_.transpose() * stateTrackingError_x,
      cy_gainK_.transpose() * stateTrackingError_y;

  if (settings_.use_rate_limited_dcm) {
    /// @todo create a low pass filter on the DCM
    // feedbackTerm = rate_limiter->onlineFiltering(feedbackTerm);
  }

  if (!settings_.integral_gain.isZero()) {
    static Eigen::Vector2d errorSum(0, 0);
    errorSum += actual_cop.head<2>() - referenceCCOP;

    Eigen::Vector2d integral_signal(
        (settings_.integral_gain.array() * errorSum.array()).matrix());
    feedbackTerm += integral_signal;
  }
  Eigen::Vector2d commandedCCOP_vel(referenceCCOP_vel + feedbackTerm);
  Eigen::Vector3d nextState_x;
  Eigen::Vector3d nextState_y;

  nextState_x = A_ * actualState_x + B_ * commandedCCOP_vel.x();
  nextState_y = A_ * actualState_y + B_ * commandedCCOP_vel.y();

  if (settings_.saturate_cop) {
    getNonLinearPart(actual_com, actual_com_acc, actual_cop, non_linear_);
    Eigen::Vector2d COP_unclamped(nextState_x(2) + non_linear_(0),
                                  nextState_y(2) + non_linear_(1));

    if (!isPointInPolygon(COP_unclamped, support_polygon)) {
      // ROS_INFO("[com_control_utils] COP_unclamped not in the support polygon
      // !");

      eVector2 COP_clamped;
      projectCOPinSupportPolygon(COP_unclamped, support_polygon, COP_clamped);

      eVector3 nextRefState_x(A_ * referenceState_x +
                              B_ * referenceCCOP_vel.x());
      eVector3 nextRefState_y(A_ * referenceState_y +
                              B_ * referenceCCOP_vel.y());
      eVector2 nextRefCCOP(nextRefState_x(2), nextRefState_y(2));
      eVector2 LA_trErr_0(A_.block<1, 3>(2, 0) * stateTrackingError_x,
                          A_.block<1, 3>(2, 0) * stateTrackingError_y);

      eVector2 saturatedFeedbackTerm =
          (COP_clamped - nextRefCCOP - non_linear_.head<2>() - LA_trErr_0) /
          B_(2);
      commandedCCOP_vel = referenceCCOP_vel + saturatedFeedbackTerm;

      nextState_x = A_ * actualState_x + B_ * commandedCCOP_vel.x();
      nextState_y = A_ * actualState_y + B_ * commandedCCOP_vel.y();
    }
  }

  desired_com << nextState_x(0), nextState_y(0), reference_com.z();
  desired_com_vel << nextState_x(1), nextState_y(1), reference_com_vel.z();
  desired_com_acc << w2_ * (nextState_x(0) - nextState_x(2)),
      w2_ * (nextState_y(0) - nextState_y(2)), reference_com_acc.z();
  desired_icp << nextState_x(0) + nextState_x(1) / w_,
      nextState_y(0) + nextState_y(1) / w_, 0.;
  actual_icp << actualState_x(0) + actualState_x(1) / w_,
      actualState_y(0) + actualState_y(1) / w_, 0.;
  desired_cop_reference << referenceState_x(2), referenceState_y(2), 0.;
  desired_cop_computed << nextState_x(2), nextState_y(2), 0.;

  // updating old_acc
  old_reference_com_acc = reference_com_acc;
}

void CopStabilizer::stabilizeP_CC(
    const eVector3 &actual_com, const eVector3 &actual_com_vel,
    const eVector3 &actual_com_acc, const eVector3 &actual_cop,
    const Polygon2D &support_polygon, const eVector3 &reference_com,
    const eVector3 &reference_com_vel, const eVector3 &reference_com_acc,
    eVector3 &desired_com, eVector3 &desired_com_vel, eVector3 &desired_com_acc,
    eVector3 &desired_icp,           // ???
    eVector3 &actual_icp,            // ???
    eVector3 &desired_cop_reference, // ???
    eVector3 &desired_cop_computed) {
  // ROS_INFO("Call stabilizeCCOP");
  if (!configured_) {
    throw std::runtime_error(
        "Stabilizer not initialized, please call the constructor with "
        "arguments first");
  }

  target_com_ = reference_com;
  target_com_vel_ = reference_com_vel;
  target_com_acc_ = reference_com_acc;

  if (first_iter_) {
    first_iter_ = false;
  }
  // REFERENCE
  const Eigen::Vector2d referenceCCOP(reference_com.head<2>() -
                                      reference_com_acc.head<2>() / w2_);
  target_cop_ = referenceCCOP;
  const Eigen::Vector2d referenceState_x(reference_com.x(),
                                         reference_com_vel.x());
  const Eigen::Vector2d referenceState_y(reference_com.y(),
                                         reference_com_vel.y());

  // REAL
  actualState2d_x_ = eVector2(actual_com.x(), actual_com_vel.x());
  actualState2d_y_ = eVector2(actual_com.y(), actual_com_vel.y());

  // TRACKING
  const Eigen::Vector2d stateTrackingError_x(actualState2d_x_ -
                                             referenceState_x);
  const Eigen::Vector2d stateTrackingError_y(actualState2d_y_ -
                                             referenceState_y);

  estimated_disturbance_[0] = estimateCopDisturbance(
      stateTrackingError_x, oldTrackingError2_x_, cx_gainK2_);
  estimated_disturbance_[1] = estimateCopDisturbance(
      stateTrackingError_y, oldTrackingError2_y_, cy_gainK2_);

  Eigen::Vector2d feedbackTerm;
  feedbackTerm << cx_gainK2_.transpose() * stateTrackingError_x,
      cy_gainK2_.transpose() * stateTrackingError_y;

  if (settings_.use_rate_limited_dcm) {
    /// @todo create a low pass filter on the DCM (actually not in the DCM, it
    /// should be on the feedback signal)
    // feedbackTerm = rate_limiter->onlineFiltering(feedbackTerm);
  }

  if (!settings_.integral_gain.isZero()) {
    // actual_com_acc_ = bc_->getComputedCOMAcceleration().head<2>();
    // actual_computed_cop_ = actual_com_2d - actual_com_acc_/pow(w, 2);

    errorSum_ += actual_cop.head<2>() - referenceCCOP;

    Eigen::Vector2d integral_signal(
        (settings_.integral_gain.array() * errorSum_.array()).matrix());
    feedbackTerm += integral_signal;
  }
  actual_command_ = (referenceCCOP + feedbackTerm);

  getNonLinearPart(actual_com, actual_com_acc, actual_cop, non_linear_);
  desired_uncampled_cop_ = actual_command_ + non_linear_.head<2>();

  if (settings_.saturate_cop) {
    if (!isPointInPolygon(desired_uncampled_cop_, support_polygon)) {
      // ROS_INFO("[com_control_utils] COP_unclamped not in the support polygon
      // !");
      eVector2 COP_clamped;
      projectCOPinSupportPolygon(desired_uncampled_cop_, support_polygon,
                                 COP_clamped);
      actual_command_ = COP_clamped - non_linear_.head<2>();
    }
  }

  Eigen::Vector2d nextState_x;
  Eigen::Vector2d nextState_y;
  nextState_x = A22_ * actualState2d_x_ + B2_ * actual_command_.x();
  nextState_y = A22_ * actualState2d_y_ + B2_ * actual_command_.y();

  desired_com << nextState_x(0), nextState_y(0), reference_com.z();
  desired_com_vel << nextState_x(1), nextState_y(1), reference_com_vel.z();
  desired_com_acc << w2_ * (nextState_x(0) - actual_command_.x()),
      w2_ * (nextState_y(0) - actual_command_.y()), reference_com_acc.z();
  desired_icp << nextState_x(0) + nextState_x(1) / w_,
      nextState_y(0) + nextState_y(1) / w_, 0.;
  actual_icp << actualState2d_x_(0) + actualState2d_x_(1) / w_,
      actualState2d_y_(0) + actualState2d_y_(1) / w_, 0.;
  desired_cop_reference << referenceCCOP + non_linear_.head<2>(), 0.;
  desired_cop_computed << actual_command_ + non_linear_.head<2>(), 0.;
}

void CopStabilizer::stabilizeJerk(
    const eVector3 &actual_com, const eVector3 &actual_com_vel,
    const eVector3 &actual_com_acc, const eVector3 &actual_cop,
    const Polygon2D &support_polygon, const eVector3 &reference_com,
    const eVector3 &reference_com_vel, const eVector3 &reference_com_acc,
    const eVector3 &reference_com_jerk, eVector3 &desired_com,
    eVector3 &desired_com_vel, eVector3 &desired_com_acc,
    eVector3 &desired_icp,           // ???
    eVector3 &actual_icp,            // ???
    eVector3 &desired_cop_reference, // ???
    eVector3 &desired_cop_computed) {
  target_com_ = reference_com;
  target_com_vel_ = reference_com_vel;
  target_com_acc_ = reference_com_acc;
  target_com_jerk_ = reference_com_jerk;

  // REFERENCE
  const eVector3 referenceState_x(reference_com.x(), reference_com_vel.x(),
                                  reference_com_acc.x());
  const eVector3 referenceState_y(reference_com.y(), reference_com_vel.y(),
                                  reference_com_acc.y());
  const eVector2 referenceCCOP(reference_com.head<2>() -
                               reference_com_acc.head<2>() / (w2_));
  target_cop_ = referenceCCOP;

  // REAL
  actualState3d_x_ =
      eVector3(actual_com.x(), actual_com_vel.x(), actual_com_acc.x());
  actualState3d_y_ =
      eVector3(actual_com.y(), actual_com_vel.y(), actual_com_acc.y());

  // TRACKING
  const Eigen::Vector3d stateTrackingError_x(actualState3d_x_ -
                                             referenceState_x);
  const Eigen::Vector3d stateTrackingError_y(actualState3d_y_ -
                                             referenceState_y);

  estimated_disturbance_[0] = estimateJerkDisturbance(
      stateTrackingError_x, oldTrackingError_x_, cx_gainK_);
  estimated_disturbance_[1] = estimateJerkDisturbance(
      stateTrackingError_y, oldTrackingError_y_, cy_gainK_);

  Eigen::Vector2d feedbackTerm;
  feedbackTerm << cx_gainK_.transpose() * stateTrackingError_x,
      cy_gainK_.transpose() * stateTrackingError_y;

  if (settings_.use_rate_limited_dcm) {
    /// @todo create a low pass filter on the DCM
    // feedbackTerm = rate_limiter->onlineFiltering(feedbackTerm);
  }

  if (!settings_.integral_gain.isZero()) {
    errorSum_ += actual_cop.head<2>() - referenceCCOP;

    Eigen::Vector2d integral_signal(
        (settings_.integral_gain.array() * errorSum_.array()).matrix());
    feedbackTerm += integral_signal;
  }
  actual_command_ = eVector2(reference_com_jerk.head<2>() + feedbackTerm);
  Eigen::Vector3d nextState_x(Aj_ * actualState3d_x_ +
                              Bj_ * actual_command_.x());
  Eigen::Vector3d nextState_y(Aj_ * actualState3d_y_ +
                              Bj_ * actual_command_.y());

  getNonLinearPart(actual_com, actual_com_acc, actual_cop, non_linear_);
  desired_uncampled_cop_ =
      eVector2(nextState_x(0) - nextState_x(2) / w2_ + non_linear_(0),
               nextState_y(0) - nextState_y(2) / w2_ + non_linear_(1));
  if (settings_.saturate_cop) {
    if (!isPointInPolygon(desired_uncampled_cop_, support_polygon)) {
      // ROS_INFO("[com_control_utils] COP_unclamped not in the support polygon
      // !");

      eVector2 COP_clamped;
      projectCOPinSupportPolygon(desired_uncampled_cop_, support_polygon,
                                 COP_clamped);

      const eVector3 nextRefState_x(Aj_ * referenceState_x +
                                    Bj_ * reference_com_jerk.x());
      const eVector3 nextRefState_y(Aj_ * referenceState_y +
                                    Bj_ * reference_com_jerk.y());
      const eVector2 nextRefCCOP(nextRefState_x(0) - nextRefState_x(2) / w2_,
                                 nextRefState_y(0) - nextRefState_y(2) / w2_);
      eVector3 L;
      L << 1, 0, -1 / w2_;
      const eVector2 LA_trErr_0(L.transpose() * Aj_ * stateTrackingError_x,
                                L.transpose() * Aj_ * stateTrackingError_y);

      const eVector2 saturatedFeedbackTerm =
          (COP_clamped - nextRefCCOP - non_linear_.head<2>() - LA_trErr_0) /
          (L.transpose() * Bj_);
      actual_command_ = reference_com_jerk.head<2>() + saturatedFeedbackTerm;

      nextState_x = Aj_ * actualState3d_x_ + Bj_ * actual_command_.x();
      nextState_y = Aj_ * actualState3d_y_ + Bj_ * actual_command_.y();
    }
  }

  desired_com << nextState_x(0), nextState_y(0), reference_com.z();
  desired_com_vel << nextState_x(1), nextState_y(1), reference_com_vel.z();
  desired_com_acc << nextState_x(2), nextState_y(2), reference_com_acc.z();
  desired_icp << nextState_x(0) + nextState_x(1) / w_,
      nextState_y(0) + nextState_y(1) / w_, 0.;
  actual_icp << actualState3d_x_(0) + actualState3d_x_(1) / w_,
      actualState3d_y_(0) + actualState3d_y_(1) / w_, 0.;
  desired_cop_reference << referenceCCOP.x() + non_linear_.x(),
      referenceCCOP.y() + non_linear_.y(), 0.;
  desired_cop_computed << nextState_x(0) - nextState_x(2) / w2_ +
                              non_linear_.x(),
      nextState_y(0) - nextState_y(2) / w2_ + non_linear_.y(), 0.;
}

double CopStabilizer::distributeForces(
    const eVector2 &desired_cop, const eVector2 LF_xy, const double LF_force_z,
    const eVector2 LF_torque_xy, const eVector2 RF_xy, const double RF_force_z,
    const eVector2 RF_torque_xy) {
  Eigen::Vector2d cop_L(LF_xy + RotPi_2_ * LF_torque_xy / LF_force_z);
  Eigen::Vector2d cop_R(RF_xy + RotPi_2_ * RF_torque_xy / RF_force_z);

  return (desired_cop - cop_R).norm() / (cop_L - cop_R).norm();
}

std::array<eVector3, 3> CopStabilizer::getStableCoMs(const double &com_height) {
  if (settings_.cop_control_type == "p_cc") {
    return {{eVector3(S_coms_.block<1, 2>(0, 0) * actualState2d_x_ +
                          U_coms_(0) * actual_command_.x(),
                      S_coms_.block<1, 2>(0, 0) * actualState2d_y_ +
                          U_coms_(0) * actual_command_.y(),
                      com_height),
             eVector3(S_coms_.block<1, 2>(1, 0) * actualState2d_x_ +
                          U_coms_(1) * actual_command_.x(),
                      S_coms_.block<1, 2>(1, 0) * actualState2d_y_ +
                          U_coms_(1) * actual_command_.y(),
                      com_height),
             eVector3(S_coms_.block<1, 2>(2, 0) * actualState2d_x_ +
                          U_coms_(2) * actual_command_.x(),
                      S_coms_.block<1, 2>(2, 0) * actualState2d_y_ +
                          U_coms_(2) * actual_command_.y(),
                      com_height)}};
  } else if (settings_.cop_control_type == "j_ccc") {
    return {{eVector3(S_coms_j_.block<1, 3>(0, 0) * actualState3d_x_ +
                          U_coms_j_(0) * actual_command_.x(),
                      S_coms_j_.block<1, 3>(0, 0) * actualState3d_y_ +
                          U_coms_j_(0) * actual_command_.y(),
                      com_height),
             eVector3(S_coms_j_.block<1, 3>(1, 0) * actualState3d_x_ +
                          U_coms_j_(1) * actual_command_.x(),
                      S_coms_j_.block<1, 3>(1, 0) * actualState3d_y_ +
                          U_coms_j_(1) * actual_command_.y(),
                      com_height),
             eVector3(S_coms_j_.block<1, 3>(2, 0) * actualState3d_x_ +
                          U_coms_j_(2) * actual_command_.x(),
                      S_coms_j_.block<1, 3>(2, 0) * actualState3d_y_ +
                          U_coms_j_(2) * actual_command_.y(),
                      com_height)}};
  } else {
    throw std::runtime_error(
        "Invalid control type in CopStabilizer::getStableCoMs");
  }
}

template <typename T, typename vec_T>
T CopStabilizer::movingAverage(const T x, const unsigned long nb_samples,
                               vec_T &queue) {
  if (queue.capacity() < nb_samples) {
    queue.reserve(nb_samples);
  }
  queue.insert(queue.begin(), x);
  unsigned long n = queue.size();
  if (n > nb_samples) {
    queue.pop_back();
    n--;
  }
  T summation = 0 * x;
  for (T &element : queue) {
    summation += element;
  }
  return summation / n;
}

void CopStabilizer::getNonLinearPart(
    const eVector6 &leftFootWrench, const eVector6 &rightFootWrench,
    const Eigen::Vector2d &leftFootPlace, const Eigen::Vector2d &rightFootPlace,
    const Eigen::Vector2d &CoM, const Eigen::Vector2d &lateral_gravity,
    const Eigen::Vector2d &externalForce, eVector3 &n) {
  const double &m = settings_.robot_mass;
  Eigen::Vector2d CoM_acc =
      (leftFootWrench.head<2>() + rightFootWrench.head<2>() + externalForce) /
          m +
      lateral_gravity;
  Eigen::Vector2d leftFootPlace_c = leftFootPlace - CoM;
  Eigen::Vector2d rightFootPlace_c = rightFootPlace - CoM;
  getNonLinearPart(leftFootWrench, rightFootWrench, leftFootPlace_c,
                   rightFootPlace_c, CoM_acc, n);
}

void CopStabilizer::getNonLinearPart(
    const eVector6 &leftFootWrench, const eVector6 &rightFootWrench,
    const Eigen::Vector2d &leftFootPlace, const Eigen::Vector2d &rightFootPlace,
    const Eigen::Vector2d &CoM, const Eigen::Vector2d &CoM_acc, eVector3 &n) {
  /**
   * FeetWrenches are sorted as [force_x, force_y, force_z, torque_x, torque_y,
   * torque_z]
   */
  Eigen::Vector2d leftFootPlace_c = leftFootPlace - CoM;
  Eigen::Vector2d rightFootPlace_c = rightFootPlace - CoM;
  getNonLinearPart(leftFootWrench, rightFootWrench, leftFootPlace_c,
                   rightFootPlace_c, CoM_acc, n);
}

void CopStabilizer::getNonLinearPart(const eVector6 &leftFootWrench,
                                     const eVector6 &rightFootWrench,
                                     const Eigen::Vector2d &leftFootPlace_c,
                                     const Eigen::Vector2d &rightFootPlace_c,
                                     const Eigen::Vector2d &CoM_acc,
                                     eVector3 &n) {
  /**
   * FeetWrenches are sorted as [force_x, force_y, force_z, torque_x, torque_y,
   * torque_z] FeetPlaces_c are the feet places measured from the CoM of the
   * robot.
   */
  n << (CoM_acc) / w2_ + (leftFootWrench.block<2, 1>(3, 0) +
                          rightFootWrench.block<2, 1>(3, 0) +
                          leftFootPlace_c * leftFootWrench(2) +
                          rightFootPlace_c * rightFootWrench(2)) /
                             (leftFootWrench(2) + rightFootWrench(2)),
      0;
}

void CopStabilizer::getNonLinearPart(const eVector3 &com,
                                     const eVector3 &com_acc,
                                     const eVector3 &cop, eVector3 &n) {
  n << cop.head<2>() - (com.head<2>() - (com_acc.head<2>()) / w2_), 0;
}

void CopStabilizer::getNonLinearPart(eVector3 &n) { n.fill(0.0); }

bool CopStabilizer::isPointInPolygon(const eVector2 &point,
                                     const Polygon2D &polygon) {
  wykobi_2d_point_.x = point.x();
  wykobi_2d_point_.y = point.y();
  return wykobi::point_in_convex_polygon(wykobi_2d_point_, polygon);
}

double
CopStabilizer::estimateJerkDisturbance(const eVector3 &currentTrackingError,
                                       eVector3 &oldTrackingError,
                                       const eVector3 &c_gainK) {
  eVector3 B_ej(currentTrackingError -
                (A_ + B_ * c_gainK.transpose()) * oldTrackingError);
  oldTrackingError = currentTrackingError;
  return ((B_.transpose() * B_ej) / (B_.transpose() * B_))(0);
}

double
CopStabilizer::estimateCopDisturbance(const eVector2 &currentTrackingError,
                                      eVector2 &oldTrackingError,
                                      const eVector2 &c_gainK) {
  eVector2 B_ej(currentTrackingError -
                (A22_ + B2_ * c_gainK.transpose()) * oldTrackingError);
  double v = ((B2_.transpose() * B_ej) / (B2_.transpose() * B2_))(0);
  oldTrackingError = currentTrackingError;
  return v;
}

void CopStabilizer::setCOPgains(const eVector3 &cop_x_gains,
                                eVector3 &cop_y_gains) {
  settings_.cop_x_gains = cop_x_gains;
  settings_.cop_y_gains = cop_y_gains;
  cx_gainK_ = settings_.cop_x_gains;
  cy_gainK_ = settings_.cop_y_gains;
}

void CopStabilizer::setPCCgains(const double cop_pcc_gains) {
  settings_.cop_p_cc_gain = cop_pcc_gains;
  cx_gainK2_ << settings_.cop_p_cc_gain, settings_.cop_p_cc_gain / w_;
  cy_gainK2_ << settings_.cop_p_cc_gain, settings_.cop_p_cc_gain / w_;
}

void CopStabilizer::setIntegralGains(const eVector2 &integral_gains) {
  settings_.integral_gain = integral_gains;
}

} // namespace biped_stabilizer
