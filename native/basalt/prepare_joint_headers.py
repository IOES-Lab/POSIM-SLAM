"""Build-local extensions of the exact pinned Basalt landmark interface.

Upstream files stay untouched; existing notices remain in their source tree.
Only unscaled, pre-QR visual rows may enter a coupled Ping patch.
"""
import hashlib
from pathlib import Path
import sys

HASHES={
    'optical_flow/optical_flow.h':'8510a2b64f05c5ee2870b1fbf7763e47dbeb8978e8c87c23fc037d33d4589696',
    'optical_flow/patch_optical_flow.h':'b862f0931aab5c89ddadb92346a2a22aca219a630cf83a4776985f61f97f96ef',
    'vi_estimator/vio_estimator.h':'48ca1e3bc8fabc7378ddd8c3b913dca93ea12cba104192145b019c16599ac754',
    'vi_estimator/sqrt_keypoint_vio.h':'1c98d1cb3efeab33d6df6371f4294880678ae854c18a8cd496abf0d0e14522b3',
    'linearization/landmark_block.hpp':'491cbd5cb633f13923b8c440364e2b72db1ea8abf5b88c604a348367fa57115e',
    'linearization/landmark_block_abs_dynamic.hpp':'5ed3a2082aa09ad41c22dd7ce36e4e904329b3d9ed446c84b106e57dc88e8927',
    'vi_estimator/ba_base.h':'4cab57e494fdb04d8a6c72bfc0e2046d5cfc11a2a02895cbaafbafd16a37add2',
}

METHODS='''
  // POSIM extension: raw columns use Basalt's stereographic direction and
  // inverse-distance coordinates. Callers must transform Ping Jacobians to
  // these same coordinates before grouping, never treat them as map XYZ.
  void exportJointLinearization(MatX& jp, MatX& jl, VecX& residual) const override {
    if (!posim_joint_raw_ || state != State::Linearized)
      throw std::logic_error("joint_rows_require_raw_linearization");
    jp = storage.topLeftCorner(num_rows - 3, padding_idx);
    jl = storage.block(0, lm_idx, num_rows - 3, 3);
    residual = storage.col(res_idx).head(num_rows - 3);
  }

  bool jointIncrementValid(const Vec3& delta) const override {
    return posim_joint_raw_ && state == State::Linearized && delta.allFinite()
        && (lm_ptr->direction + delta.template head<2>()).allFinite()
        && std::isfinite(lm_ptr->inv_dist + delta[2])
        && lm_ptr->inv_dist + delta[2] >= Scalar(0);
  }

  void applyJointIncrement(const Vec3& delta) override {
    if (!jointIncrementValid(delta))
      throw std::invalid_argument("invalid_joint_landmark_increment");
    lm_ptr->direction += delta.template head<2>();
    lm_ptr->inv_dist += delta[2];
    posim_joint_raw_ = false;
    state = State::Allocated;
  }
'''


def replace_once(text,old,new):
    if text.count(old)!=1:raise ValueError('pinned_basalt_joint_anchor_changed')
    return text.replace(old,new,1)


def prepare(source,destination):
    output=Path(destination)/'basalt';output.mkdir(parents=True,exist_ok=True)
    raw=(Path(source).parent/'headers/include/basalt/imu/preintegration.h').read_bytes()
    if hashlib.sha256(raw).hexdigest()!='08e2fe5272b3dc81e7b019861d4d8e3613d5cc5f62f81ebee9d3ae0c3399fcbd':
        raise ValueError('pinned_basalt_preintegration_header_changed')
    header=replace_once(raw.decode(),'#pragma once','#pragma once\n#include <stdexcept>\n#include <cmath>')
    extension=Path(__file__).with_name('imu_continuous.inc').read_text()
    header=replace_once(header,'  /// @brief Predict state given this pseudo-measurement',
                        extension+'  /// @brief Predict state given this pseudo-measurement')
    target=output/'imu/preintegration.h';target.parent.mkdir(parents=True,exist_ok=True)
    if not target.exists() or target.read_text()!=header:target.write_text(header)
    for name,checksum in HASHES.items():
        raw=(Path(source)/'include/basalt'/name).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=checksum:raise ValueError('pinned_basalt_joint_header_changed:'+name)
        text=raw.decode()
        if name=='optical_flow/optical_flow.h':
            text=replace_once(text,'  int64_t t_ns;\n  std::vector<Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>>',
                '  int64_t t_ns;\n  uint64_t posim_owner_control_id = 0;\n  size_t reference_patch_count = 0;\n  size_t reference_patch_peak = 0;\n  std::vector<Eigen::aligned_map<KeypointId, Eigen::AffineCompact2f>>')
        elif name=='optical_flow/patch_optical_flow.h':
            text=replace_once(text,'#include <basalt/optical_flow/optical_flow.h>',
                '#include <basalt/optical_flow/optical_flow.h>\n#include "reference_patches.hpp"')
            text=replace_once(text,'// TODO: patches are currently never erased, so over time memory consumption\n// increases',
                '// POSIM: reference patches retire after their final camera observation.')
            text=replace_once(text,'    if (output_queue && frame_counter % config.optical_flow_skip_frames == 0) {',
                '''    reference_patch_peak = std::max(reference_patch_peak, patches.size());
    posim_slam::retain_observed_references(patches, transforms->observations);
    transforms->reference_patch_count = patches.size();
    transforms->reference_patch_peak = reference_patch_peak;
    if (output_queue && frame_counter % config.optical_flow_skip_frames == 0) {''')
            text=replace_once(text,'  size_t frame_counter;',
                '  size_t frame_counter;\n  size_t reference_patch_peak = 0;')
        elif name.endswith('/vio_estimator.h'):
            text=replace_once(text,'#pragma once','#pragma once\n#include <map>\n#include "visual_geometry.hpp"')
            text=replace_once(text,'  std::vector<int> point_ids;','  std::vector<int> point_ids;\n  std::map<int,posim_slam::VisualPointSupport> map_quality;\n  std::string map_information_error;')
        elif name.endswith('/sqrt_keypoint_vio.h'):
            text=replace_once(text,'#pragma once','#pragma once\n#include <functional>\nnamespace posim_slam { class PingFactorStore; class SonarFactorStore; class DvlFactorStore; class PressureFactorStore; struct NativeSurfaceInformation; }')
            text=replace_once(text,'  using Scalar = Scalar_;','''  using Scalar = Scalar_;
  // Configured before input is submitted; called only by the estimator thread.
  std::function<void(SqrtKeypointVioEstimator&, int64_t)> posim_before_marginalization;
  std::function<void(SqrtKeypointVioEstimator&, int64_t)> posim_after_joint;
  std::function<void(SqrtKeypointVioEstimator&, int64_t)> posim_after_marginalization;
  std::function<void(SqrtKeypointVioEstimator&, uint64_t, MargLinData<Scalar>&)> posim_owner_control;
  std::shared_ptr<posim_slam::PingFactorStore> posim_ping_factors;
  std::shared_ptr<posim_slam::SonarFactorStore> posim_sonar_factors;
  std::shared_ptr<posim_slam::DvlFactorStore> posim_dvl_factors;
  std::shared_ptr<posim_slam::PressureFactorStore> posim_pressure_factors;
  bool posim_joint_started = false;
  bool posim_had_visual_landmarks = false;
  int posim_joint_last_accepted = 0;
  uint64_t posim_projection_rejections = 0;
  void posim_optimize_joint();
  posim_slam::NativeSurfaceInformation posim_surface_information();
  void posim_update_map_information();''')
        elif name.endswith('/ba_base.h'):
            text=replace_once(text,'#pragma once','#pragma once\n#include "ping_state.hpp"\n#include "consumed_visual.hpp"\n#include "visual_geometry.hpp"\n#include "map_information.hpp"')
            text=replace_once(text,'  inline void backup() {','''  mutable std::map<int,posim_slam::VisualPointSupport> posim_map_quality;
  bool posim_map_information_enabled = false;
  double posim_map_point_three_sigma_m = 0.;
  std::map<int,posim_slam::MapLandmarkInformation> posim_point_information;
  std::string posim_map_information_error;
  posim_slam::PingPlaneStore posim_ping_planes;
  posim_slam::ConsumedVisualObservations posim_consumed_visual;
  bool posim_ping_trial_active = false;
  uint64_t posim_ping_state_revision = 0;

  inline void backup() {
    posim_ping_planes.backup();''')
            text=replace_once(text,'  inline void restore() {','  inline void restore() {\n    ++posim_ping_state_revision;\n    posim_ping_planes.restore();')
        elif name.endswith('/landmark_block.hpp'):
            marker='  virtual void performQR() = 0;'
            text=replace_once(text,marker,'''  virtual void exportJointLinearization(MatX&, MatX&, VecX&) const = 0;
  virtual bool jointIncrementValid(const Vec3&) const = 0;
  virtual void applyJointIncrement(const Vec3&) = 0;
'''+marker)
        else:
            text=replace_once(text,'#pragma once','#pragma once\n#include <stdexcept>')
            text=replace_once(text,'  virtual inline void allocateLandmark(',METHODS+'\n  virtual inline void allocateLandmark(')
            text=replace_once(text,'    storage.setZero();','    storage.setZero();\n    posim_joint_raw_ = false;')
            text=replace_once(text,'      state = State::Linearized;','      state = State::Linearized;\n      posim_joint_raw_ = true;')
            text=replace_once(text,'  virtual inline void scaleJl_cols() override {','  virtual inline void scaleJl_cols() override {\n    posim_joint_raw_ = false;')
            # QR only mixes rows: columns absent from this landmark remain
            # exactly zero, including inertial velocity/bias columns. Assemble
            # the same dense normal system through its observed pose blocks.
            text=replace_once(text,"""    H.noalias() += J.transpose() * J;
    b.noalias() += J.transpose() * r;""","""    std::vector<size_t> offsets;
    for (const auto& item : res_idx_by_abs_pose_)
      offsets.push_back(aom_->abs_order_map.at(item.first).first);
    MatX compact(J.rows(), POSE_SIZE * offsets.size());
    for (size_t i = 0; i < offsets.size(); ++i)
      compact.middleCols(POSE_SIZE * i, POSE_SIZE) =
          J.middleCols(offsets[i], POSE_SIZE);
    const MatX normal = compact.transpose() * compact;
    const VecX gradient = compact.transpose() * r;
    for (size_t i = 0; i < offsets.size(); ++i) {
      b.template segment<POSE_SIZE>(offsets[i]) +=
          gradient.template segment<POSE_SIZE>(POSE_SIZE * i);
      for (size_t j = 0; j < offsets.size(); ++j)
        H.template block<POSE_SIZE, POSE_SIZE>(offsets[i], offsets[j]) +=
            normal.template block<POSE_SIZE, POSE_SIZE>(POSE_SIZE * i, POSE_SIZE * j);
    }""")
            text=replace_once(text,'  State state = State::Uninitialized;','  bool posim_joint_raw_ = false;\n  State state = State::Uninitialized;')
        target=output/name
        target.parent.mkdir(parents=True,exist_ok=True)
        rendered='// Build-local POSIM joint-landmark interface; see prepare_joint_headers.py.\n'+text
        if not target.exists() or target.read_text()!=rendered:target.write_text(rendered)


    raw=(Path(source)/'src/vi_estimator/sqrt_keypoint_vio.cpp').read_bytes()
    if hashlib.sha256(raw).hexdigest()!='141dc9747dfb60e19e8ab93eab3b5600a6bb95fc249afdd5e15f4d844276c5c9':
        raise ValueError('pinned_basalt_owner_source_changed')
    # A bootstrap state has no preceding IMU interval. Upstream inserts an
    # empty placeholder and waits for later visual frames before optimization;
    # auxiliary factors can optimize immediately. Only measure() may publish a
    # completed integration, otherwise the IMU bias rows divide by sqrt(0).
    text=replace_once(raw.decode(),'''  imu_meas[t_ns] = IntegratedImuMeasurement<Scalar>(t_ns, bg.cast<Scalar>(),
                                                    ba.cast<Scalar>());''',
        '  // No completed IMU interval at the initial state.')
    text=replace_once(text,'      // Correct camera time offset', '''      // Control events execute on this same estimator thread and do not
      // consume IMU samples, create a pose or advance the camera/sonar clock.
      if (curr_frame->posim_owner_control_id) {
        if (config.vio_enforce_realtime || !posim_owner_control)
          throw std::invalid_argument("owner_control_requires_ordered_processing");
        posim_owner_control(*this, curr_frame->posim_owner_control_id, marg_data);
        continue;
      }
      // Correct camera time offset''')
    # PoseVelBiasState::applyInc stores gyro at 9:12 and accel at 12:15.
    # The pinned constructor applies the named initial weights in reverse.
    # Correct both square-root and Hessian branches without changing weights.
    for wrapper in ('std::sqrt(Scalar({}))', 'Scalar({})'):
        for index, before, after in ((9, 'ba', 'bg'), (12, 'bg', 'ba')):
            prefix = f'marg_data.H.diagonal().template segment<3>({index}).array() =\n        '
            text=replace_once(text, prefix+wrapper.format(f'config.vio_init_{before}_weight')+';',
                              prefix+wrapper.format(f'config.vio_init_{after}_weight')+';')
    text=replace_once(text,'calib.dicrete_time_accel_noise_std().array().square()',
                      'calib.accel_noise_std.array().square()')
    text=replace_once(text,'calib.dicrete_time_gyro_noise_std().array().square()',
                      'calib.gyro_noise_std.array().square()')
    if text.count('meas->integrate(*data, accel_cov, gyro_cov);')!=2:
        raise ValueError('pinned_basalt_preintegration_calls_changed')
    text=text.replace('meas->integrate(*data, accel_cov, gyro_cov);',
                      'meas->integrateContinuous(*data, accel_cov, gyro_cov);')
    text=replace_once(text,'''        imu_meas[last_state_t_ns] =
            IntegratedImuMeasurement<Scalar>(last_state_t_ns, bg, ba);''',
        '        // No completed IMU interval at the initial state.')
    text=replace_once(text,'  optimize();\n  marginalize(num_points_connected, lost_landmaks);',
        # Camera/IMU also needs the complete factor-ownership path: a lost
        # landmark may be hosted by a still-retained inertial state. The
        # upstream short-window marginal order omits such younger hosts.
        '  if (!posim_joint_started && lmdb.numLandmarks() > 0) {\n'
        '    posim_joint_started = true; opt_started = true;\n'
        '  }\n'
        '  if (posim_joint_started) posim_optimize_joint(); else optimize();\n'
        '  const auto acoustic_revision = posim_sonar_factors ? posim_sonar_factors->revision() : 0;\n'
        '  if (posim_before_marginalization) posim_before_marginalization(*this, last_state_t_ns);\n'
        '  const bool new_acoustic = posim_sonar_factors && posim_sonar_factors->size() &&\n'
        '    (!posim_joint_started || posim_sonar_factors->revision() != acoustic_revision);\n'
        '  const bool new_dvl = posim_dvl_factors && posim_dvl_factors->lastObservation() == last_state_t_ns;\n'
        '  const bool new_pressure = posim_pressure_factors && posim_pressure_factors->lastObservation() == last_state_t_ns;\n'
        '  if ((posim_ping_factors && posim_ping_factors->lastObservation() == last_state_t_ns) || new_acoustic || new_dvl || new_pressure) {\n'
        '    posim_joint_started = true; opt_started = true; posim_optimize_joint();\n'
        '  }\n'
        '  if (posim_after_joint) posim_after_joint(*this, last_state_t_ns);\n'
        '  if (this->posim_map_information_enabled) posim_update_map_information();\n'
        '  // Preserve the first unobserved startup, not every later visual dropout.\n'
        '  posim_had_visual_landmarks = posim_had_visual_landmarks || lmdb.numLandmarks() > 0;\n'
        '  if (posim_had_visual_landmarks || posim_joint_started) marginalize(num_points_connected, lost_landmaks);\n'
        '  if (posim_after_marginalization) posim_after_marginalization(*this, last_state_t_ns);')
    # An empty camera list explicitly denotes a non-camera sensor event. It
    # advances the inertial state without declaring every visual track lost.
    text=replace_once(text,'  // Make new residual for existing keypoints',
        '  const bool posim_auxiliary = opt_flow_meas->observations.empty();\n'
        '  // Make new residual for existing keypoints')
    text=replace_once(text,'  if (take_kf) {', '  if (!posim_auxiliary && take_kf) {')
    text=replace_once(text,'    frames_after_kf++;', '    if (!posim_auxiliary) frames_after_kf++;')
    text=replace_once(text,'  std::unordered_set<KeypointId> lost_landmaks;\n  if (config.vio_marg_lost_landmarks) {',
        '  std::unordered_set<KeypointId> lost_landmaks;\n  if (!posim_auxiliary && config.vio_marg_lost_landmarks) {')
    text='#include "ping_factors.hpp"\n#include "ping_optimizer.hpp"\n#include "joint_map_information.hpp"\n#include "surface_information.hpp"\n#include <cstdlib>\n'+text
    text=replace_once(text,'      if (kv.first > last_state_to_marg) break;',
        '      if (!posim_joint_started && kv.first > last_state_to_marg) break;')
    text=replace_once(text,'      if (kv.first != last_state_to_marg) {',
        '      if (kv.first < last_state_to_marg) {')
    # A delayed first marginalization can retire several startup keyframes at
    # once. Each retained pose needs a valid FEJ reference before it enters the
    # prior; the upstream short-window path normally converts only old states.
    text=replace_once(text,'      PoseStateWithLin<Scalar> pose(state);',
        '      PoseStateWithLin<Scalar> pose(state);\n'
        '      if (!pose.isLinearized()) pose.setLinTrue();')
    # Keyframes can still own full velocity/bias states in a longer sensor
    # window. Score their poses through the common accessor rather than assume
    # every older keyframe has already become a pose-only state.
    for old in ('frame_poses.at(*it1).getPose().translation()',
                'frame_poses.at(*it2).getPose().translation()',
                'frame_states.at(last_kf).getState().T_w_i.translation()'):
        expected = 2 if '*it1' in old else 1
        if text.count(old) != expected:
            raise ValueError('pinned_basalt_keyframe_score_anchor_changed')
        stamp = '*it1' if '*it1' in old else ('*it2' if '*it2' in old else 'last_kf')
        text = text.replace(old, 'this->getPoseStateWithLin(' + stamp + ').getPose().translation()')
    text=replace_once(text,'    Timer t_actual_marg;', '''    if (posim_joint_started) {
      std::set<int64_t> removed = poses_to_marg;
      removed.insert(states_to_marg_all.begin(), states_to_marg_all.end());
      for (int64_t stamp : removed) states_to_marg_vel_bias.erase(stamp);
      ImuLinData<double> imu = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};
      for (const auto& [stamp, measurement] : imu_meas) imu.imu_meas[stamp] = &measurement;
      posim_ping_factors->marginalize(aom, marg_data, imu_meas, imu, removed,
                                    states_to_marg_vel_bias, lost_landmaks, posim_sonar_factors.get(), posim_dvl_factors.get(), posim_pressure_factors.get());
      for (int64_t stamp : removed) {
        kf_ids.erase(stamp); num_points_kf.erase(stamp); prev_opt_flow_res.erase(stamp);
      }
      for (auto& [stamp, count] : num_points_kf) count = 0;
      for (const auto& [id, point] : lmdb.getLandmarks())
        if (num_points_kf.count(point.host_kf_id.frame_id)) ++num_points_kf.at(point.host_kf_id.frame_id);
      this->posim_consumed_visual.prune(prev_opt_flow_res);
      return;
    }
    Timer t_actual_marg;''')
    text=replace_once(text,'template <class Scalar_>\nvoid SqrtKeypointVioEstimator<Scalar_>::debug_finalize()', '''template <class Scalar_>
posim_slam::NativeSurfaceInformation SqrtKeypointVioEstimator<Scalar_>::posim_surface_information() {
  auto& base = static_cast<BundleAdjustmentBase<double>&>(*this);
  const auto order = posim_slam::planPingMarginalLayout(base, {}, {}, {}, {}).source;
  if (!posim_ping_factors || !posim_ping_factors->surfaceOwner() || !posim_pressure_factors ||
      !posim_ping_factors->specs(order).empty() ||
      (posim_sonar_factors && posim_sonar_factors->size()))
    throw std::invalid_argument("exclusive_surface_information_required");
  ImuLinData<double> imu = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};
  for (const auto& [stamp, measurement] : imu_meas) imu.imu_meas[stamp] = &measurement;
  return posim_slam::nativeSurfaceInformation(base, order, marg_data, imu, posim_dvl_factors.get(),
      *posim_pressure_factors, *posim_ping_factors->surfaceOwner());
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::posim_optimize_joint() {
  if (!posim_ping_factors) {
    // An empty auxiliary-factor store is valid for camera/IMU estimation.
    posim_ping_factors = std::make_shared<posim_slam::PingFactorStore>(
        static_cast<BundleAdjustmentBase<double>&>(*this));
  }
  auto& base = static_cast<BundleAdjustmentBase<double>&>(*this);
  const auto order = posim_slam::planPingMarginalLayout(base, {}, {}, {}, {}).source;
  ImuLinData<double> imu = {g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};
  for (const auto& [stamp, measurement] : imu_meas) imu.imu_meas[stamp] = &measurement;
  const bool trace_joint = std::getenv("POSIM_SLAM_TRACE_JOINT") != nullptr;
  if (trace_joint) std::cerr << "POSIM_JOINT before " << last_state_t_ns
      << " bias " << frame_states.at(last_state_t_ns).getState().bias_accel.transpose()
      << " prior " << marg_data.H.norm() << " " << marg_data.b.norm()
      << " sonar " << (posim_sonar_factors ? posim_sonar_factors->size() : 0) << std::endl;
  if (posim_ping_factors->surfaceOwner()) {
    if (!posim_pressure_factors) throw std::invalid_argument("surface_owner_requires_pressure");
    posim_ping_factors->adoptSurfaceOwner(order, marg_data, *posim_pressure_factors);
  }
  const auto report = posim_slam::optimizePingGroups(base, order,
      posim_ping_factors->specs(order), marg_data, imu, {}, posim_sonar_factors.get(), posim_dvl_factors.get(), posim_pressure_factors.get(), posim_ping_factors->surfaceOwner());
  if (trace_joint) std::cerr << "POSIM_JOINT after " << last_state_t_ns
      << " bias " << frame_states.at(last_state_t_ns).getState().bias_accel.transpose()
      << " cost " << report.initial_cost << " " << report.final_cost
      << " steps " << report.accepted << " " << report.rejected << std::endl;
  posim_joint_last_accepted = report.accepted;
  posim_projection_rejections += report.projection_rejections;
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::posim_update_map_information() {
  this->posim_point_information.clear();this->posim_map_information_error.clear();
  if (!lmdb.numLandmarks()) return;
  try {
    auto& base=static_cast<BundleAdjustmentBase<double>&>(*this);
    const auto order=posim_slam::planPingMarginalLayout(base, {}, {}, {}, {}).source;
    ImuLinData<double> imu={g, gyro_bias_sqrt_weight, accel_bias_sqrt_weight, {}};
    for(const auto& [stamp,measurement]:imu_meas)imu.imu_meas[stamp]=&measurement;
    const auto groups=posim_ping_factors ? posim_ping_factors->specs(order) : std::vector<posim_slam::JointPingGroupSpec>{};
    this->posim_point_information=posim_slam::jointMapInformation(base,order,groups,marg_data,imu,
        posim_sonar_factors.get(),posim_dvl_factors.get(),posim_pressure_factors.get());
  } catch(const std::exception& error) {
    // No certificate on failed queries; independently valid tracking continues.
    this->posim_point_information.clear();this->posim_map_information_error=error.what();
  }
}

template <class Scalar_>
void SqrtKeypointVioEstimator<Scalar_>::debug_finalize()''')
    text=replace_once(text,'    get_current_points(data->points, data->point_ids);',
        '    get_current_points(data->points, data->point_ids);\n    data->map_quality = this->posim_map_quality;\n    data->map_information_error = this->posim_map_information_error;')
    text=replace_once(text,'  prev_opt_flow_res[opt_flow_meas->t_ns] = opt_flow_meas;',
        '  prev_opt_flow_res[opt_flow_meas->t_ns] = opt_flow_meas;\n'
        '  this->posim_consumed_visual.prune(prev_opt_flow_res);')
    text=replace_once(text,'      int kpt_id = kv_obs.first;',
        '      int kpt_id = kv_obs.first;\n'
        '      if (!this->posim_consumed_visual.allows(kpt_id, opt_flow_meas->t_ns)) continue;')
    text=replace_once(text,'    for (int lm_id : unconnected_obs0) {',
        '    for (int lm_id : unconnected_obs0) {\n'
        '      if (!this->posim_consumed_visual.allows(lm_id, opt_flow_meas->t_ns)) continue;')
    text=replace_once(text,'      for (const auto& kv : prev_opt_flow_res) {',
        '      for (const auto& kv : prev_opt_flow_res) {\n'
        '        if (!this->posim_consumed_visual.allows(lm_id, kv.first)) continue;')
    target=Path(destination)/'sqrt_keypoint_vio.cpp'
    if not target.exists() or target.read_text()!=text:target.write_text(text)


if __name__=='__main__':prepare(sys.argv[1],sys.argv[2])
