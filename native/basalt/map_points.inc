// POSIM modification of Basalt's get_current_points (BSD-3-Clause).
// The original file's copyright and license header is retained by the build.
// Export landmarks, not one copy per observing target frame. Keep real IDs.
template <class Scalar_>
template <class Scalar2>
void BundleAdjustmentBase<Scalar_>::get_current_points(
    Eigen::aligned_vector<Eigen::Matrix<Scalar2, 3, 1>>& points,
    std::vector<int>& ids) const {
  points.clear();
  ids.clear();
  for (const auto& entry : lmdb.getLandmarks()) {
    const auto& landmark = entry.second;
    const auto& host = landmark.host_kf_id;
    Sophus::SE3<Scalar> T_w_i;
    if (frame_states.count(host.frame_id)) {
      T_w_i = frame_states.at(host.frame_id).getState().T_w_i;
    } else if (frame_poses.count(host.frame_id)) {
      T_w_i = frame_poses.at(host.frame_id).getPose();
    } else {
      throw std::runtime_error("landmark_host_unavailable");
    }
    Vec4 point = StereographicParam<Scalar>::unproject(landmark.direction);
    point[3] = landmark.inv_dist;
    point = (T_w_i * calib.T_i_c[host.cam_id]).matrix() * point;
    points.emplace_back((point.template head<3>() / point[3]).template cast<Scalar2>());
    ids.emplace_back(entry.first);
  }
}

