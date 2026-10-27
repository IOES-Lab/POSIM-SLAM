// POSIM modification of Basalt's get_current_points (BSD-3-Clause).
// The original file's copyright and license header is retained by the build.
// Export landmarks, not one copy per observing target frame. Keep real IDs.
template <class Scalar_>
template <class Scalar2>
void BundleAdjustmentBase<Scalar_>::get_current_points(
    Eigen::aligned_vector<Eigen::Matrix<Scalar2, 3, 1>>& points,
    std::vector<int>& ids) const {
  points.clear();
  posim_map_quality.clear();
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
    const Vec3 world=point.template head<3>()/point[3];
    posim_slam::VisualPointSupport quality;
    if(world.allFinite() && landmark.inv_dist>0) {
      std::vector<Eigen::Matrix<double,2,3>> rows;
      for(const auto& [target,observation]:landmark.obs) {
        Sophus::SE3<Scalar> pose;
        if(frame_states.count(target.frame_id))pose=frame_states.at(target.frame_id).getState().T_w_i;
        else if(frame_poses.count(target.frame_id))pose=frame_poses.at(target.frame_id).getPose();
        else continue;
        const auto inverse=(pose*calib.T_i_c[target.cam_id]).inverse();
        Vec4 local;local.template head<3>()=inverse*world;local[3]=1;
        Vec2 pixel;Eigen::Matrix<Scalar,2,4> derivative;
        if(!calib.intrinsics[target.cam_id].project(local,pixel,&derivative)
            || !pixel.allFinite() || !derivative.allFinite()
            || (pixel-observation).norm()>huber_thresh)continue;
        rows.push_back((derivative.template leftCols<3>()*inverse.rotationMatrix()/obs_std_dev).template cast<double>());
      }
      Eigen::MatrixXd jacobian(2*rows.size(),3);
      for(size_t i=0;i<rows.size();++i)jacobian.middleRows(2*i,2)=rows[i];
      const Vec3 host_origin=(T_w_i*calib.T_i_c[host.cam_id]).translation();
      quality=posim_slam::visualPointSupport(jacobian,(world-host_origin).template cast<double>(),rows.size());
    }
    if(posim_map_information_enabled) {
      quality.joint_information_evaluated=true;
      const auto joint=posim_point_information.find(entry.first);
      if(joint!=posim_point_information.end()) {
        quality.joint_position_observed=joint->second.world.observed;
        quality.joint_position_sigma_m=joint->second.world.maximum_sigma_m;
        quality.joint_host_position_observed=joint->second.host_camera.observed;
        quality.joint_host_position_sigma_m=joint->second.host_camera.maximum_sigma_m;
        quality.conditional_host_position_sigma_m=joint->second.conditional_host_camera.maximum_sigma_m;
      }
      if(posim_map_point_three_sigma_m>0)
        quality.eligible=quality.eligible && quality.joint_position_observed &&
            3*quality.joint_position_sigma_m<=posim_map_point_three_sigma_m;
    }
    posim_map_quality.emplace(entry.first,quality);
    points.emplace_back(world.template cast<Scalar2>());
    ids.emplace_back(entry.first);
  }
}

