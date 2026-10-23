"""Build-local extensions of the exact pinned Basalt landmark interface.

Upstream files stay untouched; existing notices remain in their source tree.
Only unscaled, pre-QR visual rows may enter a coupled Ping patch.
"""
import hashlib
from pathlib import Path
import sys

HASHES={
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
    for name,checksum in HASHES.items():
        raw=(Path(source)/'include/basalt'/name).read_bytes()
        if hashlib.sha256(raw).hexdigest()!=checksum:raise ValueError('pinned_basalt_joint_header_changed:'+name)
        text=raw.decode()
        if name.endswith('/ba_base.h'):
            text=replace_once(text,'#pragma once','#pragma once\n#include "ping_state.hpp"')
            text=replace_once(text,'  inline void backup() {','''  posim_slam::PingPlaneStore posim_ping_planes;
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
            text=replace_once(text,'  State state = State::Uninitialized;','  bool posim_joint_raw_ = false;\n  State state = State::Uninitialized;')
        target=output/name
        target.parent.mkdir(parents=True,exist_ok=True)
        rendered='// Build-local POSIM joint-landmark interface; see prepare_joint_headers.py.\n'+text
        if not target.exists() or target.read_text()!=rendered:target.write_text(rendered)


if __name__=='__main__':prepare(sys.argv[1],sys.argv[2])
