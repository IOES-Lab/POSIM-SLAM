// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "ping_marginal.hpp"
#include <basalt/vi_estimator/ba_base.h>
#include <optional>

namespace posim_slam {
struct PressureCalibration {Eigen::Vector3d imu_to_pressure=Eigen::Vector3d::Zero();};
struct PressureObservation {int64_t stamp;double height,variance;};
inline void validatePressure(const PressureCalibration& c,const PressureObservation& o) {
  if(!c.imu_to_pressure.allFinite() || c.imu_to_pressure.norm()>10 || o.stamp<=0 ||
      !std::isfinite(o.height) || std::abs(o.height)>20000 || !std::isfinite(o.variance) ||
      o.variance<=0 || o.variance>1e6)throw std::invalid_argument("invalid_pressure_observation");
}
struct PressureRow {Eigen::Matrix<double,1,6> pose;double datum,residual,cost;};
inline PressureRow pressureRow(const Sophus::SE3d& current,const Sophus::SE3d& reference,
    double datum,const PressureCalibration& c,const PressureObservation& o,bool robust=true) {
  validatePressure(c,o);
  if(!std::isfinite(datum))throw std::invalid_argument("invalid_pressure_datum");
  const double scale=1/std::sqrt(o.variance);
  const double residual=((current.translation()+current.rotationMatrix()*c.imu_to_pressure).z()+datum-o.height)*scale;
  const auto lever=reference.rotationMatrix()*c.imu_to_pressure;
  PressureRow row;row.pose<<0,0,scale,lever.y()*scale,-lever.x()*scale,0;
  const double norm=std::abs(residual),weight=robust && norm>3 ? 3/norm : 1.;
  row.cost=weight==1 ? .5*norm*norm : 3*(norm-1.5);
  row.pose*=std::sqrt(weight);row.datum=scale*std::sqrt(weight);row.residual=residual*std::sqrt(weight);
  if(!row.pose.allFinite() || !std::isfinite(row.residual))throw std::invalid_argument("nonfinite_pressure_rows");
  return row;
}

// One row retains ALL datum/state cross-information; orthogonal rows live in
// Basalt's state-only prior. They must be consumed together at marginalization.
struct PressureDatumPrior {
  basalt::AbsOrderMap order;
  Eigen::RowVectorXd states;
  double datum=0,b=0,reference=0;
  bool active=false;
};
struct PressureMarginalResult {basalt::MargLinData<double> states;PressureDatumPrior datum;};
inline PressureMarginalResult marginalizePressureSystem(const Eigen::MatrixXd& j,const Eigen::VectorXd& r,
    const basalt::AbsOrderMap& source,const basalt::AbsOrderMap& retained,
    const Eigen::VectorXd& delta,double datum) {
  requireStateOrder(source);requireStateOrder(retained);
  if(j.cols()!=Eigen::Index(source.total_size+1) || j.rows()!=r.size() || !j.rows() ||
      !j.allFinite() || !r.allFinite() || delta.size()!=Eigen::Index(retained.total_size) ||
      !delta.allFinite() || !std::isfinite(datum))throw std::invalid_argument("invalid_pressure_marginal_system");
  std::vector<bool> used(source.total_size,false);Eigen::MatrixXd ordered(j.rows(),j.cols());
  for(const auto& [stamp,block]:retained.abs_order_map) {
    const auto found=source.abs_order_map.find(stamp);
    if(found==source.abs_order_map.end() || found->second.second<block.second)
      throw std::invalid_argument("invalid_pressure_retained_state");
    for(int i=0;i<block.second;++i) {
      const int col=found->second.first+i;ordered.col(block.first+i)=j.col(col);used[col]=true;
    }
  }
  ordered.col(retained.total_size)=j.col(source.total_size);
  int col=retained.total_size+1;
  for(size_t i=0;i<source.total_size;++i)if(!used[i])ordered.col(col++)=j.col(i);
  Eigen::VectorXd rhs=r;
  if(ordered.rows()>ordered.cols()+1) {
    Eigen::MatrixXd stack(ordered.rows(),ordered.cols()+1);stack<<ordered,rhs;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(stack);
    const Eigen::MatrixXd compact=qr.matrixQR().topRows(stack.cols()).triangularView<Eigen::Upper>();
    ordered=compact.leftCols(ordered.cols());rhs=compact.rightCols(1);
  }
  const auto reduced=eliminateJointPatch(ordered,rhs,retained.total_size+1);
  Eigen::MatrixXd a=reduced.jacobian;Eigen::VectorXd b=reduced.residual-a.leftCols(retained.total_size)*delta;
  PressureMarginalResult out;out.states.is_sqrt=true;out.states.order=retained;
  // Rotate the single datum column into row zero without dropping correlation.
  const double norm=a.col(retained.total_size).norm();
  int first=0;
  if(norm>1e-12) {
    Eigen::MatrixXd column=a.rightCols(1);Eigen::HouseholderQR<Eigen::MatrixXd> qr(column);
    a=(qr.householderQ().adjoint()*a).eval();b=(qr.householderQ().adjoint()*b).eval();
    out.datum={retained,a.row(0).head(retained.total_size),a(0,retained.total_size),b[0],datum,true};first=1;
  }
  out.states.H=a.bottomRows(a.rows()-first).leftCols(retained.total_size);
  out.states.b=b.tail(b.size()-first);
  if(!out.states.H.rows()) {
    out.states.H=Eigen::MatrixXd::Zero(1,retained.total_size);out.states.b=Eigen::VectorXd::Zero(1);
  } else if(out.states.H.rows()>out.states.H.cols()) {
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(out.states.H);
    const Eigen::VectorXd transformed=qr.householderQ().adjoint()*out.states.b;
    out.states.H=qr.matrixQR().topRows(out.states.H.cols()).triangularView<Eigen::Upper>();
    out.states.b=transformed.head(out.states.H.cols());
  }
  if(!out.states.H.allFinite() || !out.states.b.allFinite())throw std::runtime_error("nonfinite_pressure_prior");
  return out;
}

class PressureFactorStore {
 public:
  struct Rows {Eigen::MatrixXd jacobian;Eigen::VectorXd residual;double cost=0.;};
  struct Retirement {uint64_t revision;std::set<int64_t> stamps;};
  explicit PressureFactorStore(PressureCalibration c):calibration_(c) {validatePressure(c,{1,0,1});}
  void add(const PressureObservation& o,const basalt::BundleAdjustmentBase<double>& owner) {
    if(trial_ || o.stamp<=last_stamp_ || observations_.size()>=kMaxLivePoseBlocks)
      throw std::invalid_argument("replayed_or_excess_pressure_observation");
    validatePressure(calibration_,o);
    const auto pose=owner.getPoseStateWithLin(o.stamp).getPose();
    observations_.emplace(o.stamp,o);
    // Numerical seed only: no prior is placed on the first noisy measurement.
    if(!initialized_) {datum_=o.height-(pose.translation()+pose.rotationMatrix()*calibration_.imu_to_pressure).z();initialized_=true;}
    last_stamp_=o.stamp;++revision_;
  }
  int64_t lastObservation() const {return last_stamp_;}
  double datum() const {return datum_;}
  size_t size() const {return observations_.size();}
  const PressureDatumPrior& prior() const {return prior_;}
  Rows linearize(const basalt::BundleAdjustmentBase<double>& owner,const basalt::AbsOrderMap& order,
      const std::set<int64_t>* selected=nullptr) const {
    requireStateOrder(order);Rows rows;
    const size_t count=selected ? selected->size() : observations_.size();
    rows.jacobian=Eigen::MatrixXd::Zero(count+int(prior_.active),order.total_size+1);
    rows.residual=Eigen::VectorXd::Zero(rows.jacobian.rows());size_t offset=0;
    if(prior_.active) {
      double residual=prior_.b+prior_.datum*(datum_-prior_.reference);
      for(const auto& [stamp,block]:prior_.order.abs_order_map) {
        const auto found=order.abs_order_map.find(stamp);
        if(found==order.abs_order_map.end() || found->second.second<block.second)
          throw std::invalid_argument("missing_pressure_prior_state");
        Eigen::VectorXd delta;
        if(block.second==15)delta=owner.frame_states.at(stamp).getDelta();
        else delta=owner.getPoseStateWithLin(stamp).getDelta();
        const auto coefficients=prior_.states.segment(block.first,block.second);
        rows.jacobian.block(0,found->second.first,1,block.second)=coefficients;
        residual+=coefficients.dot(delta);
      }
      rows.jacobian(0,order.total_size)=prior_.datum;rows.residual[0]=residual;
      rows.cost=.5*residual*residual;++offset;
    }
    for(const auto& [stamp,o]:observations_) {
      if(selected && !selected->count(stamp))continue;
      const auto found=order.abs_order_map.find(stamp);
      if(found==order.abs_order_map.end())throw std::invalid_argument("missing_pressure_pose");
      const auto state=owner.getPoseStateWithLin(stamp);
      const auto row=pressureRow(state.getPose(),state.getPoseLin(),datum_,calibration_,o);
      rows.jacobian.block<1,6>(offset,found->second.first)=row.pose;
      rows.jacobian(offset,order.total_size)=row.datum;rows.residual[offset]=row.residual;
      rows.cost+=row.cost;++offset;
    }
    if(offset!=size_t(rows.residual.size()))throw std::invalid_argument("unknown_pressure_retirement_stamp");
    return rows;
  }
  Retirement planRetirement(const std::set<int64_t>& removed) const {
    if(trial_)throw std::logic_error("pressure_retirement_during_trial");
    Retirement plan{revision_,{}};
    for(const auto& [stamp,o]:observations_)if(removed.count(stamp))plan.stamps.insert(stamp);
    return plan;
  }
  template<class Install> void retireWith(const Retirement& plan,const PressureDatumPrior& next,Install install) {
    if(trial_ || plan.revision!=revision_)throw std::invalid_argument("stale_pressure_retirement");
    for(auto stamp:plan.stamps)if(!observations_.count(stamp))throw std::invalid_argument("unknown_pressure_retirement_stamp");
    auto staged=next; // Allocation before the owner commit, swap afterwards.
    if(staged.active) {
      requireStateOrder(staged.order);
      if(staged.states.size()!=Eigen::Index(staged.order.total_size) || !staged.states.allFinite() ||
          !std::isfinite(staged.datum) || !std::isfinite(staged.b) || !std::isfinite(staged.reference))
        throw std::invalid_argument("invalid_pressure_prior");
    }
    install();std::swap(prior_,staged);
    for(auto stamp:plan.stamps)observations_.erase(stamp);
    ++revision_;
  }
  class Trial {
   public:
    explicit Trial(PressureFactorStore& store):store_(store),saved_(store.datum_) {
      if(store_.trial_)throw std::logic_error("nested_pressure_trial");store_.trial_=true;
    }
    ~Trial(){if(!finished_)finish(false);}
    void apply(double increment) {
      if(finished_ || !std::isfinite(increment) || !std::isfinite(store_.datum_+increment))
        throw std::invalid_argument("invalid_pressure_increment");
      store_.datum_+=increment;
    }
    void finish(bool accepted) {
      if(finished_)throw std::logic_error("pressure_trial_finished");
      if(!accepted)store_.datum_=saved_;
      store_.trial_=false;finished_=true;
    }
   private:PressureFactorStore& store_;double saved_;bool finished_=false;
  };
 private:
  PressureCalibration calibration_;std::map<int64_t,PressureObservation> observations_;
  PressureDatumPrior prior_;double datum_=0;bool initialized_=false,trial_=false;
  int64_t last_stamp_=0;uint64_t revision_=0;
};
}
