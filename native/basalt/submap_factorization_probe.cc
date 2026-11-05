// SPDX-License-Identifier: Apache-2.0
#include "submap_factorization.hpp"
#include <cassert>
#include <iostream>
#include <numeric>

namespace {
using Graph=posim_slam::SubmapFactorization;
using Matrix=Eigen::MatrixXd;using Vector=Eigen::VectorXd;
template<class A,class B> void close(const A& a,const B& b) {
  assert(a.rows()==b.rows() && a.cols()==b.cols());
  if((a-b).norm()>=1e-9*std::max(1.,b.norm()))
    std::cerr<<"comparison "<<a.rows()<<"x"<<a.cols()<<" difference "<<(a-b).norm()<<" reference "<<b.norm()<<'\n';
  assert((a-b).norm()<1e-9*std::max(1.,b.norm()));
}
template<class F> void rejects(F call,const std::string& message) {
  bool caught=false;try{call();}catch(const std::exception& error){assert(error.what()==message);caught=true;}
  assert(caught);
}
Graph::Factor edge(const std::string& id,const std::string& original,int a,int b,double distance,double weight) {
  Graph::Rows row;row.originals={original};row.columns={a,b};
  row.jacobian=Matrix::Zero(1,2);row.jacobian<<weight,-weight;
  row.residual=Vector::Constant(1,-distance*weight);return {id,row};
}
struct Svd {Matrix inverse,nullspace;};
Svd decompose(const Matrix& matrix) {
  if(!matrix.rows())return {Matrix::Zero(matrix.cols(),0),Matrix::Identity(matrix.cols(),matrix.cols())};
  if(!matrix.cols())return {Matrix(0,matrix.rows()),Matrix(0,0)};
  Eigen::JacobiSVD<Matrix> svd(matrix,Eigen::ComputeThinU|Eigen::ComputeFullV);
  const auto& singular=svd.singularValues();int rank=0;
  while(rank<singular.size() && singular[rank]>1e-11*std::max(1.,singular[0]))++rank;
  return {svd.matrixV().leftCols(rank)*singular.head(rank).cwiseInverse().asDiagonal()*svd.matrixU().leftCols(rank).transpose(),
          svd.matrixV().rightCols(matrix.cols()-rank)};
}
// Independent full-system constrained least squares. Only observable queries
// are compared numerically; arbitrary coordinates in a free gauge need not
// have the same minimum-norm representative under conditional elimination.
void verify(Graph& graph,const Matrix& projection=Matrix()) {
  const auto state=graph.snapshot();std::vector<int64_t> order;
  for(auto column:state->information->known)order.push_back(column);
  std::map<int64_t,size_t> offsets;for(size_t i=0;i<order.size();++i)offsets[order[i]]=i;
  size_t rows=0,exact=0;
  for(const auto& [id,source]:state->factors){rows+=source->jacobian.rows();exact+=source->constraints.rows();}
  Matrix j=Matrix::Zero(rows,order.size()),c=Matrix::Zero(exact,order.size());Vector r(rows),target(exact);
  size_t n=0,m=0;std::set<std::string> originals;
  for(const auto& [id,source]:state->factors) {
    for(const auto& key:source->originals)assert(originals.insert(key).second);
    for(size_t i=0;i<source->columns.size();++i) {
      j.block(n,offsets.at(source->columns[i]),source->jacobian.rows(),1)=source->jacobian.col(i);
      if(source->constraints.rows())c.block(m,offsets.at(source->columns[i]),source->constraints.rows(),1)=source->constraints.col(i);
    }
    r.segment(n,source->residual.size())=source->residual;target.segment(m,source->target.size())=source->target;
    n+=source->jacobian.rows();m+=source->constraints.rows();
  }
  assert(originals==state->information->measurements);
  assert(originals.size()==state->assignments.size());
  const auto constraint=decompose(c);const Vector anchor=constraint.inverse*target;
  close(c*anchor,target);const auto reduced=decompose(j*constraint.nullspace);
  const Vector mean=anchor-constraint.nullspace*reduced.inverse*(j*anchor+r);
  const Matrix noise=constraint.nullspace*reduced.inverse,free=constraint.nullspace*reduced.nullspace;
  const Matrix d=projection.size()?projection:Matrix::Identity(order.size(),order.size());
  const auto result=graph.query(graph.solve(state),order,d);
  const bool observed=(d*free).norm()<1e-9;
  assert(result.observed==observed);
  if(observed){close(result.mean,d*mean);close(result.covariance,d*noise*noise.transpose()*d.transpose());}
  else assert(result.unbounded_gram.norm()>1e-9);
}
std::vector<Graph::Factor> factors() {
  auto ref=edge("reference","sensor:reference",5,4,0,1);
  ref.rows.columns={5};ref.rows.jacobian=Matrix::Constant(1,1,2);ref.rows.residual=Vector::Constant(1,-.2);
  return {edge("a","pixel:a",0,2,.3,5),edge("b","pixel:b",1,2,-.7,3),
    edge("c","pixel:c",2,5,.5,2),edge("d","pixel:d",3,4,1,1.4),
    edge("e","pixel:e",4,5,.2,5),ref};
}
void replacementAndGrowth() {
  Graph graph({{5},{{0},{1},{2},{3},{4}}});graph.replace({},factors());verify(graph);
  const auto old=graph.snapshot();const auto old_solution=graph.solve(old);
  const auto old_result=graph.query(old_solution,{0,1,2,3,4,5},Matrix::Identity(6,6));
  graph.replace({"a"},{edge("a","pixel:a",0,2,.8,4)});verify(graph);
  assert(graph.snapshot()->rebuilt==std::vector<size_t>({0,2,5}));
  for(auto i:{1,3,4})assert(old->buckets[i]==graph.snapshot()->buckets[i]);
  close(graph.query(old_solution,{0,1,2,3,4,5},Matrix::Identity(6,6)).covariance,old_result.covariance);
  auto merged=edge("merged","pixel:a",0,2,.8,4);
  merged.rows.originals={"pixel:a","pixel:b"};merged.rows.columns={0,1,2};
  merged.rows.jacobian=Matrix::Zero(2,3);merged.rows.jacobian<<4,.2,-4.2,.3,3,-3.3;
  merged.rows.residual=Vector(2);merged.rows.residual<<-3.2,2.1;
  const auto before_merge=graph.snapshot();graph.replace({"a","b"},{merged});verify(graph);
  assert(graph.snapshot()->buckets[3]==before_merge->buckets[3]);
  assert(graph.snapshot()->buckets[4]==before_merge->buckets[4]);
  const auto before_unassign=graph.snapshot();
  rejects([&]{graph.replace({"merged"},{edge("a","pixel:a",0,2,.8,4)});},"submap_factor_original_missing");
  assert(graph.snapshot()==before_unassign);
  graph.replace({"merged"},{edge("a","pixel:a",0,2,.8,4)},{"pixel:b"});verify(graph);
  assert(graph.snapshot()->originals.count("pixel:b") && !graph.snapshot()->assignments.count("pixel:b"));
  Matrix observed=Matrix::Zero(5,6);for(int row=0,column=0;column<6;++column)if(column!=1)observed(row++,column)=1;
  verify(graph,observed);
  graph.replace({}, {edge("b-again","pixel:b",1,2,-.7,3)});verify(graph);
  const auto before_growth=graph.snapshot();auto layout=before_growth->layout;
  layout.eliminated.push_back({5});layout.frontier={6};
  graph.replace({}, {edge("new-frame","pixel:new",5,6,-.4,3)}, {}, &layout);verify(graph);
  for(size_t i=0;i<5;++i)assert(graph.snapshot()->buckets[i]==before_growth->buckets[i]);
  assert(graph.snapshot()->rebuilt==std::vector<size_t>({5,6}));
  const auto latest=graph.snapshot();graph.replace({},{});verify(graph);
  assert(graph.snapshot()->rebuilt.empty() && graph.snapshot()->buckets==latest->buckets);
  graph.replace({"reference"},{},{"sensor:reference"});verify(graph);
  Matrix relative=Matrix::Zero(2,7);relative(0,0)=relative(1,3)=1;relative(0,2)=relative(1,6)=-1;
  verify(graph,relative);
  Graph other(layout);rejects([&]{other.solve(graph.snapshot());},"submap_factor_owner");
  rejects([&]{other.query(old_solution,{0},Matrix::Identity(1,1));},"submap_snapshot_owner");
}
void rollbackAndConstraints() {
  Graph graph({{5},{{0},{1},{2},{3},{4}}});graph.replace({},factors());
  const auto before=graph.snapshot();int calls=0;
  rejects([&]{graph.replace({"a"},{edge("a","pixel:a",0,2,.5,1)}, {},nullptr,[&]{return ++calls==5;});},"submap_factor_cancelled");
  assert(graph.snapshot()==before);
  rejects([&]{graph.replace({}, {edge("other","pixel:a",0,1,0,1)});},"submap_measurement_reused");
  rejects([&]{graph.replace({}, {edge("unknown","pixel:new",0,99,0,1)});},"submap_factor_coordinates");
  rejects([&]{graph.replace({"a"},{},{"not-owned"});},"submap_factor_unassignment");
  auto layout=before->layout;layout.eliminated[0]={1};
  rejects([&]{graph.replace({}, {},{},&layout);},"submap_factor_elimination_changed");
  layout=before->layout;layout.frontier={6};
  rejects([&]{graph.replace({}, {},{},&layout);},"submap_factor_coordinate_missing");
  assert(graph.snapshot()==before);
  auto exact=edge("exact","exact:0",0,2,0,1);exact.rows.jacobian=Matrix::Zero(0,2);exact.rows.residual.resize(0);
  exact.rows.constraints=Matrix::Zero(1,2);exact.rows.constraints<<1,0;exact.rows.target=Vector::Constant(1,1.1);
  graph.replace({}, {exact});verify(graph);const auto constrained=graph.snapshot();
  auto conflict=exact;conflict.id="conflict";conflict.rows.originals={"exact:conflict"};conflict.rows.target[0]=2;
  rejects([&]{graph.replace({}, {conflict});},"inconsistent_affine_constraints");
  assert(graph.snapshot()==constrained);
  exact.rows.target[0]=.4;graph.replace({"exact"},{exact});verify(graph);
  graph.replace({"exact"},{},{"exact:0"});verify(graph);
}
void quotasAndBatching() {
  auto limits=Graph::Information::Limits{};limits.active_columns=1;limits.temporary_columns=3;
  limits.nodes=5;limits.measurements=6;limits.numeric_bytes=16384;
  Graph graph({{5},{{0},{1},{2},{3},{4}}},limits);graph.replace({},factors());verify(graph);
  graph.replace({"b"},{},{"pixel:b"});const auto before=graph.snapshot();
  rejects([&]{graph.replace({}, {edge("new","pixel:new",1,2,0,1)});},"submap_measurement_capacity");
  auto huge=edge("huge","pixel:b",1,2,0,1);huge.rows.jacobian=Matrix::Ones(4096,2);huge.rows.residual=Vector::Zero(4096);
  rejects([&]{graph.replace({}, {huge});},"submap_factor_numeric_capacity");
  auto layout=before->layout;layout.eliminated.push_back({5});layout.frontier={6};
  rejects([&]{graph.replace({}, {},{},&layout);},"submap_factor_layout_capacity");
  assert(graph.snapshot()==before);
  Graph batches({{1},{{0}}});std::vector<Graph::Factor> all;
  for(int i=0;i<3;++i) {
    auto factor=edge(std::to_string(i),"source:"+std::to_string(i),0,1,.1*i,1);
    factor.rows.jacobian=Matrix::Ones(2100,2);factor.rows.jacobian.col(1).setConstant(-1);
    factor.rows.residual=Vector::Constant(2100,-.1*i);all.push_back(std::move(factor));
  }
  batches.replace({},all);Matrix relative(1,2);relative<<1,-1;verify(batches,relative);
  assert(batches.snapshot()->information->measurements.size()==3);
}
} // namespace
int main() {
  replacementAndGrowth();rollbackAndConstraints();quotasAndBatching();
  std::cout<<"source factorization: archived replacement, merge, unassignment, growth, shared branches, gauge, constraints and rollback passed\n";
}
