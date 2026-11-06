// SPDX-License-Identifier: Apache-2.0
#include "submap_factorization.hpp"
#include <cassert>
#include <iostream>
#include <numeric>
#include <sqlite3.h>
#include <thread>

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
  for(const auto& [id,source]:state->factors){rows+=source->rows();exact+=source->exactRows();}
  Matrix j=Matrix::Zero(rows,order.size()),c=Matrix::Zero(exact,order.size());Vector r(rows),target(exact);
  size_t n=0,m=0;std::set<std::string> originals;
  for(const auto& [id,page]:state->factors) {
    const auto source=page->pinNumeric();
    page->eachOriginal([&](const auto& key){assert(originals.insert(key).second);});
    for(size_t i=0;i<source->columns.size();++i) {
      j.block(n,offsets.at(source->columns[i]),source->jacobian.rows(),1)=source->jacobian.col(i);
      if(source->constraints.rows())c.block(m,offsets.at(source->columns[i]),source->constraints.rows(),1)=source->constraints.col(i);
    }
    r.segment(n,source->residual.size())=source->residual;target.segment(m,source->target.size())=source->target;
    n+=source->jacobian.rows();m+=source->constraints.rows();
  }
  assert(originals==state->information->measurements);
  assert(originals.size()==state->assignedCount());
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
void replacementAndGrowth(std::shared_ptr<posim_slam::SubmapRowStore> store={}) {
  Graph graph({{5},{{0},{1},{2},{3},{4}}},{},{},store);graph.replace({},factors());verify(graph);
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
  assert(graph.snapshot()->hasOriginal("pixel:b") && !graph.snapshot()->assignment("pixel:b"));
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
void rollbackAndConstraints(std::shared_ptr<posim_slam::SubmapRowStore> store={}) {
  Graph graph({{5},{{0},{1},{2},{3},{4}}},{},{},store);graph.replace({},factors());
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
  rejects([&]{graph.replace({}, {},{},&layout);},"submap_node_capacity");
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
void symbolicElimination() {
  // Cross packed-word boundaries and compare every weighted degree against an
  // independent set graph, including fill edges and nonuniform block sizes.
  constexpr size_t count=137;std::vector<size_t> weights(count);
  for(size_t i=0;i<count;++i)weights[i]=1+i%6;
  posim_slam::SubmapOrderingGraph packed(weights);std::vector<std::set<size_t>> reference(count);
  for(size_t i=0;i<count;++i)for(size_t j=i+1;j<count;++j)if((i*29+j*17)%41<3) {
    packed.connect(i,j);packed.connect(j,i);reference[i].insert(j);reference[j].insert(i);
  }
  for(size_t step=0;step<count;++step) {
    for(size_t i=0;i<count;++i) {
      size_t expected=0;for(auto j:reference[i])expected+=weights[j];
      assert(packed.degree(i)==expected);
    }
    const size_t node=(step*43)%count;size_t width=weights[node];
    for(auto other:reference[node])width+=weights[other];
    assert(packed.eliminate(node)==width);
    const auto neighbors=reference[node];
    for(auto a:neighbors) {
      reference[a].erase(node);
      for(auto b:neighbors)if(a!=b)reference[a].insert(b);
    }
    reference[node].clear();
  }
}
void affectedOrdering(std::shared_ptr<posim_slam::SubmapRowStore> store={}) {
  Graph::Layout layout;layout.frontier={15};for(int i=0;i<15;++i)layout.eliminated.push_back({i});
  for(int i=0;i<16;++i)layout.variables.push_back({i});
  Graph graph(layout,{},{},store);std::vector<Graph::Factor> inputs;
  for(int i=1;i<=12;++i)inputs.push_back(edge("leaf:"+std::to_string(i),"pixel:"+std::to_string(i),0,i,.1*i,1+.1*i));
  inputs.push_back(edge("center","pixel:center",0,15,.3,2));
  inputs.push_back(edge("branch:a","pixel:branch:a",13,14,.2,3));
  inputs.push_back(edge("branch:b","pixel:branch:b",14,15,.4,2));
  auto anchor=edge("anchor","sensor:anchor",15,14,0,1);
  anchor.rows.columns={15};anchor.rows.jacobian=Matrix::Constant(1,1,2);inputs.push_back(anchor);
  graph.replace({},inputs);verify(graph);
  const auto before=graph.snapshot();const auto old_solution=graph.solve(before);
  const auto old_query=graph.query(old_solution,{0,15},Matrix::Identity(2,2));
  auto update=edge("leaf:1","pixel:1",0,1,.7,3);
  int calls=0;
  rejects([&]{graph.replace({"leaf:1"},{update},{},nullptr,[&]{return ++calls==6;},Graph::Ordering::minimum_degree);},"submap_factor_cancelled");
  assert(graph.snapshot()==before);
  graph.replace({"leaf:1"},{update},{},nullptr,{},Graph::Ordering::minimum_degree);verify(graph);
  const auto after=graph.snapshot();assert(after->layout.eliminated!=before->layout.eliminated);
  for(auto context:{13,14}) {
    const auto found=std::find(after->layout.eliminated.begin(),after->layout.eliminated.end(),std::vector<int64_t>{context});
    assert(found!=after->layout.eliminated.end());
    assert(after->buckets[found-after->layout.eliminated.begin()]==before->buckets[context]);
  }
  auto width=[](const Graph::Snapshot& state) {
    size_t largest=0;for(const auto& bucket:state->buckets)
      largest=std::max(largest,bucket->context.size()+bucket->message->columns.size());
    return largest;
  };
  assert(width(after)<width(before));
  close(graph.query(old_solution,{0,15},Matrix::Identity(2,2)).mean,old_query.mean);
  close(graph.query(old_solution,{0,15},Matrix::Identity(2,2)).covariance,old_query.covariance);
  graph.replace({}, {},{},nullptr,{},Graph::Ordering::minimum_degree);
  assert(graph.snapshot()->buckets==after->buckets && graph.snapshot()->rebuilt.empty());
  auto grown=graph.snapshot()->layout;grown.eliminated.push_back({15});grown.frontier={16};grown.variables.push_back({16});
  graph.replace({}, {edge("new","pixel:new",15,16,.2,2)}, {}, &grown,{},Graph::Ordering::minimum_degree);verify(graph);
  graph.replace({"anchor"},{},{"sensor:anchor"},nullptr,{},Graph::Ordering::minimum_degree);verify(graph);
  Matrix relative=Matrix::Zero(2,17);relative(0,0)=relative(1,13)=1;relative(0,16)=relative(1,15)=-1;verify(graph,relative);
  anchor.rows.jacobian.resize(0,1);anchor.rows.residual.resize(0);
  anchor.rows.constraints=Matrix::Ones(1,1);anchor.rows.target=Vector::Constant(1,.2);
  graph.replace({}, {anchor},{},nullptr,{},Graph::Ordering::minimum_degree);verify(graph);
}
void strictSupernodes(std::shared_ptr<posim_slam::SubmapRowStore> store={}) {
  constexpr int n=12;Graph::Layout layout;layout.frontier={n-1};
  for(int i=0;i<n-1;++i)layout.eliminated.push_back({i});
  Graph graph(layout,{},{},store);Graph::Rows rows;rows.originals={"dense-measurement"};
  for(int i=0;i<n;++i)rows.columns.push_back(i);
  rows.jacobian=Matrix::Identity(n,n);rows.jacobian.array()+=.02;
  rows.residual=Vector::LinSpaced(n,-.2,.3);
  graph.replace({},{{"dense",rows}});verify(graph);
  const auto before=graph.snapshot();const auto old_solution=graph.solve(before);
  const auto old=graph.query(old_solution,rows.columns,Matrix::Identity(n,n));
  rows.residual*=2;
  graph.replace({"dense"},{{"dense",rows}},{},nullptr,{},Graph::Ordering::minimum_degree);verify(graph);
  const auto grouped=graph.snapshot();assert(grouped->layout.eliminated.size()==1);
  assert(grouped->layout.eliminated.front().size()==n-1 && grouped->factors.size()==1);
  close(graph.query(old_solution,rows.columns,Matrix::Identity(n,n)).covariance,old.covariance);
  graph.replace({}, {},{},nullptr,{},Graph::Ordering::minimum_degree);
  assert(graph.snapshot()->buckets==grouped->buckets && graph.snapshot()->rebuilt.empty());
  auto growth=grouped->layout;growth.eliminated.push_back({n-1});growth.frontier={n};
  graph.replace({}, {edge("later","later-source",n-1,n,.4,2)}, {}, &growth,{},Graph::Ordering::minimum_degree);verify(graph);
  // Removing absolute information keeps the relative measurement and its gauge.
  graph.replace({"dense"},{},{"dense-measurement"},nullptr,{},Graph::Ordering::minimum_degree);
  Matrix relative=Matrix::Zero(1,n+1);relative(0,n-1)=1;relative(0,n)=-1;verify(graph,relative);
  assert(!graph.query(graph.solve(graph.snapshot()),{n},Matrix::Identity(1,1)).observed);
  rows.jacobian.resize(0,n);rows.residual.resize(0);
  rows.constraints=Matrix::Identity(n,n);rows.target=Vector::LinSpaced(n,-.2,.3);
  graph.replace({},{{"exact",rows}},{},nullptr,{},Graph::Ordering::minimum_degree);verify(graph);
}
void regroupChangedSupernodes(std::shared_ptr<posim_slam::SubmapRowStore> store={}) {
  constexpr int n=20;Graph::Layout layout;layout.frontier={18,19};
  for(int i=0;i<n;i+=2) {
    layout.variables.push_back({i,i+1});
    if(i<18)layout.eliminated.push_back({i,i+1});
  }
  auto invalid=layout;invalid.variables.pop_back();
  rejects([&]{Graph bad(invalid);},"submap_factor_variables");
  invalid=layout;invalid.variables[0]={0,2};
  rejects([&]{Graph bad(invalid);},"submap_factor_variables");
  Graph graph(layout,{},{},store);Graph::Rows dense;
  for(int i=0;i<n;++i)dense.columns.push_back(i);
  for(int i=0;i<n/2;++i)dense.originals.push_back("measurement:"+std::to_string(i));
  dense.jacobian=Matrix::Identity(n,n);dense.jacobian.array()+=.02;
  dense.residual=Vector::LinSpaced(n,-.2,.3);
  graph.replace({},{{"dense",dense}},{},nullptr,{},Graph::Ordering::minimum_degree);verify(graph);
  const auto grouped=graph.snapshot();assert(grouped->layout.eliminated.size()==1);
  const auto old_solution=graph.solve(grouped);const auto old=graph.query(old_solution,dense.columns,Matrix::Identity(n,n));
  std::vector<Graph::Factor> chain;
  for(int i=0;i<n;i+=2) {
    Graph::Rows rows;rows.originals={"measurement:"+std::to_string(i/2)};
    rows.columns=i?std::vector<int64_t>{i-2,i-1,i,i+1}:std::vector<int64_t>{0,1};
    rows.jacobian=Matrix::Zero(2,rows.columns.size());rows.jacobian.rightCols(2).setIdentity();
    if(i)rows.jacobian.leftCols(2)=-Matrix::Identity(2,2);
    rows.residual=Vector::Constant(2,-.1*(i+1));chain.push_back({"chain:"+std::to_string(i),rows});
  }
  graph.replace({"dense"},chain,{},nullptr,{},Graph::Ordering::minimum_degree);verify(graph);
  assert(graph.snapshot()->layout.variables==layout.variables);
  assert(graph.snapshot()->layout.eliminated.size()==layout.eliminated.size());
  for(const auto& bucket:graph.snapshot()->buckets)
    assert(bucket->context.size()+bucket->message->columns.size()<=4);
  close(graph.query(old_solution,dense.columns,Matrix::Identity(n,n)).covariance,old.covariance);
  const auto unchanged=graph.snapshot();
  graph.replace({}, {},{},nullptr,{},Graph::Ordering::minimum_degree);
  assert(graph.snapshot()->buckets==unchanged->buckets && graph.snapshot()->rebuilt.empty());
  invalid=graph.snapshot()->layout;std::swap(invalid.variables[0],invalid.variables[1]);
  const auto valid=graph.snapshot();
  rejects([&]{graph.replace({}, {},{},&invalid);},"submap_factor_variables_changed");
  assert(graph.snapshot()==valid);
  // Optional expansion cannot turn an already valid layout into a quota failure.
  auto bounded=grouped->layout;Graph::Information::Limits limits;limits.nodes=1;
  Graph limited(bounded,limits);limited.replace({},{{"dense",dense}});
  limited.replace({"dense"},chain,{},nullptr,{},Graph::Ordering::minimum_degree);verify(limited);
  assert(limited.snapshot()->layout.eliminated.size()==1);
}
void groupedWeakDirections() {
  constexpr int n=48,strong=44,repeats=8;Matrix a=Matrix::Zero(n,n);
  for(int i=0;i<strong;++i)for(int k=0;k<strong;++k)a(i,k)=(i==k?2.:0.)+.02*std::sin(.13*i+.17*k);
  a(44,44)=1e-8;a(45,45)=2e-8;
  const Matrix h=a.topLeftCorner(strong,strong).transpose()*a.topLeftCorner(strong,strong);
  const Matrix covariance=h.ldlt().solve(Matrix::Identity(strong,strong))/repeats;
  Vector mean(n);for(int i=0;i<n;++i)mean[i]=.01*(i+1);mean.tail(2).setZero();
  std::vector<int64_t> columns(n);std::iota(columns.begin(),columns.end(),0);
  for(double scale:{1e-4,1.,1e4}) {
    Graph::Layout layout;layout.frontier={44,45,46,47};for(int i=0;i<strong;++i)layout.eliminated.push_back({i});
    Graph graph(layout);Graph::Rows rows;rows.originals={"strong-weak-free"};rows.columns=columns;rows.jacobian.resize(repeats*n,n);
    for(int i=0;i<repeats;++i)rows.jacobian.middleRows(i*n,n)=scale*a;
    rows.residual=-rows.jacobian*mean;
    graph.replace({},{{"likelihood",rows}},{},nullptr,{},Graph::Ordering::minimum_degree);
    assert(graph.snapshot()->layout.eliminated.size()==1);
    Matrix derivative=Matrix::Zero(8,n);std::vector<int> chosen{0,6,17,39,44,45,46,47};
    for(size_t i=0;i<chosen.size();++i)derivative(i,chosen[i])=1;
    const auto solved=graph.solve(graph.snapshot());const auto finite=graph.query(solved,columns,derivative.topRows(6));
    assert(finite.observed);assert((finite.mean-derivative.topRows(6)*mean).norm()<1e-8);
    Matrix expected=Matrix::Zero(4,4);for(int i=0;i<4;++i)for(int k=0;k<4;++k)expected(i,k)=covariance(chosen[i],chosen[k])/(scale*scale);
    close(finite.covariance.topLeftCorner(4,4),expected);
    Matrix weak=Matrix::Zero(2,2);weak(0,0)=1e16/(repeats*scale*scale);weak(1,1)=1e16/(4*repeats*scale*scale);
    assert((finite.covariance.bottomRightCorner(2,2)-weak).norm()<1e-10*weak.norm());
    const auto all=graph.query(solved,columns,derivative);Matrix free=Matrix::Zero(8,8);free.bottomRightCorner(2,2).setIdentity();
    assert(!all.observed);close(all.unbounded_gram,free);
  }
}
void sql(const std::filesystem::path& directory,const char* statement) {
  sqlite3* db=nullptr;assert(sqlite3_open((directory/"rows.sqlite").string().c_str(),&db)==SQLITE_OK);
  const auto result=sqlite3_exec(db,statement,nullptr,nullptr,nullptr);sqlite3_close(db);assert(result==SQLITE_OK);
}
Graph::Rows projectedRows(int rank) {
  constexpr int m=64,n=24;Matrix source=Matrix::Zero(m,n),nuisance=Matrix::Zero(m,3);
  Vector residual(m);
  for(int i=0;i<m;++i) {
    source(i,i%n)=std::sin(.3*(i+1));source(i,(5*i+3)%n)+=.03;
    residual[i]=std::cos(.17*(i+1));
    for(int k=0;k<rank;++k)nuisance(i,k)=std::sin(.21*(i+1)*(k+1));
  }
  source(0,n-1)=-0.;Matrix joint(m,n+3);joint<<source,nuisance;
  Eigen::HouseholderQR<Matrix> transform;
  const auto split=posim_slam::splitJointInformation(joint,residual,n,true,0.,&transform);
  Graph::Rows rows;rows.originals={"projected-original"};rows.columns.resize(n);
  std::iota(rows.columns.begin(),rows.columns.end(),0);
  rows.jacobian=split.reduced.jacobian;rows.residual=split.reduced.residual;
  rows.projection=posim_slam::SubmapRowProjection::create(source,residual,transform,
      split.reduced.nuisance_rank,rows.jacobian,rows.residual);
  assert(rows.projection && rows.retainedBytes()<rows.bytes());return rows;
}
void identicalRows(const Graph::Rows& a,const Graph::Rows& b) {
  assert(a.columns==b.columns && a.originals.empty());
  assert(a.jacobian.rows()==b.jacobian.rows() && a.jacobian.cols()==b.jacobian.cols());
  assert(!std::memcmp(a.jacobian.data(),b.jacobian.data(),a.jacobian.size()*sizeof(double)));
  assert(!std::memcmp(a.residual.data(),b.residual.data(),a.residual.size()*sizeof(double)));
}
void identicalOriginals(const posim_slam::SubmapRowPage::Reference& page,const Graph::Rows& rows) {
  std::vector<std::string> actual;page->eachOriginal([&](const auto& key){actual.push_back(key);});
  assert(actual==rows.originals && page->originalCount()==rows.originals.size());
}
void projectedStorage(const std::filesystem::path& root) {
  using Store=posim_slam::SubmapRowStore;using Projection=posim_slam::SubmapRowProjection;
  for(int rank=0;rank<=3;++rank) {
    const auto rows=projectedRows(rank);const auto resident=posim_slam::SubmapRowPage::resident(rows);
    assert(resident->bytes()==rows.retainedBytes() && resident->decodedBytes()==rows.bytes());
    identicalRows(*resident->pinNumeric(),rows);identicalOriginals(resident,rows);
    posim_slam::SubmapLinearization dense_owner;dense_owner.append({rows});
    assert(!dense_owner.snapshot()->active.front()->projection);
    auto malformed=rows.projection->payload();int64_t invalid=5000;std::memcpy(malformed.data(),&invalid,sizeof(invalid));
    rejects([&]{Projection::decode(malformed.data(),malformed.size(),rows.jacobian.rows(),rows.jacobian.cols(),rows.projection->expected());},"submap_projection_shape");
    auto altered=rows;altered.residual[0]+=1;
    rejects([&]{posim_slam::SubmapRowPage::resident(altered);},"submap_projection_source");
    const auto path=root/("projected-"+std::to_string(rank));auto store=std::make_shared<Store>(path,rows.bytes());
    auto first=store->put(rows),second=store->put(rows);auto lease=first->pinNumeric();identicalRows(*lease,rows);
    identicalOriginals(first,rows);identicalOriginals(second,rows);
    store->evict();assert(store->statistics().resident_bytes==rows.bytes());
    rejects([&]{second->pinNumeric();},"submap_row_cache_capacity");
    first.reset();identicalRows(*lease,rows);assert(store->statistics().pages==1);
    lease.reset();assert(store->statistics().resident_bytes==0);identicalRows(*second->pinNumeric(),rows);
    assert(store->statistics().peak_resident_bytes==rows.bytes());
    store->evict();sql(path,"UPDATE pages SET payload=zeroblob(length(payload))");
    rejects([&]{second->pinNumeric();},"submap_row_storage_digest");
  }
  const auto rows=projectedRows(3);const auto path=root/"projected-atomic";
  auto store=std::make_shared<Store>(path,65536);Graph::Layout layout;
  for(int i=0;i<18;++i)layout.eliminated.push_back({i});
  for(int i=18;i<24;++i)layout.frontier.push_back(i);
  Graph graph(layout,{},{},store);graph.replace({},{{"original",rows}});verify(graph);
  auto before=graph.snapshot();const auto pages=store->statistics().pages;
  auto changed=rows;changed.jacobian(0,0)+=.1;
  rejects([&]{graph.replace({"original"},{{"original",changed}});},"submap_projection_source");
  assert(graph.snapshot()==before && store->statistics().pages==pages);verify(graph);
  sql(path,"CREATE TRIGGER fail_insert BEFORE INSERT ON pages BEGIN SELECT RAISE(FAIL,'test'); END");
  rejects([&]{graph.replace({"original"},{{"original",rows}});},"submap_row_storage_io");
  assert(graph.snapshot()==before && store->statistics().pages==pages);verify(graph);
  sql(path,"DROP TRIGGER fail_insert");
}
void ownedPages(const std::filesystem::path& root) {
  using Store=posim_slam::SubmapRowStore;
  const auto path=root/"owned-index";auto store=std::make_shared<Store>(path,1024);
  Graph graph({{0},{}},{},{},store,Graph::Ownership::paged);
  Graph::Rows rows;rows.columns={0};rows.jacobian=Matrix::Constant(1,1,2);rows.residual=Vector::Constant(1,-6);
  // More originals than fit in the shared cache; preserve source-label order.
  for(size_t i=65537;i>0;--i)rows.originals.push_back("source:"+std::to_string(i));
  Graph resident({{0},{}});
  rejects([&]{resident.replace({},{{"large",rows}});},"submap_measurement_capacity");
  const auto first=graph.replace({},{{"large",rows}});assert(first->pagedOwnership());
  assert(first->originalCount()==65537 && first->assignedCount()==65537);
  assert(first->information->measurements.size()==65537 && first->base->measurements.empty());
  identicalOriginals(first->factors.at("large"),rows);
  auto solution=graph.solve(first);auto result=graph.query(solution,{0},Matrix::Identity(1,1));
  assert(result.observed && std::abs(result.mean[0]-3)<1e-12 && std::abs(result.covariance(0,0)-.25)<1e-12);
  auto changed=rows;const auto unused=changed.originals.back();changed.originals.pop_back();changed.originals.push_back("new-source");
  changed.jacobian(0,0)=4;changed.residual[0]=-8;
  auto rejected=changed;rejected.originals.pop_back();
  rejects([&]{graph.replace({"large"},{{"changed",rejected}});},"ownership_original_missing");
  assert(graph.snapshot()==first);store->metadata()->collect();
  const auto before_metadata=store->metadata()->statistics().pages,before_numeric=store->statistics().pages;
  int cancelled_calls=0;
  rejects([&]{graph.replace({"large"},{{"changed",changed}},{unused},nullptr,[&]{return ++cancelled_calls==5;});},"submap_factor_cancelled");
  store->metadata()->collect();
  assert(graph.snapshot()==first && store->statistics().pages==before_numeric && store->metadata()->statistics().pages==before_metadata);
  const auto trigger="CREATE TRIGGER fail_after_source BEFORE INSERT ON pages WHEN (SELECT count(*) FROM pages)>="+
      std::to_string(before_numeric+1)+" BEGIN SELECT RAISE(FAIL,'after ownership stage'); END";
  sql(path,trigger.c_str());
  rejects([&]{graph.replace({"large"},{{"changed",changed}},{unused});},"submap_row_storage_io");
  sql(path,"DROP TRIGGER fail_after_source");store->metadata()->collect();
  assert(graph.snapshot()==first && store->statistics().pages==before_numeric && store->metadata()->statistics().pages==before_metadata);
  const auto second=graph.replace({"large"},{{"changed",changed}},{unused});
  assert(second->originalCount()==65538 && second->assignedCount()==65537 && second->hasOriginal(unused) && !second->assignment(unused));
  std::vector<std::string> unassigned;second->eachUnassigned([&](const auto& key){unassigned.push_back(key);});assert(unassigned==std::vector<std::string>{unused});
  assert(first->assignment(unused)=="large" && first->information->measurements.count(unused));
  assert(!second->information->measurements.count(unused) && second->information->measurements.count("new-source"));
  auto next=graph.solve(second);auto now=graph.query(next,{0},Matrix::Identity(1,1));
  assert(now.observed && std::abs(now.mean[0]-2)<1e-12 && std::abs(now.covariance(0,0)-.0625)<1e-12);
  assert(store->metadata()->statistics().peak_entries<=65536 && store->metadata()->statistics().peak_cache_bytes<=8*1024*1024);
  // A view keeps the shared working directory alive after its row-store handle.
  auto view=second->information->measurements;assert(view.size()==65537);
}
void metadataLeases(const std::filesystem::path& root) {
  using Store=posim_slam::SubmapRowStore;posim_slam::SubmapOwnershipStore::Limits limits;
  limits.entries=3;limits.page_entries=2;
  auto store=std::make_shared<Store>(root/"metadata-leases",128,limits);auto meta=store->metadata();
  auto labels=meta->keys({"z","a"});auto held=meta->pin(labels->pages.front());meta->evict();
  auto rows=edge("a","a",0,1,.2,3).rows;rows.originals={"c","b"};auto page=store->put(rows);
  rejects([&]{page->eachOriginal([](const auto&){});},"ownership_cache_capacity");
  assert(meta->statistics().entries==2);held.reset();identicalOriginals(page,rows);
  auto inventory=meta->stage(meta->empty(),1,[](auto& edit){edit.factor("owner");edit.assign("a");edit.assign("z");});
  auto view=posim_slam::SubmapMeasurements::view(meta->measurements(inventory));
  rejects([&]{view.insert("forbidden");},"submap_measurement_view_read_only");
  page.reset();labels.reset();inventory.reset();meta.reset();store.reset();
  assert(view.size()==2 && view.count("z"));view={};
  assert(!std::filesystem::exists(root/"metadata-leases"));
}
void paged(const std::filesystem::path& root) {
  using Store=posim_slam::SubmapRowStore;
  assert(std::filesystem::create_directory(root));
  projectedStorage(root);
  ownedPages(root);metadataLeases(root);
  {
    auto store=std::make_shared<Store>(root/"graph",128);
    rejects([&]{Store duplicate(root/"graph",128);},"submap_row_storage_exists");
    replacementAndGrowth(store);rollbackAndConstraints(store);
    assert(store->statistics().pages==0 && store->statistics().resident_bytes==0);
    assert(store->statistics().reads>30 && store->statistics().peak_resident_bytes<=128);
    Graph graph({{5},{{0},{1},{2},{3},{4}}},{},{},store);graph.replace({},factors());
    const auto before=graph.snapshot();const auto count=store->statistics().pages;
    rejects([&]{graph.replace({"a"},{edge("a","pixel:a",0,2,.9,2),edge("bad","new:bad",0,99,0,1)});},"submap_factor_coordinates");
    assert(graph.snapshot()==before && store->statistics().pages==count);verify(graph);
    sql(root/"graph","CREATE TRIGGER fail_insert BEFORE INSERT ON pages BEGIN SELECT RAISE(FAIL,'test'); END");
    rejects([&]{graph.replace({"a"},{edge("a","pixel:a",0,2,.9,2)});},"submap_row_storage_io");
    assert(graph.snapshot()==before && store->statistics().pages==count);verify(graph);
    sql(root/"graph","DROP TRIGGER fail_insert");
    store->evict();sql(root/"graph","UPDATE pages SET payload=zeroblob(length(payload))");
    rejects([&]{graph.replace({"a"},{edge("a","pixel:a",0,2,.9,2)});},"submap_row_storage_digest");
    assert(graph.snapshot()==before && store->statistics().pages==count);
  }
  {
    auto store=std::make_shared<Store>(root/"ordering",4096);affectedOrdering(store);
    assert(store->statistics().pages==0 && store->statistics().resident_bytes==0);
  }
  {
    auto store=std::make_shared<Store>(root/"supernodes",4096);strictSupernodes(store);
    assert(store->statistics().pages==0 && store->statistics().resident_bytes==0);
  }
  {
    auto store=std::make_shared<Store>(root/"regroup",4096);regroupChangedSupernodes(store);
    assert(store->statistics().pages==0 && store->statistics().resident_bytes==0);
  }
  {
    auto store=std::make_shared<Store>(root/"leases",24);
    auto first=store->put(edge("a","a",0,1,.2,3).rows),second=store->put(edge("b","b",0,1,.3,4).rows);
    auto lease=first->pinNumeric();store->evict();assert(store->statistics().resident_bytes==24);
    rejects([&]{second->pinNumeric();},"submap_row_cache_capacity");
    first.reset();assert(lease->jacobian(0,0)==3 && store->statistics().pages==1);
    lease.reset();assert(store->statistics().resident_bytes==0);assert(second->pinNumeric()->jacobian(0,0)==4);
    // Empty and constant likelihoods preserve residuals without invented columns.
    Graph::Rows constant;constant.jacobian.resize(1,0);constant.residual=Vector::Constant(1,2);
    auto term=store->put(constant);assert(term->pinNumeric()->residual[0]==2);
    store.reset();assert(second->pinNumeric()->jacobian(0,0)==4);
  }
  {
    auto store=std::make_shared<Store>(root/"threads",128);
    std::vector<Store::Reference> pages;
    for(int i=0;i<12;++i)pages.push_back(store->put(edge("a","a",0,1,.2,1+i).rows));
    std::vector<std::thread> threads;
    for(int t=0;t<3;++t)threads.emplace_back([&,t]{for(int n=0;n<60;++n) {
      const size_t i=(n+t)%pages.size();const auto lease=pages[i]->pinNumeric();assert(lease->jacobian(0,0)==1+i);
    }});
    for(auto& thread:threads)thread.join();assert(store->statistics().peak_resident_bytes<=128);
    pages.clear();assert(store->statistics().pages==0 && store->statistics().resident_bytes==0);
  }
  assert(std::filesystem::is_empty(root));std::filesystem::remove(root);
}
} // namespace
int main(int argc,char** argv) {
  replacementAndGrowth();rollbackAndConstraints();quotasAndBatching();symbolicElimination();affectedOrdering();strictSupernodes();regroupChangedSupernodes();groupedWeakDirections();
  if(argc==2)paged(std::filesystem::path(argv[1]));else assert(argc==1);
  std::cout<<"source factorization: archived replacement, merge, unassignment, growth, shared branches, gauge, constraints and rollback passed\n";
}
