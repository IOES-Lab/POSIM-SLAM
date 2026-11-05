// SPDX-License-Identifier: Apache-2.0
#include "submap_row_store.hpp"
#include <sqlite3.h>
#include <cstring>

namespace posim_slam {
namespace {
struct Statement {
  sqlite3_stmt* value=nullptr;
  Statement(sqlite3* db,const char* sql) {
    if(sqlite3_prepare_v2(db,sql,-1,&value,nullptr)!=SQLITE_OK)
      throw std::runtime_error("submap_row_storage_io");
  }
  ~Statement(){sqlite3_finalize(value);}
  void number(int index,int64_t n) {
    if(sqlite3_bind_int64(value,index,n)!=SQLITE_OK)throw std::runtime_error("submap_row_storage_io");
  }
  void done(){if(sqlite3_step(value)!=SQLITE_DONE)throw std::runtime_error("submap_row_storage_io");}
};
InputDigest::Value digest(const std::array<int64_t,6>& shape,const void* bytes,size_t size) {
  InputDigest hash;hash.update(shape.data(),sizeof(shape));hash.update(bytes,size);return hash.finish();
}
}

SubmapRowPage::SubmapRowPage(const Rows& rows):originals(rows.originals),columns(rows.columns),bytes_(rows.bytes()),
    shape_{rows.jacobian.rows(),rows.jacobian.cols(),rows.constraints.rows(),rows.constraints.cols(),
           rows.residual.size(),rows.target.size()} {}
SubmapRowPage::Reference SubmapRowPage::resident(const Rows& rows) {
  auto page=std::shared_ptr<SubmapRowPage>(new SubmapRowPage(rows));
  page->resident_=std::make_shared<const Rows>(rows);return page;
}
SubmapRowPage::~SubmapRowPage(){if(store_ && id_)store_->release(id_);}
SubmapRowPage::Lease SubmapRowPage::pin() const {return resident_?resident_:store_->read(*this);}

SubmapRowStore::SubmapRowStore(const std::filesystem::path& directory,size_t cache_bytes)
    :directory_(directory),capacity_(cache_bytes) {
  if(!capacity_ || capacity_>96*1024*1024)throw std::invalid_argument("submap_row_cache_capacity");
  if(!std::filesystem::create_directory(directory_))throw std::invalid_argument("submap_row_storage_exists");
  try {
    if(sqlite3_open_v2((directory_/"rows.sqlite").string().c_str(),&db_,
        SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE|SQLITE_OPEN_FULLMUTEX,nullptr)!=SQLITE_OK)
      throw std::runtime_error("submap_row_storage_io");
    // Derived cache only: original inputs and durable checkpoints live elsewhere.
    // Never create a SQLite temporary file on an implicit/system temp disk.
    execute("PRAGMA temp_store=MEMORY");execute("PRAGMA journal_mode=MEMORY");
    execute("PRAGMA synchronous=OFF");execute("PRAGMA cache_size=-1024");
    execute("CREATE TABLE pages(id INTEGER PRIMARY KEY, payload BLOB NOT NULL)");
  }catch(...) {
    if(db_)sqlite3_close(db_);db_=nullptr;std::error_code error;
    std::filesystem::remove(directory_/"rows.sqlite",error);std::filesystem::remove(directory_,error);throw;
  }
}
SubmapRowStore::~SubmapRowStore() {
  if(db_)sqlite3_close(db_);std::error_code error;
  std::filesystem::remove(directory_/"rows.sqlite",error);
  std::filesystem::remove(directory_,error);
}
void SubmapRowStore::check() const {if(failed_)throw std::runtime_error("submap_row_storage_io");}
void SubmapRowStore::execute(const char* sql) {
  if(sqlite3_exec(db_,sql,nullptr,nullptr,nullptr)!=SQLITE_OK)throw std::runtime_error("submap_row_storage_io");
}
SubmapRowStore::Reference SubmapRowStore::put(const Rows& rows) {
  if(rows.columns.empty()) {
    if(rows.jacobian.cols() || rows.constraints.cols() || rows.jacobian.rows()>4096 ||
        rows.constraints.rows()>1024 || rows.residual.size()!=rows.jacobian.rows() ||
        rows.target.size()!=rows.constraints.rows() || !rows.residual.allFinite() || !rows.target.allFinite())
      throw std::invalid_argument("submap_information_rows");
  }else SubmapInformation::validateRows(rows.columns,rows.jacobian,rows.residual,rows.constraints,rows.target);
  if(rows.bytes()>capacity_)throw std::runtime_error("submap_row_cache_capacity");
  auto self=shared_from_this();auto page=std::shared_ptr<SubmapRowPage>(new SubmapRowPage(rows));
  std::vector<uint8_t> bytes(page->bytes());size_t offset=0;
  const auto append=[&](const auto& matrix) {
    const size_t size=sizeof(double)*matrix.size();
    if(size)std::memcpy(bytes.data()+offset,matrix.data(),size);offset+=size;
  };
  append(rows.jacobian);append(rows.constraints);append(rows.residual);append(rows.target);
  page->digest_=digest(page->shape_,bytes.data(),bytes.size());
  std::lock_guard<std::mutex> guard(mutex_);check();
  if(next_id_==INT64_MAX)throw std::runtime_error("submap_row_storage_capacity");
  Statement insert(db_,"INSERT INTO pages(id,payload) VALUES(?1,?2)");insert.number(1,next_id_);
  const int bound=bytes.empty()?sqlite3_bind_zeroblob(insert.value,2,0):
      sqlite3_bind_blob(insert.value,2,bytes.data(),int(bytes.size()),SQLITE_STATIC);
  if(bound!=SQLITE_OK)throw std::runtime_error("submap_row_storage_io");insert.done();
  page->id_=next_id_++;page->store_=std::move(self);++pages_;return page;
}
void SubmapRowStore::trim(size_t incoming) {
  while(incoming>capacity_-counter_->bytes.load() && !order_.empty()) {
    const auto id=order_.back();order_.pop_back();cache_.erase(id);
  }
  if(incoming>capacity_-counter_->bytes.load())throw std::runtime_error("submap_row_cache_capacity");
}
SubmapRowStore::Lease SubmapRowStore::read(const SubmapRowPage& page) {
  std::lock_guard<std::mutex> guard(mutex_);check();const auto found=cache_.find(page.id_);
  if(found!=cache_.end()) {
    order_.splice(order_.begin(),order_,found->second.position);return found->second.rows;
  }
  trim(page.bytes());Statement query(db_,"SELECT payload FROM pages WHERE id=?1");query.number(1,page.id_);
  if(sqlite3_step(query.value)!=SQLITE_ROW || sqlite3_column_type(query.value,0)!=SQLITE_BLOB ||
      size_t(sqlite3_column_bytes(query.value,0))!=page.bytes())throw std::runtime_error("submap_row_storage_shape");
  const auto bytes=static_cast<const uint8_t*>(sqlite3_column_blob(query.value,0));
  if(digest(page.shape_,bytes,page.bytes())!=page.digest_)throw std::runtime_error("submap_row_storage_digest");
  auto rows=std::make_unique<Rows>();rows->originals=page.originals;rows->columns=page.columns;
  rows->jacobian.resize(page.shape_[0],page.shape_[1]);rows->constraints.resize(page.shape_[2],page.shape_[3]);
  rows->residual.resize(page.shape_[4]);rows->target.resize(page.shape_[5]);size_t offset=0;
  const auto copy=[&](auto& matrix) {
    const size_t size=sizeof(double)*matrix.size();if(size)std::memcpy(matrix.data(),bytes+offset,size);offset+=size;
  };
  copy(rows->jacobian);copy(rows->constraints);copy(rows->residual);copy(rows->target);
  const auto counter=counter_;const auto size=page.bytes();counter->bytes+=size;
  Lease lease(rows.release(),[counter,size](const Rows* value){delete value;counter->bytes-=size;});
  peak_=std::max(peak_,counter->bytes.load());++reads_;order_.push_front(page.id_);
  try {cache_.emplace(page.id_,Cached{lease,order_.begin()});}
  catch(...){order_.pop_front();throw;}
  return lease;
}
void SubmapRowStore::release(int64_t id) noexcept {
  std::lock_guard<std::mutex> guard(mutex_);
  const auto found=cache_.find(id);
  if(found!=cache_.end()){order_.erase(found->second.position);cache_.erase(found);}
  try {Statement remove(db_,"DELETE FROM pages WHERE id=?1");remove.number(1,id);remove.done();--pages_;}
  catch(...){failed_=true;}
}
SubmapRowStore::Statistics SubmapRowStore::statistics() const {
  std::lock_guard<std::mutex> guard(mutex_);return {counter_->bytes.load(),peak_,pages_,reads_};
}
void SubmapRowStore::evict(){std::lock_guard<std::mutex> guard(mutex_);cache_.clear();order_.clear();}
} // namespace posim_slam
