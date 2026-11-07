// SPDX-License-Identifier: Apache-2.0
#include "mapping_input_archive.hpp"

namespace posim_slam {
namespace {
void check(int code,int expected=SQLITE_OK) {
  if(code!=expected)throw std::runtime_error("mapping_input_storage:"+std::to_string(code));
}
}
MappingFrameReference::Lease MappingFrameReference::read(size_t maximum) const {
  if(resident_) {
    if(maximum!=SIZE_MAX && mapping_frame_io::encode(*resident_).size()>maximum)
      throw std::runtime_error("mapping_input_page_budget");return resident_;
  }
  if(!reader_)throw std::invalid_argument("mapping_input_frame_missing");return reader_(maximum);
}
MappingInputArchive::Statement::Statement(sqlite3* db,const char* sql) {
  const auto code=sqlite3_prepare_v2(db,sql,-1,&value,nullptr);
  if(code!=SQLITE_OK){sqlite3_finalize(value);value=nullptr;check(code);}
}
void MappingInputArchive::Statement::number(int index,int64_t value) {check(sqlite3_bind_int64(this->value,index,value));}
void MappingInputArchive::Statement::blob(int index,const Bytes& value) {
  check(sqlite3_bind_blob64(this->value,index,value.data(),value.size(),SQLITE_TRANSIENT));
}
bool MappingInputArchive::Statement::row() {
  const auto code=sqlite3_step(value);if(code==SQLITE_ROW)return true;check(code,SQLITE_DONE);return false;
}
void MappingInputArchive::Statement::done(){check(sqlite3_step(value),SQLITE_DONE);}
MappingInputArchive::Bytes MappingInputArchive::Statement::bytes(int column,size_t maximum) {
  const int count=sqlite3_column_bytes(value,column);
  if(sqlite3_column_type(value,column)!=SQLITE_BLOB || count<=0 || size_t(count)>maximum)
    throw std::invalid_argument("mapping_input_storage_shape");
  const auto* data=static_cast<const uint8_t*>(sqlite3_column_blob(value,column));
  if(!data)throw std::runtime_error("mapping_input_storage_read");return Bytes(data,data+count);
}
void MappingInputArchive::execute(const char* sql){check(sqlite3_exec(db_,sql,nullptr,nullptr,nullptr));}
MappingInputArchive::MappingInputArchive(const std::filesystem::path& path,const Json& metadata,bool restore) {
  // An explicit session-owned path is required. Never fall back to /tmp, the
  // user's home or a different volume when the selected volume is unavailable.
  if(!path.is_absolute() || path.filename().empty() || std::filesystem::is_symlink(path) ||
      !std::filesystem::is_directory(path.parent_path()) || (!restore && std::filesystem::exists(path)))
    throw std::invalid_argument("mapping_input_storage_path");
  try {
    check(sqlite3_open_v2(path.string().c_str(),&db_,SQLITE_OPEN_READWRITE|
      (restore?0:SQLITE_OPEN_CREATE)|SQLITE_OPEN_FULLMUTEX,nullptr));
    check(sqlite3_busy_timeout(db_,5000));
    sqlite3_limit(db_,SQLITE_LIMIT_LENGTH,96*1024*1024);
    if(restore) {
      // Inspect identity before changing journal settings on a supplied file.
      Statement statement(db_,"SELECT value FROM identity WHERE id=1");
      if(!statement.row() || statement.bytes(0,1024*1024)!=Json::to_cbor(metadata))
        throw std::invalid_argument("mapping_input_storage_identity");
    }
    execute("PRAGMA foreign_keys=ON; PRAGMA journal_mode=DELETE; PRAGMA synchronous=FULL;"
            "PRAGMA fullfsync=ON; PRAGMA temp_store=MEMORY; PRAGMA mmap_size=0; PRAGMA cache_size=-2048;");
    if(!restore) {
      execute("PRAGMA page_size=4096; PRAGMA max_page_count=24576; BEGIN IMMEDIATE;"
        "CREATE TABLE identity(id INTEGER PRIMARY KEY CHECK(id=1), value BLOB NOT NULL);"
        "CREATE TABLE batches(revision INTEGER PRIMARY KEY CHECK(revision>0), digest BLOB NOT NULL, value BLOB NOT NULL);"
        "CREATE TABLE frames(stamp INTEGER PRIMARY KEY CHECK(stamp>0), revision INTEGER NOT NULL REFERENCES batches(revision) DEFERRABLE INITIALLY DEFERRED, value BLOB NOT NULL);");
      Statement statement(db_,"INSERT INTO identity VALUES(1,?)");statement.blob(1,Json::to_cbor(metadata));statement.done();
      execute("COMMIT");
    } else {
      execute("PRAGMA max_page_count=24576;");
    }
  } catch(...) {if(db_){sqlite3_exec(db_,"ROLLBACK",nullptr,nullptr,nullptr);sqlite3_close(db_);db_=nullptr;}throw;}
}
MappingInputArchive::~MappingInputArchive(){if(db_)sqlite3_close(db_);}
uint64_t MappingInputArchive::latest() {
  Statement query(db_,"SELECT coalesce(max(revision),0) FROM batches");
  if(!query.row())throw std::runtime_error("mapping_input_storage_read");return sqlite3_column_int64(query.value,0);
}
MappingInputArchive::Page MappingInputArchive::page(const MappingInputFrame& frame) {
  using namespace mapping_frame_io;
  auto bytes=encode(frame);
  Json identity={frame.stamp,frame.width,frame.height,frame.features.corners.size(),binary(frame.digest),bytes.size(),binary(digest(bytes))};
  return {frame.stamp,std::move(bytes),std::move(identity)};
}
void MappingInputArchive::commit(uint64_t revision,const Json& batch,const std::vector<Page>& pages) {
  const auto bytes=Json::to_cbor(batch);const auto digest=mapping_frame_io::digest(bytes);
  std::lock_guard<std::mutex> lock(mutex_);
  execute("BEGIN IMMEDIATE");
  try {
    if(!revision || latest()+1!=revision)throw std::invalid_argument("mapping_input_storage_revision");
    for(const auto& page:pages) {
      Statement statement(db_,"INSERT INTO frames VALUES(?,?,?)");
      statement.number(1,page.stamp);statement.number(2,revision);statement.blob(3,page.bytes);statement.done();
    }
    Statement statement(db_,"INSERT INTO batches VALUES(?,?,?)");
    statement.number(1,revision);statement.blob(2,Bytes(digest.begin(),digest.end()));statement.blob(3,bytes);statement.done();
    execute("COMMIT");
  } catch(...) {sqlite3_exec(db_,"ROLLBACK",nullptr,nullptr,nullptr);throw;}
}
void MappingInputArchive::replay(size_t maximum,const std::function<void(const Json&)>& consume) {
  std::lock_guard<std::mutex> lock(mutex_);
  Statement query(db_,"SELECT revision,digest,value FROM batches ORDER BY revision");
  size_t count=0,total=0;
  while(query.row()) {
    if(count>=maximum || sqlite3_column_int64(query.value,0)!=int64_t(count+1))
      throw std::invalid_argument("mapping_input_storage_revision");
    const auto bytes=query.bytes(2,96*1024*1024-total);total+=bytes.size();
    const auto digest=mapping_frame_io::digest(bytes);
    if(query.bytes(1,32)!=Bytes(digest.begin(),digest.end()))throw std::invalid_argument("mapping_input_batch_digest");
    consume(Json::from_cbor(bytes));++count;
  }
}
MappingFrameReference MappingInputArchive::reference(const Json& identity,uint64_t revision) {
  using namespace mapping_binary;
  if(!identity.is_array() || identity.size()!=7 || !identity[4].is_binary() || identity[4].get_binary().size()!=32 ||
      !identity[6].is_binary() || identity[6].get_binary().size()!=32)
    throw std::invalid_argument("mapping_input_frame_identity");
  MappingFrameReference::Metadata metadata{integer(identity[0],1,INT64_MAX),int(integer(identity[1],1,1920)),
    int(integer(identity[2],1,1080)),size_t(integer(identity[3],0,1000000)),{}};
  std::copy(identity[4].get_binary().begin(),identity[4].get_binary().end(),metadata.digest.begin());
  integer(identity[5],1,96*1024*1024);
  return {metadata,[self=shared_from_this(),identity,revision](size_t maximum){return self->read(identity,revision,maximum);}};
}
MappingFrameReference::Lease MappingInputArchive::read(const Json& identity,uint64_t revision,size_t maximum) {
  using namespace mapping_frame_io;
  const size_t length=identity[5];if(length>maximum)throw std::runtime_error("mapping_input_page_budget");
  Bytes bytes;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Statement query(db_,"SELECT revision,length(value),value FROM frames WHERE stamp=?");query.number(1,identity[0]);
    if(!query.row() || sqlite3_column_int64(query.value,0)!=int64_t(revision) || sqlite3_column_int64(query.value,1)!=int64_t(length))
      throw std::invalid_argument("mapping_input_frame_identity");
    bytes=query.bytes(2,length);
  }
  if(binary(digest(bytes))!=identity[6])throw std::invalid_argument("mapping_input_page_digest");
  Json expected=Json::array();for(int i=0;i<5;++i)expected.push_back(identity[i]);return decode(bytes,expected);
}
} // namespace posim_slam
