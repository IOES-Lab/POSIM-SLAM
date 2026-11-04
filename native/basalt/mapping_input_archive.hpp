// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_frame_io.hpp"
#include <sqlite3.h>
#include <filesystem>

namespace posim_slam {
// SQLite owns transaction rollback and on-disk indexing. Frames and retirement
// batches are immutable; a failed commit publishes neither. The connection stays
// owned by frame references even after the producer has stopped.
class MappingInputArchive : public std::enable_shared_from_this<MappingInputArchive> {
 public:
  using Json=nlohmann::json;using Bytes=std::vector<uint8_t>;
  struct Page {int64_t stamp;Bytes bytes;Json identity;};
  MappingInputArchive(const std::filesystem::path& path,const Json& metadata,bool restore);
  ~MappingInputArchive();
  MappingInputArchive(const MappingInputArchive&)=delete;
  MappingInputArchive& operator=(const MappingInputArchive&)=delete;
  void commit(uint64_t revision,const Json& batch,const std::vector<Page>& pages);
  void replay(size_t maximum,const std::function<void(const Json&)>& consume);
  MappingFrameReference reference(const Json& identity,uint64_t revision);
  static Page page(const MappingInputFrame& frame);
 private:
  struct Statement {
    sqlite3_stmt* value=nullptr;
    Statement(sqlite3* db,const char* sql);
    ~Statement(){sqlite3_finalize(value);}
    Statement(const Statement&)=delete;
    void number(int index,int64_t value);
    void blob(int index,const Bytes& value);
    bool row();void done();
    Bytes bytes(int column,size_t maximum);
  };
  void execute(const char* sql);
  uint64_t latest();
  MappingFrameReference::Lease read(const Json& identity,uint64_t revision,size_t maximum);
  sqlite3* db_=nullptr;std::mutex mutex_;
};
} // namespace posim_slam
