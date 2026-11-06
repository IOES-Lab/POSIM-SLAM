// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "submap_linearization.hpp"
#include "input_digest.hpp"
#include <filesystem>
#include <list>

struct sqlite3;
namespace posim_slam {
class SubmapRowStore;
// Immutable likelihood metadata stays resident. Numeric rows are either owned
// directly or leased from a bounded cache. A lease counts until its last reader
// releases it, including after eviction from the cache index.
class SubmapRowPage {
 public:
  using Rows=SubmapLinearization::Rows;
  using Lease=std::shared_ptr<const Rows>;
  using Reference=std::shared_ptr<const SubmapRowPage>;
  const std::vector<std::string> originals;
  const std::vector<SubmapInformation::Coordinate> columns;
  static Reference resident(const Rows& rows);
  ~SubmapRowPage();
  SubmapRowPage(const SubmapRowPage&)=delete;
  SubmapRowPage& operator=(const SubmapRowPage&)=delete;
  Lease pin() const;
  size_t bytes() const {return bytes_;}
  size_t decodedBytes() const {return decoded_bytes_;}
  Eigen::Index rows() const {return shape_[0];}
  Eigen::Index exactRows() const {return shape_[2];}
 private:
  friend class SubmapRowStore;
  explicit SubmapRowPage(const Rows& rows);
  std::unique_ptr<Rows> decode(const void* bytes) const;
  size_t bytes_,decoded_bytes_;
  std::array<int64_t,6> shape_;
  InputDigest::Value digest_{};
  InputDigest::Value projected_digest_{};
  bool projected_=false;
  Lease resident_;
  SubmapRowProjection::Ptr projection_;
  std::shared_ptr<SubmapRowStore> store_;
  int64_t id_=0;
};

// A disposable, process-local cache of derived rows, not an input/checkpoint
// archive. The caller chooses an available working disk and a new directory.
// SQLite reuses released pages; snapshots keep their rows alive independently.
// Corrupt/unavailable storage and pinned-cache exhaustion fail explicitly.
class SubmapRowStore : public std::enable_shared_from_this<SubmapRowStore> {
 public:
  using Rows=SubmapRowPage::Rows;
  using Lease=SubmapRowPage::Lease;
  using Reference=SubmapRowPage::Reference;
  struct Statistics {size_t resident_bytes,peak_resident_bytes,pages,reads;};
  SubmapRowStore(const std::filesystem::path& directory,size_t cache_bytes);
  ~SubmapRowStore();
  SubmapRowStore(const SubmapRowStore&)=delete;
  SubmapRowStore& operator=(const SubmapRowStore&)=delete;
  Reference put(const Rows& rows);
  Statistics statistics() const;
  void evict();
 private:
  friend class SubmapRowPage;
  struct Counter {std::atomic<size_t> bytes{0};};
  struct Cached {Lease rows;std::list<int64_t>::iterator position;};
  Lease read(const SubmapRowPage& page);
  void release(int64_t id) noexcept;
  void trim(size_t incoming);
  void check() const;
  void execute(const char* sql);
  std::filesystem::path directory_;
  size_t capacity_,peak_=0,pages_=0,reads_=0;
  int64_t next_id_=1;
  bool failed_=false;
  sqlite3* db_=nullptr;
  std::shared_ptr<Counter> counter_=std::make_shared<Counter>();
  std::list<int64_t> order_;
  std::map<int64_t,Cached> cache_;
  mutable std::mutex mutex_;
};
} // namespace posim_slam
