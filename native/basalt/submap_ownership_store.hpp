// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "input_digest.hpp"
#include "submap_measurements.hpp"
#include <atomic>
#include <filesystem>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace posim_slam {
// Disposable derived ownership pages;
// original input/checkpoint archives remain authoritative. A stage never
// publishes the graph root. Numeric owners publish the returned immutable root
// only after their complete transaction succeeds.
class SubmapOwnershipStore {
  struct State;
 public:
  struct Limits {
    size_t entries=65536,cache_bytes=8*1024*1024,page_entries=256;
    size_t directory_bytes=8*1024*1024,workspace_bytes=1024*1024;
    size_t database_bytes=96*1024*1024;
  };
  struct Entry {
    std::string key,owner; // Empty owner denotes an explicitly unassigned source.
    bool operator==(const Entry& other) const {return key==other.key && owner==other.owner;}
  };
  using Lease=std::shared_ptr<const std::vector<Entry>>;
  struct Page {
    ~Page();
    int64_t id=0;
    size_t count=0,assigned=0,encoded_bytes=0,decoded_bytes=0;
    std::string first;
    InputDigest::Value digest{};
    bool ordered=true;
   private:
    friend class SubmapOwnershipStore;
    std::shared_ptr<State> state;
    size_t charged=0;
  };
  using Reference=std::shared_ptr<const Page>;
  struct Root {
    ~Root();
    uint64_t revision=0;
    size_t count=0,assigned=0;
    std::vector<Reference> pages;
   private:
    friend class SubmapOwnershipStore;
    std::shared_ptr<State> state;
    size_t charged=0;
    bool inventory=true;
  };
  using Snapshot=std::shared_ptr<const Root>;
  struct Statistics {
    size_t entries,peak_entries,cache_bytes,peak_cache_bytes;
    size_t directory_bytes,peak_directory_bytes,peak_workspace_bytes;
    size_t pages,payload_bytes,database_bytes,reads,transaction_entries;
  };
  class Editor {
   public:
    void remove(const std::string& owner);
    void factor(const std::string& owner);
    void assign(const std::string& key);
    void unassign(const std::string& key);
    void seed(const std::string& key);
   private:
    friend class SubmapOwnershipStore;
    Editor(State& state,bool empty,const std::function<bool()>& cancelled);
    void insert(const std::string& key,const std::string& owner,int kind);
    State& state_;
    bool empty_;
    std::string owner_;
    std::function<bool()> cancelled_;
    size_t staged_=0;
  };
  SubmapOwnershipStore(const std::filesystem::path& directory,Limits limits,
      std::shared_ptr<void> directory_owner={});
  size_t capacity() const;
  Snapshot empty() const;
  Snapshot stage(const Snapshot& previous,uint64_t revision,
      const std::function<void(Editor&)>& edit,const std::function<bool()>& cancelled={});
  Lease pin(const Reference& page) const;
  void each(const Snapshot& root,const std::function<void(const Entry&)>& visitor) const;
  std::optional<std::string> lookup(const Snapshot& root,const std::string& key) const;
  // Original ordering of a row's labels is preserved separately from the
  // sorted ownership index. Both use the same decoded lease and byte budget.
  Snapshot keys(const std::vector<std::string>& values,const std::function<bool()>& cancelled={});
  void eachKey(const Snapshot& keys,const std::function<void(const std::string&)>& visitor) const;
  std::shared_ptr<const SubmapMeasurements::Source> measurements(
      const Snapshot& root,const std::vector<std::string>& excluded_owners={}) const;
  void evict();
  void collect();
  Statistics statistics() const;
 private:
  std::shared_ptr<State> state_;
  void validate(const Snapshot& root) const;
  std::shared_ptr<Root> newRoot(uint64_t revision) const;
  void append(Root& root,Reference page) const;
  Reference put(const std::vector<Entry>& entries,size_t workspace,bool ordered=true);
};
} // namespace posim_slam
