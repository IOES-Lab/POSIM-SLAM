// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <functional>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

namespace posim_slam {
// A complete measurement inventory. Resident information workspaces may mutate
// their set. A factor owner installs an immutable paged view after staging all
// source assignments; generic append/erase must not silently materialize it.
class SubmapMeasurements {
 public:
  struct Source {
    virtual ~Source()=default;
    virtual size_t size() const=0;
    virtual bool contains(const std::string& key) const=0;
    virtual void each(const std::function<void(const std::string&)>& visitor) const=0;
  };
  static SubmapMeasurements view(std::shared_ptr<const Source> source) {
    if(!source)throw std::invalid_argument("submap_measurement_view");
    SubmapMeasurements result;result.source_=std::move(source);return result;
  }
  size_t size() const {return source_?source_->size():resident_.size();}
  bool empty() const {return size()==0;}
  size_t count(const std::string& key) const {return source_?size_t(source_->contains(key)):resident_.count(key);}
  bool paged() const {return bool(source_);}
  void insert(const std::string& key){writable();resident_.insert(key);}
  template<class Iterator> void insert(Iterator begin,Iterator end){writable();resident_.insert(begin,end);}
  void erase(const std::string& key){writable();resident_.erase(key);}
  void each(const std::function<void(const std::string&)>& visitor) const {
    if(source_)source_->each(visitor);else for(const auto& key:resident_)visitor(key);
  }
  bool operator==(const SubmapMeasurements& other) const {
    if(source_ && source_==other.source_)return true;
    if(size()!=other.size())return false;bool equal=true;
    each([&](const auto& key){if(!other.count(key))equal=false;});return equal;
  }
  bool operator!=(const SubmapMeasurements& other) const {return !(*this==other);}
  bool operator==(const std::set<std::string>& other) const {
    if(size()!=other.size())return false;auto cursor=other.begin();bool equal=true;
    each([&](const auto& key){if(cursor==other.end() || *cursor!=key)equal=false;if(cursor!=other.end())++cursor;});return equal;
  }
  friend bool operator==(const std::set<std::string>& a,const SubmapMeasurements& b){return b==a;}
 private:
  void writable() const {if(source_)throw std::logic_error("submap_measurement_view_read_only");}
  std::set<std::string> resident_;
  std::shared_ptr<const Source> source_;
};
} // namespace posim_slam
