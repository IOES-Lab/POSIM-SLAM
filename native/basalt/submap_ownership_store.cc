// SPDX-License-Identifier: Apache-2.0
#include "submap_ownership_store.hpp"
#include <sqlite3.h>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace posim_slam {
namespace {
void fail(const char* message){throw std::runtime_error(message);}
void key(const std::string& value) {
  if(value.empty() || value.size()>256 || value.find('\0')!=std::string::npos)
    throw std::invalid_argument("ownership_identity");
}
void cancel(const std::function<bool()>& callback){if(callback && callback())fail("ownership_cancelled");}
struct Statement {
  sqlite3_stmt* value=nullptr;
  Statement(sqlite3* db,const char* sql) {
    if(sqlite3_prepare_v2(db,sql,-1,&value,nullptr)!=SQLITE_OK)fail("ownership_storage_io");
  }
  ~Statement(){sqlite3_finalize(value);}
  void reset() {
    if(sqlite3_reset(value)!=SQLITE_OK || sqlite3_clear_bindings(value)!=SQLITE_OK)
      fail("ownership_storage_io");
  }
  void text(int index,const std::string& item) {
    if(sqlite3_bind_text(value,index,item.data(),int(item.size()),SQLITE_TRANSIENT)!=SQLITE_OK)
      fail("ownership_storage_io");
  }
  void number(int index,int64_t number) {
    if(sqlite3_bind_int64(value,index,number)!=SQLITE_OK)fail("ownership_storage_io");
  }
  bool row() {const int code=sqlite3_step(value);if(code==SQLITE_ROW)return true;
    if(code!=SQLITE_DONE)fail("ownership_storage_io");return false;}
  void done(const char* duplicate=nullptr) {
    const int code=sqlite3_step(value);
    if(code==SQLITE_CONSTRAINT && duplicate)throw std::invalid_argument(duplicate);
    if(code==SQLITE_FULL)fail("ownership_database_capacity");
    if(code!=SQLITE_DONE)fail("ownership_storage_io");
  }
  int64_t number(int index) const {return sqlite3_column_int64(value,index);}
  std::string text(int index) const {
    const auto bytes=sqlite3_column_text(value,index);const int size=sqlite3_column_bytes(value,index);
    if(size && !bytes)fail("ownership_storage_shape");
    return bytes?std::string(reinterpret_cast<const char*>(bytes),size):std::string();
  }
};
size_t decoded(const std::vector<SubmapOwnershipStore::Entry>& rows) {
  size_t bytes=sizeof(rows)+rows.capacity()*sizeof(SubmapOwnershipStore::Entry);
  for(const auto& row:rows)bytes+=row.key.capacity()+row.owner.capacity()+2;
  return bytes;
}
void integer(std::vector<uint8_t>& out,size_t value) {
  if(value>65535)fail("ownership_storage_shape");
  out.push_back(uint8_t(value));out.push_back(uint8_t(value>>8));
}
std::vector<uint8_t> encode(const std::vector<SubmapOwnershipStore::Entry>& rows) {
  size_t count=2;for(const auto& row:rows)count+=4+row.key.size()+row.owner.size();
  std::vector<uint8_t> out;out.reserve(count);integer(out,rows.size());
  for(const auto& row:rows) {
    integer(out,row.key.size());integer(out,row.owner.size());
    out.insert(out.end(),row.key.begin(),row.key.end());out.insert(out.end(),row.owner.begin(),row.owner.end());
  }
  return out;
}
InputDigest::Value digest(const void* bytes,size_t size){InputDigest hash;hash.update(bytes,size);return hash.finish();}
}

struct SubmapOwnershipStore::State {
  struct Counter {std::atomic<size_t> entries{0},bytes{0},directory{0};};
  struct Cached {Lease value;std::list<int64_t>::iterator position;};
  Limits limits;
  std::shared_ptr<void> directory_owner;
  std::filesystem::path directory;
  sqlite3* db=nullptr;
  std::shared_ptr<Counter> counter=std::make_shared<Counter>();
  mutable std::recursive_mutex mutex;
  std::mutex garbage_mutex;
  std::vector<int64_t> garbage;
  std::map<int64_t,Cached> cache;
  std::list<int64_t> order;
  size_t peak_entries=0,peak_bytes=0,peak_directory=0,peak_workspace=0,reads=0,transaction_entries=0;
  int64_t next_id=1;
  std::atomic<bool> failed{false};
  bool in_transaction=false;
  State(const std::filesystem::path& path,Limits value,std::shared_ptr<void> owner)
      :limits(value),directory_owner(std::move(owner)),directory(path) {
    if(!limits.entries || limits.entries>65536 || !limits.page_entries || limits.page_entries>256 ||
        limits.page_entries>limits.entries || !limits.cache_bytes || limits.cache_bytes>8*1024*1024 ||
        !limits.directory_bytes || limits.directory_bytes>8*1024*1024 || !limits.workspace_bytes ||
        limits.workspace_bytes>1024*1024 || limits.database_bytes<128*1024 || limits.database_bytes>96*1024*1024)
      throw std::invalid_argument("ownership_limits");
    if(!std::filesystem::create_directory(directory))throw std::invalid_argument("ownership_storage_exists");
    try {
      if(sqlite3_open_v2((directory/"ownership.sqlite").string().c_str(),&db,
          SQLITE_OPEN_READWRITE|SQLITE_OPEN_CREATE|SQLITE_OPEN_FULLMUTEX,nullptr)!=SQLITE_OK)
        fail("ownership_storage_io");
      execute("PRAGMA page_size=4096");execute("PRAGMA temp_store=MEMORY");execute("PRAGMA journal_mode=DELETE");
      execute("PRAGMA synchronous=FULL");execute("PRAGMA cache_size=-1024");execute("PRAGMA mmap_size=0");
      // Main database is bounded. The rollback journal is on this same explicit
      // working disk, at most another database plus SQLite sector/header overhead.
      execute(("PRAGMA max_page_count="+std::to_string(limits.database_bytes/4096)).c_str());
      execute("CREATE TABLE pages(id INTEGER PRIMARY KEY,payload BLOB NOT NULL)");
      execute("CREATE TABLE changes(key TEXT PRIMARY KEY,owner TEXT,kind INTEGER NOT NULL)");
      execute("CREATE TABLE removals(owner TEXT PRIMARY KEY,seen INTEGER NOT NULL DEFAULT 0)");
      execute("CREATE TABLE incoming(owner TEXT PRIMARY KEY,n INTEGER NOT NULL DEFAULT 0)");
    }catch(...) {close();throw;}
  }
  ~State(){close();}
  void close() noexcept {
    if(db)sqlite3_close(db);db=nullptr;std::error_code error;
    std::filesystem::remove(directory/"ownership.sqlite-journal",error);
    std::filesystem::remove(directory/"ownership.sqlite",error);std::filesystem::remove(directory,error);
  }
  void check() const {if(failed)fail("ownership_storage_io");}
  void execute(const char* sql) {
    const int code=sqlite3_exec(db,sql,nullptr,nullptr,nullptr);
    if(code==SQLITE_FULL)fail("ownership_database_capacity");
    if(code!=SQLITE_OK)fail("ownership_storage_io");
  }
  void reserveDirectory(size_t bytes) {
    if(bytes>limits.directory_bytes-counter->directory.load())fail("ownership_directory_capacity");
    counter->directory+=bytes;peak_directory=std::max(peak_directory,counter->directory.load());
  }
  void workspace(size_t bytes) {
    if(bytes>limits.workspace_bytes)fail("ownership_workspace_capacity");
    peak_workspace=std::max(peak_workspace,bytes);
  }
  void trim(size_t entries,size_t bytes) {
    while((entries>limits.entries-counter->entries.load() || bytes>limits.cache_bytes-counter->bytes.load()) && !order.empty()) {
      const auto id=order.back();order.pop_back();cache.erase(id);
    }
    if(entries>limits.entries-counter->entries.load() || bytes>limits.cache_bytes-counter->bytes.load())
      fail("ownership_cache_capacity");
  }
  void retire(int64_t id) noexcept {
    try{std::lock_guard<std::mutex> lock(garbage_mutex);garbage.push_back(id);}
    catch(...){failed=true;}
  }
  void collect() {
    check();if(in_transaction)throw std::logic_error("ownership_collect_in_transaction");
    std::vector<int64_t> ids;
    {std::lock_guard<std::mutex> lock(garbage_mutex);ids.swap(garbage);}
    if(ids.empty())return;
    try {
      execute("BEGIN IMMEDIATE");
      for(auto id:ids) {
        auto found=cache.find(id);if(found!=cache.end()){order.erase(found->second.position);cache.erase(found);}
        Statement remove(db,"DELETE FROM pages WHERE id=?1");remove.number(1,id);remove.done();
      }
      execute("COMMIT");
    }catch(...){sqlite3_exec(db,"ROLLBACK",nullptr,nullptr,nullptr);failed=true;throw;}
  }
};

SubmapOwnershipStore::Page::~Page(){if(state){if(id)state->retire(id);state->counter->directory-=charged;}}
SubmapOwnershipStore::Root::~Root(){if(state)state->counter->directory-=charged;}
SubmapOwnershipStore::SubmapOwnershipStore(const std::filesystem::path& directory,Limits limits,std::shared_ptr<void> owner)
    :state_(std::make_shared<State>(directory,limits,std::move(owner))) {}
size_t SubmapOwnershipStore::capacity() const {return state_->limits.entries;}
void SubmapOwnershipStore::validate(const Snapshot& root) const {
  if(!root || root->state!=state_ || !root->inventory)throw std::invalid_argument("ownership_root_owner");
}
std::shared_ptr<SubmapOwnershipStore::Root> SubmapOwnershipStore::newRoot(uint64_t revision) const {
  auto root=std::make_shared<Root>();state_->reserveDirectory(sizeof(Root)+32);
  root->charged=sizeof(Root)+32;root->state=state_;root->revision=revision;return root;
}
SubmapOwnershipStore::Snapshot SubmapOwnershipStore::empty() const {
  std::lock_guard<std::recursive_mutex> lock(state_->mutex);state_->check();return newRoot(0);
}
void SubmapOwnershipStore::append(Root& root,Reference page) const {
  if(root.pages.size()==root.pages.capacity()) {
    const size_t capacity=std::max(size_t(1),root.pages.capacity()*2);
    const size_t bytes=(capacity-root.pages.capacity())*sizeof(Reference);
    state_->reserveDirectory(bytes);
    try{root.pages.reserve(capacity);}catch(...){state_->counter->directory-=bytes;throw;}
    root.charged+=bytes;
  }
  root.count+=page->count;root.assigned+=page->assigned;root.pages.push_back(std::move(page));
}
SubmapOwnershipStore::Reference SubmapOwnershipStore::put(const std::vector<Entry>& entries,size_t workspace,bool ordered) {
  if(entries.empty() || entries.size()>state_->limits.page_entries)fail("ownership_storage_shape");
  auto payload=encode(entries);state_->workspace(workspace+decoded(entries)+payload.capacity());
  auto page=std::make_shared<Page>();page->first=entries.front().key;page->ordered=ordered;
  state_->reserveDirectory(sizeof(Page)+32+page->first.capacity()+1);
  page->charged=sizeof(Page)+32+page->first.capacity()+1;page->state=state_;
  page->count=entries.size();page->encoded_bytes=payload.size();
  // Decoding reserves exactly count elements; string capacities are verified
  // after decoding and accounted using their actual allocations.
  page->decoded_bytes=sizeof(std::vector<Entry>)+entries.size()*sizeof(Entry);
  for(const auto& row:entries){page->assigned+=!row.owner.empty();page->decoded_bytes+=row.key.capacity()+row.owner.capacity()+2;}
  page->digest=digest(payload.data(),payload.size());
  if(state_->next_id==INT64_MAX)fail("ownership_storage_capacity");
  page->id=state_->next_id++;
  Statement insert(state_->db,"INSERT INTO pages VALUES(?1,?2)");insert.number(1,page->id);
  if(sqlite3_bind_blob(insert.value,2,payload.data(),int(payload.size()),SQLITE_STATIC)!=SQLITE_OK)fail("ownership_storage_io");
  insert.done();return page;
}
SubmapOwnershipStore::Lease SubmapOwnershipStore::pin(const Reference& page) const {
  if(!page || page->state!=state_)throw std::invalid_argument("ownership_page_owner");
  auto& s=*state_;std::lock_guard<std::recursive_mutex> lock(s.mutex);s.check();
  auto found=s.cache.find(page->id);
  if(found!=s.cache.end()){s.order.splice(s.order.begin(),s.order,found->second.position);return found->second.value;}
  s.trim(page->count,page->decoded_bytes);
  Statement query(s.db,"SELECT payload FROM pages WHERE id=?1");query.number(1,page->id);
  if(!query.row() || sqlite3_column_type(query.value,0)!=SQLITE_BLOB ||
      size_t(sqlite3_column_bytes(query.value,0))!=page->encoded_bytes)fail("ownership_storage_shape");
  auto bytes=static_cast<const uint8_t*>(sqlite3_column_blob(query.value,0));const auto end=bytes+page->encoded_bytes;
  if(digest(bytes,page->encoded_bytes)!=page->digest)fail("ownership_storage_digest");
  const auto number=[&]() {
    if(end-bytes<2)fail("ownership_storage_shape");
    const size_t value=size_t(bytes[0])|(size_t(bytes[1])<<8);bytes+=2;return value;
  };
  if(number()!=page->count)fail("ownership_storage_shape");
  auto rows=std::make_unique<std::vector<Entry>>();rows->reserve(page->count);size_t assigned=0;
  for(size_t i=0;i<page->count;++i) {
    const size_t a=number(),b=number();if(!a || a>256 || b>256 || size_t(end-bytes)<a+b)fail("ownership_storage_shape");
    Entry row{std::string(reinterpret_cast<const char*>(bytes),a),std::string(reinterpret_cast<const char*>(bytes+a),b)};
    bytes+=a+b;key(row.key);if(!row.owner.empty())key(row.owner);
    if(page->ordered && i && rows->back().key>=row.key)fail("ownership_storage_order");
    assigned+=!row.owner.empty();rows->push_back(std::move(row));
  }
  if(bytes!=end || rows->front().key!=page->first || assigned!=page->assigned)fail("ownership_storage_shape");
  const size_t size=decoded(*rows);s.trim(page->count,size);
  const auto counter=s.counter;const auto count=page->count;counter->entries+=count;counter->bytes+=size;
  Lease lease(rows.release(),[counter,count,size](const std::vector<Entry>* rows){delete rows;counter->entries-=count;counter->bytes-=size;});
  s.peak_entries=std::max(s.peak_entries,counter->entries.load());s.peak_bytes=std::max(s.peak_bytes,counter->bytes.load());
  ++s.reads;s.order.push_front(page->id);
  try{s.cache.emplace(page->id,State::Cached{lease,s.order.begin()});}catch(...){s.order.pop_front();throw;}
  return lease;
}
void SubmapOwnershipStore::each(const Snapshot& root,const std::function<void(const Entry&)>& visitor) const {
  validate(root);for(const auto& page:root->pages){auto rows=pin(page);for(const auto& row:*rows)visitor(row);}
}
SubmapOwnershipStore::Snapshot SubmapOwnershipStore::keys(const std::vector<std::string>& values,const std::function<bool()>& cancelled) {
  if(values.empty())return {};
  auto& s=*state_;std::lock_guard<std::recursive_mutex> lock(s.mutex);s.check();
  // Factor-row labels participate in the enclosing ownership transaction.
  // Independent row clients get a short transaction of their own.
  const bool nested=s.in_transaction;if(!nested)s.collect();cancel(cancelled);
  auto root=newRoot(0);root->inventory=false;
  if(!nested){s.execute("BEGIN IMMEDIATE");s.in_transaction=true;}
  try {
    std::vector<Entry> page;page.reserve(s.limits.page_entries);
    for(const auto& value:values) {
      key(value);page.push_back({value,{}});
      if(page.size()==s.limits.page_entries){cancel(cancelled);append(*root,put(page,0,false));page.clear();}
    }
    if(!page.empty())append(*root,put(page,0,false));cancel(cancelled);
    if(!nested){s.execute("COMMIT");s.in_transaction=false;}return root;
  }catch(...){if(!nested){sqlite3_exec(s.db,"ROLLBACK",nullptr,nullptr,nullptr);s.in_transaction=false;}throw;}
}
void SubmapOwnershipStore::eachKey(const Snapshot& root,const std::function<void(const std::string&)>& visitor) const {
  if(!root)return;
  if(root->state!=state_ || root->inventory)throw std::invalid_argument("ownership_keys_owner");
  for(const auto& page:root->pages){auto rows=pin(page);for(const auto& row:*rows)visitor(row.key);}
}
std::shared_ptr<const SubmapMeasurements::Source> SubmapOwnershipStore::measurements(
    const Snapshot& root,const std::vector<std::string>& excluded_owners) const {
  validate(root);auto& s=*state_;std::lock_guard<std::recursive_mutex> lock(s.mutex);s.check();
  struct View final:SubmapMeasurements::Source {
    SubmapOwnershipStore store;Snapshot root;std::set<std::string> excluded;
    std::shared_ptr<State> state;size_t count=0,charged=0;
    View(const SubmapOwnershipStore& source,Snapshot snapshot):store(source),root(std::move(snapshot)){}
    ~View(){if(state)state->counter->directory-=charged;}
    size_t size() const override{return count;}
    bool contains(const std::string& key) const override {
      auto owner=store.lookup(root,key);return owner && !owner->empty() && !excluded.count(*owner);
    }
    void each(const std::function<void(const std::string&)>& visitor) const override {
      store.each(root,[&](const Entry& row){if(!row.owner.empty() && !excluded.count(row.owner))visitor(row.key);});
    }
  };
  auto view=std::make_shared<View>(*this,root);size_t bytes=sizeof(View)+32;
  for(const auto& owner:excluded_owners) {
    key(owner);auto inserted=view->excluded.insert(owner);
    if(inserted.second)bytes+=sizeof(std::string)+4*sizeof(void*)+inserted.first->capacity()+1;
    if(bytes>s.limits.directory_bytes-s.counter->directory.load())fail("ownership_directory_capacity");
  }
  s.reserveDirectory(bytes);view->state=state_;view->charged=bytes;view->count=root->assigned;
  if(!view->excluded.empty())each(root,[&](const Entry& row){if(view->excluded.count(row.owner))--view->count;});
  return view;
}
std::optional<std::string> SubmapOwnershipStore::lookup(const Snapshot& root,const std::string& key) const {
  validate(root);const auto page=std::upper_bound(root->pages.begin(),root->pages.end(),key,
      [](const std::string& key,const auto& page){return key<page->first;});
  if(page==root->pages.begin())return std::nullopt;
  auto rows=pin(*std::prev(page));const auto found=std::lower_bound(rows->begin(),rows->end(),key,
      [](const Entry& row,const std::string& key){return row.key<key;});
  if(found==rows->end() || found->key!=key)return std::nullopt;return found->owner;
}
SubmapOwnershipStore::Editor::Editor(State& state,bool empty,const std::function<bool()>& cancelled)
    :state_(state),empty_(empty),cancelled_(cancelled) {}
void SubmapOwnershipStore::Editor::remove(const std::string& owner) {
  key(owner);Statement insert(state_.db,"INSERT INTO removals(owner) VALUES(?1)");insert.text(1,owner);insert.done("ownership_removed_reused");
}
void SubmapOwnershipStore::Editor::factor(const std::string& owner) {
  key(owner);Statement insert(state_.db,"INSERT INTO incoming(owner) VALUES(?1)");insert.text(1,owner);insert.done("ownership_factor_reused");owner_=owner;
}
void SubmapOwnershipStore::Editor::insert(const std::string& source,const std::string& owner,int kind) {
  key(source);Statement insert(state_.db,"INSERT INTO changes VALUES(?1,?2,?3)");insert.text(1,source);insert.text(2,owner);insert.number(3,kind);
  insert.done("ownership_original_reused");++staged_;
  if(staged_%256==0)cancel(cancelled_);
}
void SubmapOwnershipStore::Editor::assign(const std::string& source) {
  if(owner_.empty())throw std::invalid_argument("ownership_factor_missing");insert(source,owner_,0);
  Statement count(state_.db,"UPDATE incoming SET n=n+1 WHERE owner=?1");count.text(1,owner_);count.done();
}
void SubmapOwnershipStore::Editor::unassign(const std::string& source){insert(source,{},1);}
void SubmapOwnershipStore::Editor::seed(const std::string& source) {
  if(!empty_)throw std::invalid_argument("ownership_noninitial_seed");insert(source,{},2);
}
SubmapOwnershipStore::Snapshot SubmapOwnershipStore::stage(const Snapshot& previous,uint64_t revision,
    const std::function<void(Editor&)>& edit,const std::function<bool()>& cancelled) {
  validate(previous);if(revision<=previous->revision)throw std::invalid_argument("ownership_revision");
  auto& s=*state_;std::lock_guard<std::recursive_mutex> lock(s.mutex);s.check();s.collect();cancel(cancelled);
  if(s.in_transaction)throw std::logic_error("ownership_nested_stage");
  auto root=newRoot(revision);s.execute("BEGIN IMMEDIATE");s.in_transaction=true;
  try {
    for(auto sql:{"DELETE FROM changes","DELETE FROM removals","DELETE FROM incoming"})s.execute(sql);
    Editor editor(s,previous->count==0,cancelled);edit(editor);cancel(cancelled);
    {Statement empty(s.db,"SELECT 1 FROM incoming WHERE n=0 LIMIT 1");if(empty.row())throw std::invalid_argument("ownership_factor_empty");}
    // One transaction can inspect the same owner on many original rows.
    // Keep statement lifetime at the transaction scope; bindings and results
    // are reset for every row, preserving all ownership checks and writes.
    Statement flags(s.db,"SELECT EXISTS(SELECT 1 FROM removals WHERE owner=?1),EXISTS(SELECT 1 FROM incoming WHERE owner=?1)");
    Statement seen(s.db,"UPDATE removals SET seen=1 WHERE owner=?1");
    const auto removed=[&](const std::string& owner) {
      if(owner.empty())return false;
      flags.reset();flags.text(1,owner);
      if(!flags.row())fail("ownership_storage_io");const bool remove=flags.number(0),incoming=flags.number(1);
      if(incoming && !remove)throw std::invalid_argument("ownership_factor_reused");
      if(remove){seen.reset();seen.text(1,owner);seen.done();}
      return remove;
    };
    {
      Statement changes(s.db,"SELECT key,owner,kind FROM changes ORDER BY key");
      bool pending=false;Entry change;int kind=0;
      const auto advance=[&](){pending=changes.row();if(pending){change={changes.text(0),changes.text(1)};kind=int(changes.number(2));}};
      advance();
      for(size_t p=0;p<std::max(size_t(1),previous->pages.size());++p) {
        cancel(cancelled);const Reference page=previous->pages.empty()?nullptr:previous->pages[p];
        Lease old=page?pin(page):Lease();
        const std::string* upper=p+1<previous->pages.size()?&previous->pages[p+1]->first:nullptr;
        const auto within=[&](){return pending && (!upper || change.key<*upper);};
        std::vector<uint8_t> flags;bool any_removed=false;
        if(old){flags.reserve(old->size());for(const auto& row:*old){flags.push_back(removed(row.owner));any_removed|=bool(flags.back());}}
        if(page && !within() && !any_removed){append(*root,page);continue;}
        std::vector<Entry> buffer;buffer.reserve(s.limits.page_entries);size_t i=0;
        const auto flush=[&](){if(!buffer.empty()){append(*root,put(buffer,flags.capacity()+sizeof(change)+change.key.capacity()+change.owner.capacity()+2));buffer.clear();}};
        while((old && i<old->size()) || within()) {
          const Entry* row=old && i<old->size()?&old->at(i):nullptr;Entry value;
          if(within() && (!row || change.key<row->key)) {
            if(kind==1)throw std::invalid_argument("ownership_unassignment");value=std::move(change);advance();
          }else if(within() && row && change.key==row->key) {
            if(!row->owner.empty() && !flags[i])throw std::invalid_argument("ownership_original_reused");
            if(kind==1 && !flags[i])throw std::invalid_argument("ownership_unassignment");
            value=std::move(change);++i;advance();
          }else {
            if(flags[i])throw std::invalid_argument("ownership_original_missing");value=*row;++i;
          }
          buffer.push_back(std::move(value));if(buffer.size()==s.limits.page_entries)flush();
        }
        flush();
      }
      if(pending)throw std::logic_error("ownership_merge_incomplete");
    }
    {Statement missing(s.db,"SELECT 1 FROM removals WHERE seen=0 LIMIT 1");if(missing.row())throw std::invalid_argument("ownership_factor_unknown");}
    for(auto sql:{"DELETE FROM changes","DELETE FROM removals","DELETE FROM incoming"})s.execute(sql);
    cancel(cancelled);s.execute("COMMIT");s.in_transaction=false;
    s.transaction_entries=std::max(s.transaction_entries,editor.staged_);return root;
  }catch(...) {sqlite3_exec(s.db,"ROLLBACK",nullptr,nullptr,nullptr);s.in_transaction=false;throw;}
}
void SubmapOwnershipStore::evict(){auto& s=*state_;std::lock_guard<std::recursive_mutex> lock(s.mutex);s.cache.clear();s.order.clear();}
void SubmapOwnershipStore::collect(){auto& s=*state_;std::lock_guard<std::recursive_mutex> lock(s.mutex);s.collect();}
SubmapOwnershipStore::Statistics SubmapOwnershipStore::statistics() const {
  auto& s=*state_;std::lock_guard<std::recursive_mutex> lock(s.mutex);s.check();
  Statement count(s.db,"SELECT count(*),COALESCE(sum(length(payload)),0) FROM pages");if(!count.row())fail("ownership_storage_io");
  Statement pages(s.db,"PRAGMA page_count"),size(s.db,"PRAGMA page_size");if(!pages.row() || !size.row())fail("ownership_storage_io");
  return {s.counter->entries.load(),s.peak_entries,s.counter->bytes.load(),s.peak_bytes,
    s.counter->directory.load(),s.peak_directory,s.peak_workspace,size_t(count.number(0)),size_t(count.number(1)),
    size_t(pages.number(0)*size.number(0)),s.reads,s.transaction_entries};
}
} // namespace posim_slam
