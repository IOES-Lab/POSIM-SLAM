// SPDX-License-Identifier: Apache-2.0
#pragma once
#include "mapping_graph.hpp"

namespace posim_slam {
namespace mapping_frame_io {
using namespace mapping_graph_detail;
using Frame=MappingInputStore::Frame;
constexpr size_t limit=96*1024*1024;
inline Json binary(const InputDigest::Value& value) {
  return Json::binary(std::vector<uint8_t>(value.begin(),value.end()));
}
inline InputDigest::Value digest(const std::vector<uint8_t>& bytes) {
  InputDigest hash;hash.update(bytes.data(),bytes.size());return hash.finish();
}
inline void shape(const Frame& frame) {
  const auto& f=frame.features;const size_t n=f.corners.size();
  if(frame.stamp<1 || frame.width<1 || frame.width>1920 || frame.height<1 || frame.height>1080 ||
      n>1000000 || f.corner_angles.size()!=n || f.corner_descriptors.size()!=n ||
      f.corners_3d.size()!=n || f.hashes.size()!=n || f.bow_vector.size()>n)
    throw std::invalid_argument("mapping_input_frame_shape");
  for(size_t i=0;i<n;++i)if(!f.corners[i].allFinite() || !std::isfinite(f.corner_angles[i]))
    throw std::invalid_argument("mapping_input_frame_nonfinite");
  std::set<uint32_t> words;
  for(const auto& [word,weight]:f.bow_vector)
    if(!words.insert(uint32_t(word.to_ulong())).second || !std::isfinite(weight) || weight<=0 || weight>1)
      throw std::invalid_argument("mapping_input_frame_bow");
}
inline std::vector<uint8_t> encode(const Frame& frame) {
  shape(frame);const auto& f=frame.features;const size_t n=f.corners.size();
  std::vector<double> corners,spatial;corners.reserve(2*n);spatial.reserve(4*n);
  std::vector<uint8_t> descriptors(32*n);Json hashes=Json::array(),bow=Json::array();
  for(size_t i=0;i<n;++i) {
    corners.insert(corners.end(),f.corners[i].data(),f.corners[i].data()+2);
    spatial.insert(spatial.end(),f.corners_3d[i].data(),f.corners_3d[i].data()+4);
    for(size_t bit=0;bit<256;++bit)if(f.corner_descriptors[i][bit])descriptors[32*i+bit/8]|=uint8_t(1u<<(bit%8));
    hashes.push_back(uint32_t(f.hashes[i].to_ulong()));
  }
  for(const auto& [word,weight]:f.bow_vector)bow.push_back({uint32_t(word.to_ulong()),doubles(&weight,1)});
  Json value={{"schema","posim-slam-feature-frame-1"},{"stamp",frame.stamp},{"width",frame.width},
    {"height",frame.height},{"image_sha256",binary(frame.digest)},{"count",n},
    {"corners",doubles(corners.data(),corners.size())},{"angles",doubles(f.corner_angles.data(),n)},
    {"descriptors",Json::binary(std::move(descriptors))},{"unprojected",doubles(spatial.data(),spatial.size())},
    {"hashes",std::move(hashes)},{"bow",std::move(bow)}};
  auto bytes=Json::to_cbor(value);
  if(bytes.size()>limit)throw std::invalid_argument("mapping_input_frame_capacity");return bytes;
}
inline std::shared_ptr<const Frame> decode(const std::vector<uint8_t>& bytes,const Json& expected) {
  const auto value=Json::from_cbor(bytes);auto out=std::make_shared<Frame>();auto& f=out->features;
  if(value.at("schema")!="posim-slam-feature-frame-1")throw std::invalid_argument("mapping_input_frame_schema");
  out->stamp=integer(value.at("stamp"),1,INT64_MAX);out->width=integer(value.at("width"),1,1920);
  out->height=integer(value.at("height"),1,1080);const size_t n=integer(value.at("count"),0,1000000);
  const auto& identity=value.at("image_sha256");
  if(!identity.is_binary() || identity.get_binary().size()!=32 ||
      Json::array({out->stamp,out->width,out->height,n,identity})!=expected)
    throw std::invalid_argument("mapping_input_frame_identity");
  std::copy(identity.get_binary().begin(),identity.get_binary().end(),out->digest.begin());
  const auto corners=doubles(value.at("corners"),2*n),spatial=doubles(value.at("unprojected"),4*n,false);
  f.corner_angles=doubles(value.at("angles"),n);const auto& descriptors=value.at("descriptors");
  const auto& hashes=value.at("hashes");const auto& bow=value.at("bow");
  if(!descriptors.is_binary() || descriptors.get_binary().size()!=32*n ||
      !hashes.is_array() || hashes.size()!=n || !bow.is_array() || bow.size()>n)
    throw std::invalid_argument("mapping_input_frame_shape");
  f.corners.reserve(n);f.corners_3d.reserve(n);f.corner_descriptors.resize(n);f.hashes.reserve(n);
  for(size_t i=0;i<n;++i) {
    f.corners.emplace_back(corners[2*i],corners[2*i+1]);
    f.corners_3d.emplace_back(spatial[4*i],spatial[4*i+1],spatial[4*i+2],spatial[4*i+3]);
    for(size_t bit=0;bit<256;++bit)f.corner_descriptors[i][bit]=bool(descriptors.get_binary()[32*i+bit/8]&(1u<<(bit%8)));
    f.hashes.emplace_back(integer(hashes[i],0,UINT32_MAX));
  }
  for(const auto& row:bow) {
    if(!row.is_array() || row.size()!=2)throw std::invalid_argument("mapping_input_frame_shape");
    f.bow_vector.emplace_back(basalt::FeatureHash(integer(row[0],0,UINT32_MAX)),doubles(row[1],1)[0]);
  }
  shape(*out);return out;
}
} // namespace mapping_frame_io

// Keep the opened file, not its pathname: replacing an archive must not change
// an existing snapshot's input identity. Concurrent readers serialize seek/read;
// each request owns at most one encoded frame and no persistent page cache.
class MappingFramePages {
 public:
  using Json=nlohmann::json;
  MappingFramePages(std::ifstream&& file,uint64_t start,uint64_t end,const Json& table,const Json& frames)
      :file_(std::move(file)),start_(start) {
    using namespace mapping_graph_detail;
    if(!table.is_array() || table.size()!=frames.size())throw std::invalid_argument("mapping_input_page_index");
    uint64_t offset=0;
    for(size_t i=0;i<table.size();++i) {
      const auto& row=table[i];const auto& frame=frames[i];
      if(!row.is_array() || row.size()!=4 || row[0]!=frame[0] || integer(row[1],0,mapping_frame_io::limit)!=int64_t(offset) ||
          !row[3].is_binary() || row[3].get_binary().size()!=32)
        throw std::invalid_argument("mapping_input_page_index");
      const auto stamp=integer(row[0],1,INT64_MAX);const size_t length=integer(row[2],1,mapping_frame_io::limit);
      if(offset+length>mapping_frame_io::limit || !pages_.emplace(stamp,Page{offset,length,row[3],frame}).second)
        throw std::invalid_argument("mapping_input_page_index");offset+=length;
    }
    if(start+offset!=end)throw std::invalid_argument("mapping_input_page_index");
  }
  std::shared_ptr<const MappingInputStore::Frame> read(int64_t stamp,size_t maximum) {
    using namespace mapping_frame_io;
    const auto found=pages_.find(stamp);
    if(found==pages_.end())throw std::invalid_argument("mapping_input_frame_missing");const auto& page=found->second;
    if(page.length>maximum)throw std::runtime_error("mapping_input_page_budget");
    std::vector<uint8_t> bytes(page.length);
    {std::lock_guard<std::mutex> lock(mutex_);file_.clear();file_.seekg(start_+page.offset);
      file_.read(reinterpret_cast<char*>(bytes.data()),bytes.size());
      if(!file_)throw std::runtime_error("mapping_input_page_read");}
    if(binary(digest(bytes))!=page.digest)throw std::invalid_argument("mapping_input_page_digest");
    return decode(bytes,page.frame);
  }
 private:
  struct Page {uint64_t offset;size_t length;Json digest,frame;};
  std::ifstream file_;uint64_t start_;std::mutex mutex_;std::map<int64_t,Page> pages_;
};
} // namespace posim_slam
