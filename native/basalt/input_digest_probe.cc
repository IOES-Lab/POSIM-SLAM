// SPDX-License-Identifier: Apache-2.0
#include "input_digest.hpp"
#include <cassert>
#include <iostream>
#include <string>
#include <vector>
std::string hex(const posim_slam::InputDigest::Value& value) {
  const char* digits="0123456789abcdef";std::string result;
  for(auto v:value){result+=digits[v>>4];result+=digits[v&15];}return result;
}
void check(const std::string& text,const std::string& expected) {
  for(size_t chunk:{size_t(1),size_t(7),size_t(64),size_t(129)}) {
    posim_slam::InputDigest digest;
    for(size_t start=0;start<text.size();start+=chunk)
      digest.update(text.data()+start,std::min(chunk,text.size()-start));
    assert(hex(digest.finish())==expected);assert(hex(digest.finish())==expected);
  }
}
int main() {
  check("","e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  check("abc","ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  check("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
        "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
  check(std::string(1000000,'a'),"cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
  std::cout<<"SHA-256 published vectors and incremental boundary handling pass\n";
}
