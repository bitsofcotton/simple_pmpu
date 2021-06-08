#include <cstdio>
#include <cmath>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <map>
#include <assert.h>
#include "lieonn.hh"
#include "spmpu.hh"

typedef DUInt<uint64_t, 64> u7;
typedef DUInt<u7,  128>   u8;
typedef DUInt<u8,  256>   u9;
typedef DUInt<u9,  512>   u10;
typedef DUInt<u10, 1024>  u11;
typedef DUInt<u11, 2048>  u12;
typedef DUInt<u12, 4096>  u13;
typedef DUInt<u13, 8192>  u14;
typedef DUInt<u14, 16384> u15;
typedef DUInt<u15, 32768> u16; // 64k
typedef DUInt<u16, 65536> u17;
typedef DUInt<u17, 131072> u18;
typedef DUInt<u18, 262144> u19;
typedef DUInt<u19, 514288> u20; // 1m
typedef DUInt<u20, 1048576> u21;
typedef DUInt<u21, 2097152> u22;

typedef SimpleMPU<uint32_t, u22, 12> mpu;

int main(int argc, char* argv[]) {
  assert(1 < argc);
  mpu p;
  std::ifstream ifs(argv[1]);
  if(ifs.fail()) {
    std::cerr << "Failed to open file." << std::endl;
    return - 1;
  }
  ifs.read(reinterpret_cast<char*>(reinterpret_cast<size_t>(&p.mem.m)), sizeof(u16));
  while(true) {
    p.process();
  }
  return 0;
}

