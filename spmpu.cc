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
typedef DUInt<u15, 32768> u16;

typedef SimplePMPU<uint16_t, u16, 65536> pmpu;
typedef SimpleMPU<uint16_t, 16, pmpu> mpu;

int main(int argc, char* argv[]) {
  assert(1 < argc);
  mpu p;
  std::ifstream ifs(argv[1]);
  if(ifs.fail()) {
    std::cerr << "Failed to open file." << std::endl;
    return - 1;
  }
  ifs.read(reinterpret_cast<char*>(reinterpret_cast<size_t>(&p.mem.m.ireg)), sizeof(pmpu));;
  p.mem.m.pctr ^= p.mem.m.pctr;
  // initialize;
  while(true) {
    p.process();
  }
  return 0;
}

