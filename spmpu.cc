#include <cstdio>
#include <cmath>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include "ifloat.hh"

#include "ifloat.hh"
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

typedef vFloat<u16, 65536, 32> vfloat;

int main(int argc, char* argv[]) {
  vfloat vf;
  return 0;
}

