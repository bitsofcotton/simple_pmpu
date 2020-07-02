#if !defined(_SIMPLE_MPU_)

template <typename T, int bits> class SimplePMPU {
public:
  inline SimplePMPU();
  inline ~SimplePMPU();
  inline void nand(const int& dst, const int& src, const int& blksize, const int& cnt, const int& dist);
  T   ireg;
  int pctr;
};

template <typename T, int bits> inline SimplePMPU<T,bits>::SimplePMPU() {
  ireg ^= ireg;
  pctr ^= pctr;
}

template <typename T, int bits> inline SimplePMPU<T,bits>::~SimplePMPU() {
  ;
}

template <typename T, int bits> inline void SimplePMPU<T,bits>::nand(const int& dst, const int& src, const int& blksize, const int& cnt, const int& dist) {
  assert(0 <= dst && 0 <= src && 0 < blksize && 0 < cnt && 0 < dist);
  assert(blksize <= dist);
  assert(dst + dist * cnt + blksize < bits);
  assert(src + dist * cnt + blksize < bits);
  static T one(1);
  const auto mask0((one << blksize) - one);
        T    maskd;
  maskd ^= maskd;
  for(int i = 0; i < cnt; i ++)
    maskd |= mask0 << (i * dist + dst);
  const auto alu(maskd & (~ (ireg & (ireg >> (src - dst)))));
  ireg &= ~ maskd;
  ireg |=   alu;
  pctr ++;
  return;
}



template <typename T, int bits, int rs> class vInt : SimplePMPU<T, bits> {
public:
  inline vInt() {
    ;
  }
  inline ~vInt() {
    ;
  }
 
  inline void load(const int& idx, const T& x);
  inline T    stor(const int& idx);
  
  // one operand.
  inline void increment(const int& cond, const int& start, const int& dist = 1);
  inline void decrement(const int& cond, const int& start, const int& dist = 1);
  inline void Not(const int& cond, const int& start, const int& dist = 1);
  inline void negate(const int& cond, const int& start, const int& dist = 1);
  // every pair.
  inline void plus(const int& cond, const int& start, const int& dist = 1);
  inline void minus(const int& cond, const int& start, const int& dist = 1);
  inline void mul(const int& cond, const int& start, const int& dist = 1);
  inline void div(const int& cond, const int& start, const int& dist = 1);
  inline void residue(const int& cond, const int& start, const int& dist = 1);
  inline void shift(const int& cond, const int& b, const int& start, const int& dist = 1);
  inline void And(const int& cond, const int& start, const int& dist = 1);
  inline void Or(const int& cond, const int& start, const int& dist = 1);
  inline void Xor(const int& cond, const int& start, const int& dist = 1);
  // 
  inline void transfer(const int& cond, const int& dst, const int& src, const int& dist = 1);
  inline void cmp(const int& cond, const int& start, const int& dist = 1);
};


template <typename T, int bits, int rs> class vFloat : vInt<T, bits, rs> {
public:
  inline vFloat(){
    ;
  }
  inline ~vFloat() {
    ;
  }
  
  inline void load(const int& idx, const T& x);
  inline T    stor(const int& idx);
  
  // one operand.
  inline void negate(const int& cond, const int& start, const int& dist = 1);
  // every pair.
  inline void plus(const int& cond, const int& start, const int& dist = 1);
  inline void minus(const int& cond, const int& start, const int& dist = 1);
  inline void mul(const int& cond, const int& start, const int& dist = 1);
  inline void div(const int& cond, const int& start, const int& dist = 1);
  inline void residue(const int& cond, const int& start, const int& dist = 1);
  inline void floor(const int& cond, const int& start, const int& dist = 1);
  // 
  inline void transfer(const int& cond, const int& dst, const int& src, const int& dist = 1);
  inline void cmp(const int& cond, const int& start, const int& dist = 1);
};

#define _SIMPLE_MPU_
#endif

