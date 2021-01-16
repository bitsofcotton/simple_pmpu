#if !defined(_SIMPLE_MPU_)

using std::vector;
using std::pair;
using std::make_pair;

template <typename T, int bits> class SimplePMPU {
public:
  inline SimplePMPU();
  inline ~SimplePMPU();
  inline void nand(const int& dst, const int& src, const int& blksize, const int& cnt, const int& intsize, const int& cond, const int& condoff);
  inline void cmp(const int& dst, const int& src, const int& blksize, const int& cnt, const int& intsize, const int& wrt);
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

template <typename T, int bits> inline void SimplePMPU<T,bits>::nand(const int& dst, const int& src, const int& blksize, const int& cnt, const int& intsize, const int& cond, const int& condoff) {
  assert(0 <= dst && dst < blksize && 0 <= condoff && condoff < blksize &&
         0 < blksize && 0 < cnt);
  assert(0 <= intsize && intsize < blksize);
  static T one(1);
  const auto mask((one << intsize) - one);
  const auto alu(~ (ireg & (ireg >> (src - dst))));
  for(int i = 0; i < cnt; i ++)
    if((int(ireg >> (condoff + dst)) & cond) == cond) {
      ireg &= mask << (i * blksize + dst);
      ireg |= alu & (mask << (i * blksize + dst));
    }
  pctr ++;
  return;
}

template <typename T, int bits> inline void SimplePMPU<T,bits>::cmp(const int& dst, const int& src, const int& blksize, const int& cnt, const int& intsize, const int& wrt) {
  assert(0 <= dst && dst < blksize &&
         0 < blksize && 0 < cnt);
  assert(0 <= intsize && intsize < blksize);
  static T one(1);
  const auto mask((one << intsize) - one);
  for(int i = 0; i < cnt; i ++) {
    ireg &= ~ (T(15) << (i * blksize + dst + wrt));
    ireg |= (((ireg & mask) == ((ireg >> (src - dst)) & mask) ?
              T(1) : T(2)) |
             ((ireg & mask) <  ((ireg >> (src - dst)) & mask) ?
              T(4) : T(0)) |
             ((ireg & mask) >  ((ireg >> (src - dst)) & mask) ?
              T(8) : T(0))) << (i * blksize + dst + wrt);
  }
  pctr ++;
  return;
}


template <typename T> class SimpleAddr {
public:
  inline SimpleAddr();
  inline ~SimpleAddr();
  inline void write(T& dst, const T& src0, const T& src1);
  inline void next();
private:
  vector<pair<T*, T> > q;
};

template <typename T> inline SimpleAddr<T>::SimpleAddr() {
  q.resize(sizeof(T) * 8, make_pair(static_cast<T*>(0), T(0)));
}

template <typename T> inline SimpleAddr<T>::~SimpleAddr() {
  ;
}

template <typename T> inline void SimpleAddr<T>::write(T& dst, const T& src0, const T& src1) {
  q[q.size() - 1] = make_pair(&dst, src0 + src1);
}

template <typename T> inline void SimpleAddr<T>::next() {
  if(q[0].first) *(q[0].first) = q[0].second;
  for(int i = 0; i < q.size() - 1; i ++)
    q[i] = q[i + 1];
  q[q.size() - 1] = make_pair(static_cast<T*>(0), T(0));
}


template <typename T> class SimpleALU {
public:
  inline SimpleALU();
  inline ~SimpleALU();
  inline void write(T& dst, const T& opoff, const T& src0, const T& src1);
  inline void next();
  T* top;
private:
  vector<pair<T*, T> > q;
};

template <typename T> inline SimpleALU<T>::SimpleALU() {
  q.resize(sizeof(T) * 8, make_pair(static_cast<T*>(0), T(0)));
  top = static_cast<T*>(0);
}

template <typename T> inline SimpleALU<T>::~SimpleALU() {
  ;
}

template <typename T> inline void SimpleALU<T>::write(T& dst, const T& opoff, const T& src0, const T& src1) {
  T wrt(0);
  for(int i = 0; i < sizeof(T) * 8; i ++) {
    wrt = ~ (wrt & (src0 & (1 << i) ? top + (sizeof(T) * 8) * (i + sizeof(T) * 8 * opoff * 2): 0 ));
    wrt = ~ (wrt & (src1 & (1 << i) ? top + (sizeof(T) * 8) * (i + sizeof(T) * 8 * (opoff * 2 + 1)): 0 ));
  }
  q[q.size() - 1] = make_pair(&dst, wrt);
}


template <typename T, int pages, int ipages> class SimpleMPU {
public:
  typedef struct {
    uint8_t r : 1;
    uint8_t w : 1;
    uint8_t x : 1;
    uint8_t u : 1;
    uint8_t i : 1;
    uint8_t nowriteflushcache : 1;
    uint8_t noreadflushcache  : 1;
    uint8_t perchipcache : 1;
    T rel0;
    T rel1;
  } paging_t;
  typedef struct {
    uint8_t r : 1;
    uint8_t w : 1;
    uint8_t x : 1;
    uint8_t i : 5;
  } interrupt_t;
  typedef struct {
    T rip;
    T reg;
    T irip;
    T ireg;
    T control;
    uint8_t  cond;
    paging_t page[pages];
    interrupt_t interrupt[ipages];
  } pu_t;
  typedef enum {
    COND_USER      = 0,
    COND_INTERRUPT = 1,
    COND_EQUAL     = 2,
    COND_NOTEQUAL  = 3,
    COND_LESSER    = 4,
    COND_GREATER   = 5,
    COND_INT_MPU   = 6,
    COND_NOUSED    = 7
  } cond_e;
  typedef enum {
    OP_LDOPTOP = 0,
    OP_OP     = 1,
    OP_NAND   = 2,
    OP_PLUS   = 3,
    OP_SLEFT  = 4,
    OP_SRIGHT = 5,
    OP_CMP    = 6,
    OP_LDIP   = 7,
    OP_STIP   = 8,
    OP_INT    = 9,
    OP_IRET   = 10,
    OP_STPAGEINTCONTROL = 11,
    OP_LDPAGEINTCONTROL = 12,
    OP_CALLPCMP  = 13,
    OP_CALLPNAND = 14,
    OP_NOP    = 15
  } op_e;
  typedef enum {
    INT_INVPRIV = 0,
    INT_DBLINT  = 1,
    INT_PGFLT   = 2,
    INT_INVOP   = 3
  } interrupt_e;
  typedef struct {
    uint8_t off : 7;
    uint8_t ref : 1;
  } operand_t;
  typedef struct {
    uint8_t   cond;
    uint8_t   op    : 4;
    uint8_t   opidx : 4;
    operand_t dst;
    operand_t src;
    operand_t wrt;
  } mnemonic_t;
  inline SimpleMPU();
  inline SimpleMPU(const int& npu);
  inline ~SimpleMPU();
  inline void process();
  vector<pu_t> pu;
  vector<SimpleAddr<T> > addr;
  vector<SimpleALU<T> > alu;
  vector<SimpleALU<T> > ialu;
  int pctr;
};

template <typename T, int pages, int ipages> inline SimpleMPU<T,pages,ipages>::SimpleMPU() {
  pctr ^= pctr;
}

template <typename T, int pages, int ipages> inline SimpleMPU<T,pages,ipages>::SimpleMPU(const int& npu) {
  pu.resize(npu);
  addr.resize(npu);
  alu.resize(npu);
  ialu.resize(npu);
  pctr ^= pctr;
}

template <typename T, int pages, int ipages> inline SimpleMPU<T,pages,ipages>::~SimpleMPU() {
  ;
}

template <typename T, int pages, int ipages> inline void SimpleMPU<T,pages,ipages>::process() {
  for(int i = 0; i < pu.size(); i ++) {
          auto& p(pu[i]);
    const auto& mnemonic(*static_cast<const mnemonic_t*>(p.rip));
    const auto  interrupted(p.cond & (1 << COND_INTERRUPT));
    addr[i].next();
    alu[i].next();
    ialu[i].next();
    if((mnemonic.cond & p.cond) == mnemonic.cond) {
      const T* top(static_cast<T*>(interrupted ? p.ireg : p.reg));
      const auto& dst(mnemonic.dst.ref ? : top + mnemonic.dst.off);
      const auto& src(mnemonic.src.ref ? : top + mnemonic.dst.off);
      const auto& wrt(mnemonic.wrt.ref ? : top + mnemonic.dst.off);
      switch(mnemonic.op & 0x0f) {
      case OP_LDOPTOP:
        (interrupted ? ialu : alu).top = static_cast<T>(&wrt);
        break;
      case OP_OP:
        (interrupted ? ialu : alu).write(wrt, dst, src);
        break;
      case OP_NAND:
        wrt = ~ (dst & src);
        break;
      case OP_PLUS:
        addr.write(wrt, mnemonic.opoff, dst, src);
        break;
      case OP_SLEFT:
        wrt = dst << src;
        break;
      case OP_SRIGHT:
        wrt = dst >> src;
        break;
      case OP_CMP:
        p.cond &= ~ ((1LL << COND_EQUAL)    |
                     (1LL << COND_NOTEQUAL) |
                     (1LL << COND_LESSER)   |
                     (1LL << COND_GREATER));
        p.cond |= (dst == src ? (1LL << COND_EQUAL)
                              : (1LL << COND_NOTEQUAL))    |
                  (dst <  src ? (1LL << COND_LESSER)  : 0) |
                  (src <  dst ? (1LL << COND_GREATER) : 0);
        break;
      case OP_LDIP:
        wrt = interrupted ? p.irip : p.rip;
        break;
      case OP_STIP:
        (interrupted ? p.irip : p.rip) = dst;
        break;
      case OP_INT:
        if(interrupted)
          // double interrupt.
          ;
        else
          ;
        break;
      case OP_IRET:
        if(interrupted)
          ;
        else
          // invop:
          ;
        break;
      case OP_LDPAGEINTCONTROL:
        if(interrupted)
          ;
        else
          ; // priv
        break;
      case OP_STPAGEINTCONTROL:
        if(interrupted)
          ;
        else
          ; // priv
        break;
      case OP_CALLPCMP:
        // call parallel PMPU.
        break;
      case OP_CALLPNAND:
        // call parallel PMPU.
        break;
      case OP_NOP:
        break;
      default:
        // interrupt:
        assert(0 && "Should not be reached.");;
      }
    }
    p.rip += sizeof(mnemonic_t);
  }
  pctr ++;
  return;
}

#define _SIMPLE_MPU_
#endif

