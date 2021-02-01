#if !defined(_SIMPLE_MPU_)

using std::vector;
using std::pair;
using std::make_pair;

template <typename T, typename U, int bits> class SimplePMPU {
public:
  inline SimplePMPU();
  inline ~SimplePMPU();
  inline void nand(const T& dst, const T& src, const T& blksize, const T& cnt, const T& intsize, const T& cond, const T& condoff);
  inline void cmp(const T& dst, const T& src, const T& blksize, const T& cnt, const T& intsize, const T& wrt);
  U   ireg;
  int pctr;
};

template <typename T, typename U, int bits> inline SimplePMPU<T,U,bits>::SimplePMPU() {
  ireg ^= ireg;
  pctr ^= pctr;
}

template <typename T, typename U, int bits> inline SimplePMPU<T,U,bits>::~SimplePMPU() {
  ;
}

template <typename T, typename U, int bits> inline void SimplePMPU<T,U,bits>::nand(const T& dst, const T& src, const T& blksize, const T& cnt, const T& intsize, const T& cond, const T& condoff) {
  assert(0 <= dst && dst < blksize && 0 <= condoff && condoff < blksize &&
         0 < blksize && 0 < cnt);
  assert(0 <= intsize && intsize < blksize);
  static U one(1);
  const auto mask((one << intsize) - one);
  const auto alu(~ (ireg & (ireg >> (src - dst))));
  for(T i = 0; i < cnt; i ++)
    if((int(ireg >> (condoff + dst)) & cond) == cond) {
      ireg &= mask << (i * blksize + dst);
      ireg |= alu & (mask << (i * blksize + dst));
    }
  pctr ++;
  return;
}

template <typename T, typename U, int bits> inline void SimplePMPU<T,U,bits>::cmp(const T& dst, const T& src, const T& blksize, const T& cnt, const T& intsize, const T& wrt) {
  assert(0 <= dst && dst < blksize &&
         0 < blksize && 0 < cnt);
  assert(0 <= intsize && intsize < blksize);
  static U one(1);
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
    wrt = wrt ^ (src0 & (1 << i) ? top[i * 8 + 8 * opoff * 2] : 0 );
    wrt = wrt ^ (src1 & (1 << i) ? top[i * 8 + 8 * (opoff * 2 + 1)] : 0 );
  }
  q[q.size() - 1] = make_pair(&dst, wrt);
}

template <typename T> inline void SimpleALU<T>::next() {
  if(q[0].first) *(q[0].first) = q[0].second;
  for(int i = 0; i < q.size() - 1; i ++)
    q[i] = q[i + 1];
  q[q.size() - 1] = make_pair(static_cast<T*>(0), T(0));
}


template <typename T, int pages, typename U> class SimpleMPU {
public:
  typedef struct {
    uint8_t r : 1;
    uint8_t w : 1;
    uint8_t x : 1;
    uint8_t u : 1;
    uint8_t i : 1;
    uint8_t nowriteflushcache : 1;
    uint8_t noreadflushcache  : 1;
    uint8_t nocache : 1;
    T rel0;
    T rel1;
  } paging_t;
  typedef struct {
    T rip;
    T reg;
    T irip;
    T ireg;
    T control;
    // T interrupt[0x10];
    // paging_t page[pages];
    T* interrupt;
    paging_t *page;
    uint8_t  cond;
    uint8_t  pending_interrupt;
    T pctr;
  } pu_t;
  typedef enum {
    COND_USER      = 0,
    COND_INTERRUPT = 1,
    COND_EQUAL     = 2,
    COND_NOTEQUAL  = 3,
    COND_LESSER    = 4,
    COND_GREATER   = 5,
    COND_INT_MPU   = 6,
    COND_HALT      = 7
  } cond_e;
  typedef enum {
    OP_LDOPTOP = 0,
    OP_OP     = 1,
    OP_NAND   = 2,
    OP_PLUS   = 3,
    OP_SHIFT  = 4,
    OP_CMP    = 5,
    OP_LDIPREGTOP  = 6,
    OP_STIPREGTOP  = 7,
    OP_STUIPREGTOP = 8,
    OP_INT    = 9,
    OP_IRET   = 10,
    OP_STPAGEINTCONTROL = 11,
    OP_LDPAGEINTCONTROL = 12,
    OP_CALLPCMP  = 13,
    OP_CALLPNAND = 14,
    OP_NOP    = 15
  } op_e;
  typedef enum {
    INT_HALT    = 0,
    INT_INVPRIV = 1,
    INT_PGFLT   = 2,
    INT_DBLINT  = 3
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
  U   mem;
  T   pctr;
};

template <typename T, int pages, typename U> inline SimpleMPU<T,pages,U>::SimpleMPU() {
  ;
}

template <typename T, int pages, typename U> inline SimpleMPU<T,pages,U>::SimpleMPU(const int& npu) {
  pu.resize(npu);
  addr.resize(npu);
  alu.resize(npu);
  ialu.resize(npu);
}

template <typename T, int pages, typename U> inline SimpleMPU<T,pages,U>::~SimpleMPU() {
  ;
}

template <typename T, int pages, typename U> inline void SimpleMPU<T,pages,U>::process() {
  for(int i = 0; i < pu.size(); i ++) {
          auto& p(pu[i]);
    if(pctr < p.pctr) continue;
    p.pctr ++;
    assert(((p.cond & (1 << COND_INTERRUPT)) >> COND_INTERRUPT) ^
           ((p.cond & (1 << COND_USER)) >> COND_USER));
    const auto  interrupted(p.cond & (1 << COND_INTERRUPT));
    // XXX paging, pipeline:
    const auto& mnemonic(*(static_cast<mnemonic_t*>(reinterpret_cast<void*>(
      reinterpret_cast<size_t>(&mem) + size_t(interrupted ? p.irip : p.rip)))));
    addr[i].next();
    alu[i].next();
    ialu[i].next();
    if((p.cond & (1 << COND_HALT)) && mnemonic.op != OP_INT) continue;
    if((mnemonic.cond & p.cond) == mnemonic.cond || p.pending_interrupt) {
      const auto  top(reinterpret_cast<size_t>(&mem) + size_t(interrupted ? p.ireg : p.reg));
      // XXX paging, pipeline:
      auto& dst0(*(static_cast<T*>(reinterpret_cast<void*>(
        reinterpret_cast<size_t>(&mem) + top + mnemonic.dst.off * sizeof(T)))));
      auto& dst(mnemonic.dst.ref ? *(static_cast<T*>(reinterpret_cast<void*>(dst0))) : dst0);
      auto& src0(*(static_cast<T*>(reinterpret_cast<void*>(
        reinterpret_cast<size_t>(&mem) + top + mnemonic.src.off * sizeof(T)))));
      auto& src(mnemonic.src.ref ? *(static_cast<T*>(reinterpret_cast<void*>(src0))) : src0);
      auto& wrt0(*(static_cast<T*>(reinterpret_cast<void*>(
        reinterpret_cast<size_t>(&mem) + top + mnemonic.wrt.off * sizeof(T)))));
      auto& wrt(mnemonic.wrt.ref ? *(static_cast<T*>(reinterpret_cast<void*>(wrt0))) : wrt0);
      if(p.pending_interrupt) goto pint;
      switch(mnemonic.op & 0x0f) {
      case OP_LDOPTOP:
        p.pctr += sizeof(T) * 8 * 2;
        (interrupted ? ialu[i] : alu[i]).top = &wrt;
        break;
      case OP_OP:
        (interrupted ? ialu[i] : alu[i]).write(wrt, mnemonic.opidx, dst, src);
        break;
      case OP_NAND:
        wrt = ~ (dst & src);
        break;
      case OP_PLUS:
        addr[i].write(wrt, dst, src);
        break;
      case OP_SHIFT:
        wrt = 0 < src ? dst << src : dst >> - src;
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
      case OP_LDIPREGTOP:
        wrt = interrupted ? p.irip : p.rip;
        dst = interrupted ? p.ireg : p.reg;
        break;
      case OP_STIPREGTOP:
        (interrupted ? p.irip : p.rip) = dst;
        (interrupted ? p.ireg : p.reg) = src;
        break;
      case OP_STUIPREGTOP:
        p.rip = dst;
        p.reg = src;
        break;
      case OP_INT:
       pint:
        if(! mnemonic.opidx)
          p.cond |= (1 << COND_HALT);
        else if(interrupted) {
          if(p.pending_interrupt == INT_DBLINT)
            p.cond |= COND_HALT;
          else
            p.pending_interrupt = INT_DBLINT;
          break;
        } else {
          p.irip  = p.interrupt[mnemonic.opidx & 0x0f];
          p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
          p.pctr += sizeof(T) * 8 * 2;
        }
        if(p.pending_interrupt) {
          p.pending_interrupt = 0;
          continue;
        }
        p.pending_interrupt = 0;
        break;
      case OP_IRET:
        if(interrupted) {
          p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
          p.pctr += sizeof(T) * 8 * 2;
        } else {
          if(p.pending_interrupt)
            p.pending_interrupt = INT_DBLINT;
          else
            p.pending_interrupt = INT_INVPRIV;
        }
        break;
      case OP_LDPAGEINTCONTROL:
        if(interrupted) {
          // XXX offset:
          dst = static_cast<T>(reinterpret_cast<size_t>(p.page));
          src = static_cast<T>(reinterpret_cast<size_t>(p.interrupt));
          wrt = p.control;
        } else {
          if(p.pending_interrupt)
            p.pending_interrupt = INT_DBLINT;
          else
            p.pending_interrupt = INT_INVPRIV;
        }
        break;
      case OP_STPAGEINTCONTROL:
        if(interrupted) {
          // XXX : offset
          p.page      = static_cast<paging_t*>(reinterpret_cast<void*>(size_t(dst)));
          p.interrupt = static_cast<T*>(reinterpret_cast<void*>(size_t(src)));
          p.control   = wrt;
        } else {
          if(p.pending_interrupt)
            p.pending_interrupt = INT_DBLINT;
          else
            p.pending_interrupt = INT_INVPRIV;
        }
        break;
      case OP_CALLPCMP:
        if(i || ! interrupted)
          p.pending_interrupt = INT_INVPRIV;
        else
          // XXX no page guard, offset.
          mem.cmp(*static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg))),
                  *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T)))),
                  *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 2))),
                  *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 3))),
                  *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 4))),
                  *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 5))));
        break;
      case OP_CALLPNAND:
        if(i || ! interrupted)
          p.pending_interrupt = INT_INVPRIV;
        else
          // XXX no page guard.
          mem.nand(*static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg))),
                   *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T)))),
                   *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 2))),
                   *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 3))),
                   *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 4))),
                   *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 5))),
                   *static_cast<T*>(reinterpret_cast<void*>(size_t(p.ireg + sizeof(T) * 6))));
        break;
      case OP_NOP:
        break;
      default:
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

