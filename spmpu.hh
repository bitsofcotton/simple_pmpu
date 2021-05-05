#if !defined(_SIMPLE_MPU_)

using std::vector;
using std::pair;
using std::make_pair;

template <typename T, typename U> class Mem {
public:
  inline Mem();
  inline ~Mem();
  inline const T& read(const T& idx) const;
  inline void     write(const T& idx, const T& wrt);
  U m;
private:
  int bs;
  // peripheral:
};

template <typename T, typename U> inline Mem<T,U>::Mem() {
  m ^= m;
  for(bs = 0; T(1) << bs != T(0); bs ++);
  bs --;
}

template <typename T, typename U> inline Mem<T,U>::~Mem() {
  m ^= m;
}

template <typename T, typename U> inline const T& Mem<T,U>::read(const T& idx) const {
  if(idx & (T(1) << bs)) {
    ;
  }
  assert(sizeof(T) * idx < sizeof(U));
  return static_cast<T*>(reinterpret_cast<void*>(&m))[size_t(idx)];
}

template <typename T, typename U> inline void Mem<T,U>::write(const T& idx, const T& wrt) {
  if(idx & (T(1) << bs)) {
    ;
  }
  assert(sizeof(T) * idx < sizeof(U));
  return static_cast<T*>(reinterpret_cast<void*>(&m))[size_t(idx)] = wrt;
}


template <typename T, typename U, int bits> class SimplePMPU {
public:
  inline SimplePMPU();
  inline ~SimplePMPU();
  inline void nand(const T& dst, const T& src, const T& blksize, const T& cnt, const T& intsize, const T& cond, const T& condoff);
  inline void cmp(const T& dst, const T& src, const T& blksize, const T& cnt, const T& intsize, const T& wrt);
  Mem<T,U> ireg;
  int      pctr;
};

template <typename T, typename U, int bits> inline SimplePMPU<T,U,bits>::SimplePMPU() {
  ireg.m ^= ireg.m;
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
  const auto alu(~ (ireg.m & (ireg.m >> (src - dst))));
  for(T i = 0; i < cnt; i ++)
    if((int(ireg.m >> (condoff + dst)) & cond) == cond) {
      ireg.m &= mask << (i * blksize + dst);
      ireg.m |= alu & (mask << (i * blksize + dst));
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
    ireg.m &= ~ (T(15) << (i * blksize + dst + wrt));
    ireg.m |= (((ireg.m & mask) == ((ireg.m >> (src - dst)) & mask) ?
              T(1) : T(2)) |
             ((ireg.m & mask) <  ((ireg.m >> (src - dst)) & mask) ?
              T(4) : T(0)) |
             ((ireg.m & mask) >  ((ireg.m >> (src - dst)) & mask) ?
              T(8) : T(0))) << (i * blksize + dst + wrt);
  }
  pctr ++;
  return;
}


template <typename T, typename U> class MemPage;

template <typename T, typename U> class SimpleAddr {
public:
  inline SimpleAddr(MemPage<T, U>& m);
  inline ~SimpleAddr();
  inline void write(const T& dsttop, const T& dstidx, const T& dstaddr, const T& src0, const T& src1, const uint8_t& cond);
  inline void next();
private:
  vector<pair<T, pair<pair<T, T>, pair<T, uint8_t> > > > q;
  MemPage<T, U>* m;
};

template <typename T, typename U> inline SimpleAddr<T,U>::SimpleAddr(MemPage<T,U>& m) {
  q.resize(sizeof(T) * 8, make_pair(static_cast<T*>(0), make_pair(T(0), make_pair(T(0), T(0)), make_pair(T(0), 0))));
  this->m = &m;
}

template <typename T, typename U> inline SimpleAddr<T,U>::~SimpleAddr() {
  ;
}

template <typename T, typename U> inline void SimpleAddr<T,U>::write(const T& dsttop, const T& dstidx, const T& dstaddr, const T& src0, const T& src1, const uint8_t& cond) {
  q[q.size() - 1] = make_pair(src0 + src1, make_pair(make_pair(dsttop, dstidx), make_pair(dstaddr, cond)));
}

template <typename T, typename U> inline void SimpleAddr<T,U>::next() {
  if(q[0].second.first.first)
    m->write(q[0].second.first.first, q[0].second.first.second, q[0].second.second.first, q[0].first, q[0].second.second.second);
  for(int i = 0; i < q.size() - 1; i ++)
    q[i] = q[i + 1];
  q[q.size() - 1] = make_pair(T(0), make_pair(make_pair(T(0), T(0)), make_pair(T(0), 0)));
}


template <typename T, typename U> class SimpleALU {
public:
  inline SimpleALU(MemPage<T,U>& m);
  inline ~SimpleALU();
  inline void write(T& dst, const T& opoff, const T& src0, const T& src1);
  inline void next();
  T top;
private:
  vector<pair<T*, T> > q;
  MemPage<T,U>* m;
};

template <typename T, typename U> inline SimpleALU<T,U>::SimpleALU(MemPage<T,U>& m) {
  q.resize(sizeof(T) * 8, make_pair(static_cast<T*>(0), T(0)));
  top = static_cast<T*>(0);
  this->m = &m;
}

template <typename T, typename U> inline SimpleALU<T,U>::~SimpleALU() {
  ;
}

template <typename T, typename U> inline void SimpleALU<T,U>::write(T& dst, const T& opoff, const T& src0, const T& src1) {
  T wrt(0);
  for(int i = 0; i < sizeof(T) * 8; i ++) {
    wrt = wrt ^ (src0 & (1 << i) ? top[i * 8 + 8 * opoff * 2] : 0 );
    wrt = wrt ^ (src1 & (1 << i) ? top[i * 8 + 8 * (opoff * 2 + 1)] : 0 );
  }
  q[q.size() - 1] = make_pair(&dst, wrt);
}

template <typename T, typename U> inline void SimpleALU<T,U>::next() {
  if(q[0].first) *(q[0].first) = q[0].second;
  for(int i = 0; i < q.size() - 1; i ++)
    q[i] = q[i + 1];
  q[q.size() - 1] = make_pair(static_cast<T*>(0), T(0));
}


template <typename T, typename U> class MemPage {
public:
  inline MemPage();
  inline ~MemPage();
  inline const T& read(const T& top, const T& idx, const T& addr, const uint8_t& cond) const;
  inline void     write(const T& top, const T& idx, const T& addr, const T& wrt, const uint8_t& cond) const;
  typedef enum {
    READ = 0,
    WRITE = 1,
    EXEC  = 2,
    USER  = 3,
    INT   = 4,
    NOWFLUSH = 5,
    NORFLUSH = 6,
    NOCACHE  = 7
  } m_cond_t;
  typedef struct {
    uint8_t cond;
    T rel;
    T bottom;
    T top;
  } paging_t;
  U m;
};

template <typename T, typename U> inline MemPage<T,U>::MemPage() {
  ;
}

template <typename T, typename U> inline MemPage<T,U>::~MemPage() {
  ;
}

template <typename T, typename U> inline const T& MemPage<T,U>::read(const T& top, const T& idx, const T& addr, const uint8_t& cond) const {
  const auto& pg(reinterpret_cast<const paging_t*>(reinterpret_cast<size_t>(&m.ireg) + sizeof(T) * top));
  assert((pg[idx].cond & cond) == cond);
  const auto  midx(pg[idx].rel + addr);
  assert(pg[idx].bottom <= midx && midx < pg[idx].top);
  return *reinterpret_cast<T*>(reinterpret_cast<size_t>(&m.ireg) + static_cast<size_t>(midx));
}

template <typename T, typename U> inline void MemPage<T,U>::write(const T& top, const T& idx, const T& addr, const T& wrt, const uint8_t& cond) const {
  const auto& pg(reinterpret_cast<const paging_t*>(reinterpret_cast<size_t>(&m.ireg) + sizeof(T) * top));
  assert((pg[idx].cond & cond) == cond);
  const auto  midx(pg[idx].rel + addr);
  assert(pg[idx].bottom <= midx && midx < pg[idx].top);
  *reinterpret_cast<T*>(reinterpret_cast<size_t>(&m.ireg) + static_cast<size_t>(midx)) = wrt;
  return;
}


template <typename T, int pages, typename U> class SimpleMPU {
public:
  typedef struct {
    T rip;
    T reg;
    T irip;
    T ireg;
    T control;
    T interrupt;
    T page;
    uint8_t cond;
    T pending_interrupt;
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
    OP_JMPREL = 15
  } op_e;
  typedef enum {
    INT_DBLINT    = 0,
    INT_INVPRIV   = 1,
    INT_MPU_START = 2,
    INT_USER      = sizeof(T) * 8 - 2,
    INT_HALT      = sizeof(T) * 8 - 1,
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
  vector<SimpleAddr<T,U> > addr;
  vector<SimpleALU<T,U> > alu;
  vector<SimpleALU<T,U> > ialu;
  MemPage<T, U> mem;
  T   pctr;
  int psz;
  
  uint8_t countLSBset(const T& pending) const;
};

template <typename T, int pages, typename U> inline SimpleMPU<T,pages,U>::SimpleMPU() {
  psz = 12;
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

// XXX paging addr pipeline isn't counted.
template <typename T, int pages, typename U> inline void SimpleMPU<T,pages,U>::process() {
  for(int i = 0; i < pu.size(); i ++) {
          auto& p(pu[i]);
    if(pctr < p.pctr) continue;
    p.pctr ++;
    assert(((p.cond & (1 << COND_INTERRUPT)) >> COND_INTERRUPT) ^
           ((p.cond & (1 << COND_USER)) >> COND_USER));
    const auto interrupted(p.cond & (1 << COND_INTERRUPT));
    const static auto mnsz((sizeof(mnemonic_t) + sizeof(T) - 1) / sizeof(T));
    const auto mref(interrupted ? p.irip : p.rip);
    T mbuf[mnsz];
    for(int j = 0; j < mnsz; j ++)
      mbuf[j] = mem.read(p.page, mref >> psz, mref, (1 << MemPage<T,U>::EXEC) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
    const auto& mnemonic(*reinterpret_cast<mnemonic_t*>(mbuf));
    addr[i].next();
    alu[i].next();
    ialu[i].next();
    if(p.pending_interrupt && ! interrupted) {
      const auto pirip(p.interrupt + sizeof(T) * countLSBset(p.pending_interrupt));
      p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
      p.irip  = mem.read(p.page, pirip >> psz, pirip, (1 << MemPage<T,U>::INT) | (1 << MemPage<T,U>::EXEC));
      p.pctr += sizeof(T) * 8 * 2;
    }
    if(p.cond & (1 << COND_HALT)) continue;
    if((mnemonic.cond & p.cond) == mnemonic.cond) {
      const auto  dst0((interrupted ? p.ireg : p.reg) + mnemonic.dst.off);
      const auto  src0((interrupted ? p.ireg : p.reg) + mnemonic.src.off);
      const auto  wrt0((interrupted ? p.ireg : p.reg) + mnemonic.wrt.off);
      const auto& dst1(mnemonic.dst.ref ? mem.read(p.page, dst0 >> psz, dst0, (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)) | (1 << MemPage<T,U>::READ)) : dst0);
      const auto& src1(mnemonic.src.ref ? mem.read(p.page, src0 >> psz, src0, (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)) | (1 << MemPage<T,U>::READ)) : src0);
      const auto& dst(mem.read(p.page, dst1 >> psz, dst1, (1 << MemPage<T,U>::READ) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER))));
      const auto& src(mem.read(p.page, src1 >> psz, src1, (1 << MemPage<T,U>::READ) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER))));
      const auto& pwrt(mnemonic.wrt.ref ? mem.read(p.page, wrt0 >> psz, wrt0, (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER))) : wrt0);
      switch(mnemonic.op & 0x0f) {
      case OP_LDOPTOP:
        p.pctr += sizeof(T) * 8 * 2;
        // (interrupted ? ialu[i] : alu[i]).top = mem.read(p.page, pwrt >> psz, pwrt);
        break;
      case OP_OP:
        //(interrupted ? ialu[i] : alu[i]).write(wrt, mnemonic.opidx, dst, src);
        break;
      case OP_NAND:
        mem.write(p.page, pwrt >> psz, pwrt, ~ (dst & src), (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
        break;
      case OP_PLUS:
        addr[i].write(p.page, pwrt >> psz, pwrt, dst, src, (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
        break;
      case OP_SHIFT:
        mem.write(p.page, pwrt >> psz, pwrt, 0 < src ? dst << src : dst >> - src, (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
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
        mem.write(p.page, pwrt >> psz, pwrt, interrupted ? p.irip : p.rip, (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
        mem.write(p.page, dst1 >> psz, dst1, interrupted ? p.ireg : p.reg, (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
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
        p.pending_interrupt |= T(1) << (mnemonic.opidx == INT_USER || mnemonic.opidx == INT_HALT ? mnemonic.opidx : INT_INVPRIV);
        break;
      case OP_IRET:
        if(interrupted) {
          p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
          p.pctr += sizeof(T) * 8 * 2;
        } else
          p.pending_interrupt |= T(1) << INT_INVPRIV;
        break;
      case OP_LDPAGEINTCONTROL:
        if(interrupted) {
          mem.write(p.page, dst1 >> psz, dst1, p.page, (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
          mem.write(p.page, src1 >> psz, src1, p.interrupt, (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
          mem.write(p.page, pwrt >> psz, pwrt, p.control, (1 << MemPage<T,U>::WRITE) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
        } else
          p.pending_interrupt |= T(1) << INT_INVPRIV;
        break;
      case OP_STPAGEINTCONTROL:
        if(interrupted) {
          p.page      = dst;
          p.interrupt = src;
          p.control   = mem.read(p.page, pwrt >> psz, pwrt, (1 << MemPage<T,U>::READ) | (1 << (interrupted ? MemPage<T,U>::INT : MemPage<T,U>::USER)));
        } else
          p.pending_interrupt |= T(1) << INT_INVPRIV;
        break;
      case OP_CALLPCMP:
        if(i || ! interrupted)
          p.pending_interrupt |= T(1) << INT_INVPRIV;
        else
          mem.m.cmp(*static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                    *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T), (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                    *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 2, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                    *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 3, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                    *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 4, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                    *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 5, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))) );
        break;
      case OP_CALLPNAND:
        if(i || ! interrupted)
          p.pending_interrupt |= T(1) << INT_INVPRIV;
        else
          mem.m.nand(*static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                     *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T), (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                     *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 2, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                     *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 3, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                     *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 4, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                     *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 5, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))),
                     *static_cast<T*>(reinterpret_cast<void*>(size_t(mem.read(p.page, p.ireg >> psz, p.ireg + sizeof(T) * 6, (1 << MemPage<T,U>::READ) | (1 << MemPage<T,U>::WRITE) | (1 << MemPage<T,U>::INT))))) );
        break;
      case OP_JMPREL:
        (interrupted ? p.irip : p.rip) += dst;
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

template <typename T, int pages, typename U> uint8_t SimpleMPU<T,pages,U>::countLSBset(const T& pending) const {
  for(int i = 0; i < sizeof(T) * 8; i ++)
    if((i && (pending & (T(1) << i))) || (! i && (pending & T(1))))
      return i;
  return sizeof(T) * 8;
}

#define _SIMPLE_MPU_
#endif

