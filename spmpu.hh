#if !defined(_SIMPLE_MPU_)

using std::vector;
using std::pair;
using std::make_pair;

template <typename T, typename U, int psize> class Mem {
public:
  typedef enum {
    READ  = 0,
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
  inline Mem() {
    ;
  }
  inline Mem(const int& ncpu) {
    m ^= m;
    pctr ^= pctr;
    for(bs = 0; T(1) << bs != T(0); bs ++);
    bs --;
    lazy.resize(ncpu);
    ptop.resize(ncpu, T(0));
  }
  inline ~Mem() {
    ;
  }
  inline const T& read(const int& pidx, const T& addr, const uint8_t& cond, const bool& ref) const {
    if(addr & (T(1) << bs)) {
      ;
    }
    const auto& pg(*reinterpret_cast<const paging_t*>(static_cast<size_t>(ptop[pidx]) + sizeof(paging_t) * (addr >> (bs - psize))));
    assert(pg.cond & NOCACHE);
    if((pg.cond & cond) != cond) {
      // throw invpriv.
      ;
    }
    const auto  midx(pg.rel + addr);
    if(! (pg.bottom <= midx && midx < pg.top)) {
      // throw invpriv.
      ;
    }
    if(ref)
      return read(pidx, midx, cond, false);
    return *reinterpret_cast<T*>(reinterpret_cast<size_t>(&m) + static_cast<size_t>(midx));
  }
  inline void     write(const int& pidx, const T& addr, const uint8_t& cond, const bool& ref, const T& wrt) {
    if(addr & (T(1) << bs)) {
      ;
    }
    const auto& pg(*reinterpret_cast<const paging_t*>(static_cast<size_t>(ptop[pidx]) + sizeof(paging_t) * (addr >> (bs - psize))));
    assert(pg.cond & NOCACHE);
    if((pg.cond & cond) != (cond | (1 << WRITE))) {
      // throw invpriv.
      ;
    }
    const auto  midx(pg.rel + addr);
    if(! (pg.bottom <= midx && midx < pg.top)) {
      // throw invpriv.
      ;
    }
    if(ref)
      write(pidx, midx, cond, false, wrt);
    else
      *reinterpret_cast<T*>(reinterpret_cast<size_t>(&m) + static_cast<size_t>(midx)) = wrt;
    return;
  }
  inline void     lazyOp(const T& src, const T& dst, const T& wrt, void (*op)(const T& x, const T& y)) {
    ;
  }
  inline void     lazyNext() {
    ;
  }
  inline void nand(const T& dst, const T& src, const T& blksize, const T& cnt, const T& intsize, const T& cond, const T& condoff) {
    assert(0 <= dst && dst < blksize && 0 <= condoff && condoff < blksize &&
           0 < blksize && 0 < cnt);
    assert(0 <= intsize && intsize < blksize);
    static U one(1);
    const auto mask((one << intsize) - one);
    const auto alu(~ (m & (m >> (src - dst))));
    for(T i = 0; i < cnt; i ++)
      if((int(m >> (condoff + dst)) & cond) == cond) {
        m &= mask << (i * blksize + dst);
        m |= alu & (mask << (i * blksize + dst));
      }
    pctr ++;
    return;
  }
  inline void cmp(const T& dst, const T& src, const T& blksize, const T& cnt, const T& intsize, const T& wrt) {
    assert(0 <= dst && dst < blksize &&
           0 < blksize && 0 < cnt);
    assert(0 <= intsize && intsize < blksize);
    static U one(1);
    const auto mask((one << intsize) - one);
    for(int i = 0; i < cnt; i ++) {
      m &= ~ (T(15) << (i * blksize + dst + wrt));
      m |= (((m & mask) == ((m >> (src - dst)) & mask) ?
                T(1) : T(2)) |
               ((m & mask) <  ((m >> (src - dst)) & mask) ?
                T(4) : T(0)) |
               ((m & mask) >  ((m >> (src - dst)) & mask) ?
                T(8) : T(0))) << (i * blksize + dst + wrt);
    }
    pctr ++;
    return;
  }
  U   m;
private:
  int bs;
  int pctr;
  vector<vector<pair<pair<T, T>, pair<T, T (*)(const T&, const T&)> > > > lazy;
  vector<T> ptop;

  // peripheral:
};


template <typename T, int psize, typename U> class SimpleMPU {
public:
  typedef struct {
    T rip;
    T reg;
    T op;
    T irip;
    T ireg;
    T iop;
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
    OP_LDOP   = 0,
    OP_OP     = 1,
    OP_NAND   = 2,
    OP_PLUS   = 3,
    OP_SHIFT  = 4,
    OP_CMP    = 5,
    OP_BMOV   = 6,
    OP_BZERO  = 7,
    OP_JMPREL = 8,
    OP_INT    = 9,
    OP_IRET   = 10,
    OP_LDIPREG = 11,
    OP_STIPREG = 12,
    OP_STPAGEINTCONTROL = 13,
    OP_LDPAGEINTCONTROL = 14,
    OP_CALLPARA = 15,
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
  inline SimpleMPU() {
    pctr ^= pctr;
  }
  inline SimpleMPU(const int& npu) {
    pu.resize(npu);
    mem = Mem<T, U, psize>(npu);
    // initialize ?? or bzero() ed??
  }
  inline ~SimpleMPU() {
    ;
  }
  inline void process() {
    for(int i = 0; i < pu.size(); i ++) {
            auto& p(pu[i]);
      if(pctr < p.pctr) continue;
      p.pctr ++;
      assert(((p.cond & (1 << COND_INTERRUPT)) >> COND_INTERRUPT) ^
             ((p.cond & (1 << COND_USER)) >> COND_USER));
      const auto interrupted(p.cond & (1 << COND_INTERRUPT));
      const static auto mnsz((sizeof(mnemonic_t) + sizeof(T) - 1) / sizeof(T));
      const auto minterrupted(1 << (interrupted ? Mem<T,U,psize>::INT : Mem<T,U,psize>::USER));
      const auto mref(interrupted ? p.irip : p.rip);
      T mbuf[mnsz];
      for(int j = 0; j < mnsz; j ++)
        mbuf[j] = mem.read(i, mref, false, (1 << Mem<T, U, psize>::EXEC) | minterrupted);
      const auto& mnemonic(*reinterpret_cast<mnemonic_t*>(mbuf));
      if(p.pending_interrupt && ! interrupted) {
        const auto pirip(p.interrupt + sizeof(T) * countLSBset(p.pending_interrupt));
        p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
        p.irip  = mem.read(i, pirip, false, (1 << Mem<T, U, psize>::INT) | (1 << Mem<T, U, psize>::EXEC));
        p.pctr += sizeof(T) * 8 * 2;
        continue;
      } else if(p.cond & (1 << COND_HALT)) continue;
      else if((mnemonic.cond & p.cond) == mnemonic.cond) {
        const auto psrc((interrupted ? p.ireg : p.reg) + mnemonic.src.off);
        const auto pdst((interrupted ? p.ireg : p.reg) + mnemonic.dst.off);
        const auto pwrt((interrupted ? p.ireg : p.reg) + mnemonic.wrt.off);
        const auto src(mem.read(i, psrc, mnemonic.src.ref, minterrupted | (1 << Mem<T, U, psize>::READ)));
        const auto dst(mem.read(i, pdst, mnemonic.dst.ref, minterrupted | (1 << Mem<T, U, psize>::READ)));
        switch(mnemonic.op & 0x0f) {
        case OP_LDOP:
          p.pctr += sizeof(T) * 8 * 2;
          (interrupted ? p.iop : p.op) = mem.read(i, psrc, mnemonic.src.ref, minterrupted | (1 << Mem<T, U, psize>::EXEC));
          break;
        case OP_OP:
          //(interrupted ? ialu[i] : alu[i]).write(wrt, mnemonic.opidx, dst, src);
          // stub: wrt := op * [src dst]
          break;
        case OP_NAND:
          // mem.lazyOp(i, , lambda);
          // stub: wrt := ~ (dst & src).
          break;
        case OP_PLUS:
          // mem.lazyOp(i, , lambda);
          // stub: wrt := dst + src.
          break;
        case OP_SHIFT:
          // mem.lazyOp(i, , lambda);
          // stub: wrt := dst << src.
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
        case OP_LDIPREG:
          mem.write(i, pwrt, false, minterrupted,
            (mnemonic.wrt.ref ? p.irip : p.rip));
          mem.write(i, pdst, false, minterrupted,
            (mnemonic.dst.ref ? p.ireg : p.reg));
          break;
        case OP_STIPREG:
          if(! interrupted && (mnemonic.wrt.ref | mnemonic.dst.ref)) {
            // invpriv:
          }
          (mnemonic.wrt.ref ? p.irip : p.rip) =
            mem.read(i, pwrt, false, minterrupted | (1 << Mem<T, U, psize>::READ));
          (mnemonic.wrt.ref ? p.ireg : p.reg) =
            mem.read(i, pdst, false, minterrupted | (1 << Mem<T, U, psize>::READ));
          break;
        case OP_BMOV:
          // block move stub.
          break;
        case OP_BZERO:
          // block zero stub.
          break;
        case OP_JMPREL:
          (interrupted ? p.irip : p.rip) += dst;
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
            mem.write(i, pwrt, mnemonic.wrt.ref, minterrupted, p.control);
            mem.write(i, pdst, mnemonic.dst.ref, minterrupted, p.page);
            mem.write(i, psrc, mnemonic.src.ref, minterrupted, p.interrupt);
          } else
            p.pending_interrupt |= T(1) << INT_INVPRIV;
          break;
        case OP_STPAGEINTCONTROL:
          if(interrupted) {
            p.control   = mem.read(i, pwrt, mnemonic.wrt.ref, (1 << Mem<T, U, psize>::READ) | minterrupted);
            p.page      = dst;
            p.interrupt = src;
          } else
            p.pending_interrupt |= T(1) << INT_INVPRIV;
          break;
        case OP_CALLPARA:
          if(i || ! interrupted)
            p.pending_interrupt |= T(1) << INT_INVPRIV;
          else
            // para call stub:
            ;
          break;
        default:
          assert(0 && "Should not be reached.");;
        }
      }
      p.rip += sizeof(mnemonic_t);
    }
    pctr ++;
    mem.lazyNext();
    return;
  }
  uint8_t countLSBset(const T& pending) const {
    for(int i = 0; i < sizeof(T) * 8; i ++)
      if((i && (pending & (T(1) << i))) || (! i && (pending & T(1))))
        return i;
    return sizeof(T) * 8;
  }
  vector<pu_t> pu;
  Mem<T, U, psize> mem;
  T pctr;
};

#define _SIMPLE_MPU_
#endif

