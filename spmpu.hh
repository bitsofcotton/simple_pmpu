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
  typedef struct {
    T dst;
    T src;
    T wrt;
    bool rdst;
    bool rsrc;
    bool rwrt;
    T minterrupted;
    T invpriv;
    T (*op)(const T&, const T&);
  } lazy_t;
  typedef struct {
    T dst;
    T src;
    T wrt;
    bool rdst;
    bool rsrc;
    bool rwrt;
    T opop;
    T minterrupted;
    T invpriv;
    T (*op)(const T&, const T&, T[]);
  } lazyop_t;
  inline Mem() {
    ;
  }
  inline Mem(const int& ncpu) {
    m ^= m;
    pctr ^= pctr;
    for(bs = 0; T(1) << bs != T(0); bs ++);
    bs --;
    vector<lazy_t> lc0;
    lc0.resize(sizeof(T) * 8);
    lazy.resize(ncpu, lc0);
    vector<lazyop_t> loc0;
    loc0.resize(sizeof(T) * 8 + sizeof(T) * 16);
    lazyop.resize(ncpu, loc0);
    ptop.resize(ncpu, T(0));
    c_read  = T(1) << READ;
    c_write = T(1) << WRITE;
  }
  inline ~Mem() {
    ;
  }
  inline T read(const int& pidx, const T& addr, const bool& ref, const uint8_t& cond, T& rd, const T& invpriv) const {
    const auto& pg(*reinterpret_cast<const paging_t*>(static_cast<size_t>(ptop[pidx]) + sizeof(paging_t) * (addr >> (bs - psize))));
    assert(pg.cond & NOCACHE);
    if((pg.cond & cond) != cond)
      return invpriv;
    const auto  midx(pg.rel + addr);
    if(! (pg.bottom <= midx && midx < pg.top))
      return invpriv;
    if(ref)
      return read(pidx, midx, false, cond, rd, invpriv);
    if(addr & (T(1) << (bs + 1))) {
      // XXX: peripherals here:
      ;
    } else
      rd = *reinterpret_cast<T*>(reinterpret_cast<size_t>(&m) + static_cast<size_t>(midx));
    return 0;
  }
  inline T write(const int& pidx, const T& addr, const bool& ref, const uint8_t& cond, const T& wrt, const T& invpriv) {
    const auto& pg(*reinterpret_cast<const paging_t*>(static_cast<size_t>(ptop[pidx]) + sizeof(paging_t) * (addr >> (bs - psize))));
    assert(pg.cond & NOCACHE);
    if((pg.cond & cond) != (cond | (1 << WRITE)))
      return invpriv;
    const auto  midx(pg.rel + addr);
    if(! (pg.bottom <= midx && midx < pg.top))
      return invpriv;
    if(ref)
      write(pidx, midx, false, cond, wrt, invpriv);
    else {
      if(addr & (T(1) << (bs + 1))) {
        // XXX: peripherals here.
        ;
      } else
        *reinterpret_cast<T*>(reinterpret_cast<size_t>(&m) + static_cast<size_t>(midx)) = wrt;
    }
    return 0;
  }
  inline void lazyOp(const T& core, const T& src, const bool& rsrc, const T& dst, const bool& rdst, const T& wrt, const bool& rwrt, const T& minterrupted, const T& invpriv, T (*op)(const T& x, const T& y)) {
    assert(0 <= core && core < lazy.size());
    auto& last(lazy[core][lazy[core].size() - 1]);
    last.src     = src;
    last.rsrc    = rsrc;
    last.dst     = dst;
    last.rdst    = rdst;
    last.wrt     = wrt;
    last.rwrt    = rwrt;
    last.minterrupted = minterrupted;
    last.invpriv = invpriv;
    last.op      = op;
    return;
  }
  inline void lazyOpOp(const T& core, const T& src, const bool& rsrc, const T& dst, const bool& rdst, const T& wrt, const bool& rwrt, const T& minterrupted, const T& invpriv, T opop, T (*op)(const T& x, const T& y, T opop[])) {
    assert(0 <= core && core < lazy.size());
    auto& last(lazyop[core][lazy[core].size() - 1]);
    last.src     = src;
    last.rsrc    = rsrc;
    last.dst     = dst;
    last.rdst    = rdst;
    last.wrt     = wrt;
    last.rwrt    = rwrt;
    last.minterrupted = minterrupted;
    last.invpriv = invpriv;
    last.opop    = opop;
    last.op      = op;
    return;
  }
  // XXX : cache timing is broken implementation on this (worst case).
  inline T lazyNext(const T& core) {
    assert(0 <= core && core < lazy.size());
    const auto& lc0(lazy[core][0]);
    T res(0);
    T src;
    T dst;
    T wrt(0);
    res |= read(core, lc0.src, lc0.rsrc, lc0.minterrupted | c_read, src, lc0.invpriv);
    if(res) goto ensure;
    res |= read(core, lc0.dst, lc0.rdst, lc0.minterrupted | c_read, dst, lc0.invpriv);
    if(res) goto ensure;
    wrt  = lc0.op(src, dst);
    res |= write(core, lc0.wrt, lc0.rwrt, lc0.minterrupted | c_write, wrt, lc0.invpriv);
   ensure:
    lazy[core].erase(lazy[core].begin());
    static const lazy_t back = {.dst = T(0), .src = T(0), .wrt = T(0),
      .rdst = false, .rsrc = false, .rwrt = false,
      .minterrupted = T(0), .invpriv = T(0),
      .op = (T (*)(const T&, const T&))(void*)0} ;
    lazy[core].emplace_back(back);
    return res;
  }
  inline T lazyNextOp(const T& core) {
    assert(0 <= core && core < lazy.size());
    const auto& lc0(lazyop[core][0]);
    T res(0);
    T src;
    T dst;
    T wrt(0);
    res |= read(core, lc0.src, lc0.rsrc, lc0.minterrupted | c_read, src, lc0.invpriv);
    if(res) goto ensure;
    res |= read(core, lc0.dst, lc0.rdst, lc0.minterrupted | c_read, dst, lc0.invpriv);
    if(res) goto ensure;
    for(int i = 0; i < 16; i ++)
      res |= read(core, lc0.opop + T(i * sizeof(T) * 16 * sizeof(T)), false, lc0.minterrupted | (T(1) << EXEC), wrt, lc0.invpriv);
    if(res) goto ensure;
    wrt  = lc0.op(src, dst, reinterpret_cast<T*>(reinterpret_cast<size_t>(&m) + static_cast<size_t>(lc0.opop)));
    res |= write(core, lc0.wrt, lc0.wrt, lc0.minterrupted | c_write, wrt, lc0.invpriv);
   ensure:
    lazyop[core].erase(lazyop[core].begin());
    static const lazyop_t back = {.dst = T(0), .src = T(0), .wrt = T(0),
      .rdst = false, .rsrc = false, .rwrt = false,
      .opop = T(0), .minterrupted = T(0), .invpriv = T(0),
      .op = (T (*)(const T&, const T&, T[]))(void*)0 } ;
    lazyop[core].emplace_back(back);
    return res;
  }
  inline T nand(const T& vaddr, const T& invpriv) {
    T dst;
    T src;
    T wrt;
    T intsize;
    T blksize;
    T condoff;
    T cnt;
    T res(0);
    const auto cond((1 << INT) | (1 << READ));
    res |= read(0, vaddr, false, cond, dst, invpriv);
    res |= read(0, vaddr + sizeof(T), false, cond, src, invpriv);
    res |= read(0, vaddr + sizeof(T) * 2, false, cond, wrt, invpriv);
    res |= read(0, vaddr + sizeof(T) * 3, false, cond, intsize, invpriv);
    res |= read(0, vaddr + sizeof(T) * 4, false, cond, blksize, invpriv);
    res |= read(0, vaddr + sizeof(T) * 5, false, cond, condoff, invpriv);
    res |= read(0, vaddr + sizeof(T) * 6, false, cond, cnt, invpriv);
    if(res) return res;
    if(!(T(0) <= dst && dst < blksize &&
         T(0) <= src && src < blksize &&
         T(0) <= wrt && wrt < blksize &&
         T(0) <= condoff && condoff < blksize &&
         T(0) < blksize && T(0) < cnt &&
         T(0) <= intsize && intsize < blksize) )
      return res |= invpriv;
    static U one(1);
    const auto mask((one << int(intsize)) - one);
    const auto alu(~ (m & (m >> int(src - dst))));
    for(T i = 0; i < cnt; i ++)
      if((int(m >> int(condoff + dst)) & cond) == cond) {
        m &= mask << int(i * blksize + dst);
        m |= alu & (mask << int(i * blksize + dst));
      }
    pctr ++;
    return res;
  }
  inline T cmp(const T& vaddr, const T& invpriv) {
    T dst;
    T src;
    T wrt;
    T intsize;
    T blksize;
    T cnt;
    T res(0);
    const auto cond((1 << INT) | (1 << READ));
    res |= read(0, vaddr, false, cond, dst, invpriv);
    res |= read(0, vaddr + sizeof(T), false, cond, src, invpriv);
    res |= read(0, vaddr + sizeof(T) * 2, false, cond, wrt, invpriv);
    res |= read(0, vaddr + sizeof(T) * 3, false, cond, intsize, invpriv);
    res |= read(0, vaddr + sizeof(T) * 4, false, cond, blksize, invpriv);
    res |= read(0, vaddr + sizeof(T) * 5, false, cond, cnt, invpriv);
    if(res) return res;
    if(!(T(0) <= dst && dst < blksize &&
         T(0) <= src && src < blksize &&
         T(0) <= wrt && wrt < blksize &&
         T(0) < blksize && T(0) < cnt &&
         T(0) <= intsize && intsize < blksize) )
      return res |= invpriv;
    static U one(1);
    const auto mask((one << int(intsize)) - one);
    for(int i = 0; i < cnt; i ++) {
      const auto work((((m & mask) == ((m >> int(src - dst)) & mask) ?
                T(1) : T(2)) |
               ((m & mask) <  ((m >> int(src - dst)) & mask) ?
                T(4) : T(0)) |
               ((m & mask) >  ((m >> int(src - dst)) & mask) ?
                T(8) : T(0))) << (i * blksize + dst + wrt));
      m &= ~ (mask << int(i * blksize + dst + wrt));
      m |= work;
    }
    pctr ++;
    return res;
  }
  U   m;
private:
  int bs;
  int pctr;
  vector<vector<lazy_t> > lazy;
  vector<vector<lazyop_t> > lazyop;
  vector<T> ptop;
  T   c_read;
  T   c_write;

  // peripheral:
  // disk i/o (pipe/disk), video i/o, audio i/o, network i/o
  // on shadow memory.
  // in hardware, we have to make i/o map by hands or protocols.
};


template <typename T, typename U, int psize> class SimpleMPU {
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
    T dblint;
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
    OP_LDOP = 0,
    OP_OP   = 1,
    OP_NAND = 2,
    OP_PLUS = 3,
    OP_SHIFT = 4,
    OP_CMP  = 5,
    OP_BMOV = 6,
    OP_BZERO = 7,
    OP_JMPREL = 8,
    OP_INT  = 9,
    OP_IRET = 10,
    OP_LDIPREG = 11,
    OP_STIPREG = 12,
    OP_LDPAGEINTCONTROL = 13,
    OP_STPAGEINTCONTROL = 14,
    // N.B. if we work with (memsize)^2.25 core and base system,
    //      we can replace this mnemonic with permutation
    //      they intends the security matter with mnemonic permutation table
    //      and addr xor table.
    OP_CALLPARA = 15
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
    bop.resize(npu, make_pair(T(0), T(0)));
    bsize.resize(npu, T(0));
    mem = Mem<T, U, psize>(npu);
    T mmsb(1);
    while(bool(mmsb)) { msb = mmsb; mmsb <<= 1; }
    for(int i = 0; i < 0x10; i ++)
      refop[i] = i;
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
      const auto minterrupted(1 << (interrupted ? Mem<T,U,psize>::INT : Mem<T,U,psize>::USER));
      const auto mref(interrupted ? p.irip : p.rip);
      const static auto mnsz((sizeof(mnemonic_t) + sizeof(T) - 1) / sizeof(T));
      const auto invpriv(T(1) << (interrupted ? INT_DBLINT : INT_INVPRIV));
      T mbuf[mnsz];
      for(int j = 0; j < mnsz; j ++)
        p.pending_interrupt |= mem.read(i, mref, false,
          minterrupted | (1 << Mem<T, U, psize>::EXEC),
          mbuf[j], invpriv);
      const auto& mnemonic(*reinterpret_cast<mnemonic_t*>(mbuf));
      assert((mnemonic.op & 0x0f) == OP_OP || !mnemonic.opidx);
      if(p.pending_interrupt && ! interrupted) {
        const auto pirip(p.interrupt + sizeof(T) * countLSBset(p.pending_interrupt));
        p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
        p.pending_interrupt |= mem.read(i, pirip, false,
          (1 << Mem<T, U, psize>::INT) | (1 << Mem<T, U, psize>::EXEC),
          p.irip, INT_DBLINT);
        p.pctr += sizeof(T) * 8 * 2;
        continue;
      } else if((p.pending_interrupt & (T(1) << INT_DBLINT)) && interrupted) {
        if(p.dblint != T(0)) {
          p.pending_interrupt ^= p.pending_interrupt;
          p.pending_interrupt |= T(1) << INT_HALT;
          p.cond |= 1 << COND_HALT;
        } else {
          p.dblint = p.irip;
          p.pending_interrupt |= mem.read(i, p.interrupt, false,
            (1 << Mem<T, U, psize>::INT) | (1 << Mem<T, U, psize>::EXEC),
            p.irip, INT_DBLINT);
          p.pending_interrupt &= ~ (T(1) << INT_DBLINT);
        }
        p.pctr += sizeof(T) * 8 * 2;
        continue;
      } else if(p.cond & (1 << COND_HALT))
        p.rip -= sizeof(mnemonic_t);
      else if((mnemonic.cond & p.cond) == mnemonic.cond) {
        const auto psrc((interrupted ? p.ireg : p.reg) + mnemonic.src.off);
        const auto pdst((interrupted ? p.ireg : p.reg) + mnemonic.dst.off);
        const auto pwrt((interrupted ? p.ireg : p.reg) + mnemonic.wrt.off);
        T src;
        T dst;
        const auto rsrc(mem.read(i, psrc, mnemonic.src.ref, minterrupted | (1 << Mem<T, U, psize>::READ), src, invpriv));
        const auto rdst(mem.read(i, pdst, mnemonic.dst.ref, minterrupted | (1 << Mem<T, U, psize>::READ), dst, invpriv));
        switch(refop[mnemonic.op & 0x0f] & 0x0f) {
        case OP_LDOP:
          p.pctr += sizeof(T) * 8 * 2;
          p.pending_interrupt |= mem.read(i, psrc, mnemonic.src.ref,
            minterrupted | (1 << Mem<T, U, psize>::EXEC),
            interrupted ? p.iop : p.op, invpriv);
          break;
        case OP_OP:
          mem.lazyOpOp(i, psrc, mnemonic.src.ref, pdst, mnemonic.dst.ref,
            pwrt, mnemonic.wrt.ref, minterrupted, invpriv,
            T((interrupted ? p.iop : p.op) + sizeof(T) * 16 * sizeof(T) * mnemonic.opidx),
            [](const T& src, const T& dst, T opop[]) -> T {
              T res(0);
              for(int i = 0; i < sizeof(T) * 8; i ++) 
                if(int(i ? (src >> i) : src) & 1)
                  res ^= opop[i];
              for(int i = 0; i < sizeof(T) * 8; i ++)
                if(int(i ? (dst >> i) : dst) & 1)
                  res ^= opop[i + sizeof(T) * 8];
              return res;
            });
          break;
        case OP_NAND:
          mem.lazyOp(i, psrc, mnemonic.src.ref, pdst, mnemonic.dst.ref,
            pwrt, mnemonic.wrt.ref, minterrupted, invpriv,
            [](const T& src, const T& dst) -> T { return ~ (src & dst); });
          break;
        case OP_PLUS:
          mem.lazyOp(i, psrc, mnemonic.src.ref, pdst, mnemonic.dst.ref,
            pwrt, mnemonic.wrt.ref, minterrupted, invpriv,
            [](const T& src, const T& dst) -> T { return src + dst; });
          break;
        case OP_SHIFT:
          mem.lazyOp(i, psrc, mnemonic.src.ref, pdst, mnemonic.dst.ref,
            pwrt, mnemonic.wrt.ref, minterrupted, invpriv,
            [](const T& src, const T& dst) -> T {
              return dst == 0 ? src : (dst < 0 ? src >> (- dst): src << dst);
            });
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
          p.pending_interrupt |= rsrc | rdst;
          break;
        case OP_LDIPREG:
          if(! interrupted && (mnemonic.wrt.ref | mnemonic.dst.ref)) {
            p.pending_interrupt |= invpriv;
            break;
          }
          p.pending_interrupt |= mem.write(i, pwrt, false, minterrupted,
            mnemonic.wrt.ref ? p.irip : p.rip, invpriv);
          p.pending_interrupt |= mem.write(i, pdst, false, minterrupted,
            mnemonic.dst.ref ? p.ireg : p.reg, invpriv);
          break;
        case OP_STIPREG:
          if(! interrupted && (mnemonic.wrt.ref | mnemonic.dst.ref)) {
            p.pending_interrupt |= invpriv;
            break;
          }
          p.pending_interrupt |= mem.read(i, pwrt, false,
            minterrupted | (1 << Mem<T, U, psize>::READ),
            mnemonic.wrt.ref ? p.irip : p.rip, invpriv);
          p.pending_interrupt |= mem.read(i, pdst, false,
            minterrupted | (1 << Mem<T, U, psize>::READ),
            mnemonic.wrt.ref ? p.ireg : p.reg, invpriv);
          break;
        // XXX : memory cache, best case.
        case OP_BMOV:
          if(bop[i].first == T(0) && bop[i].second == T(0) && bsize[i] == T(0)) {
            bop[i].first  = src;
            bop[i].second = dst;
            p.pending_interrupt |= rsrc | rdst |
              mem.read(i, pwrt, mnemonic.wrt.ref,
                minterrupted | (1 << Mem<T, U, psize>::READ),
                bsize[i], invpriv);
            if(bsize[i] & msb)
              bop[i].first = bop[i].second = bsize[i] = T(0);
          }
          if(bop[i].first == T(0) && bop[i].second == T(0) && bsize[i] == T(0))
            break;
          if(bop[i].second < bop[i].first + bsize[i]) {
            T buf(0);
            p.pending_interrupt |= mem.read(i,
              bop[i].first + bsize[i] - sizeof(T), false,
              minterrupted, buf, invpriv);
            p.pending_interrupt |= mem.write(i,
              bop[i].second + bsize[i] - sizeof(T), false,
              minterrupted, buf, invpriv);
          } else {
            T buf(0);
            p.pending_interrupt |= mem.read(i, bop[i].first, false,
              minterrupted, buf, invpriv);
            p.pending_interrupt |= mem.write(i, bop[i].second, false,
              minterrupted, buf, invpriv);
            bop[i].first  += sizeof(T);
            bop[i].second += sizeof(T);
          }
          bsize[i] -= sizeof(T);
          if(bsize[i] == T(0) || bsize[i] & msb) {
            bop[i].first = bop[i].second = bsize[i] = T(0);
            break;
          }
          p.rip -= sizeof(mnemonic_t);
          break;
        case OP_BZERO:
          if(bop[i].first == T(0) && bop[i].second == T(0)) {
            bop[i].first  = src;
            bop[i].second = dst;
            p.pending_interrupt |= rsrc | rdst;
          }
          if(bop[i].first == T(0) && bop[i].second == T(0)) break;
          if(bop[i].second < bop[i].first) {
            bop[i].first = bop[i].second = T(0);
            p.pending_interrupt |= invpriv;
            break;
          }
          p.pending_interrupt |= mem.write(i, bop[i].first, false,
            minterrupted, T(0), invpriv);
          bop[i].first += sizeof(T);
          if(bop[i].second <= bop[i].first) {
            bop[i].first = bop[i].second = T(0);
            break;
          }
          p.rip -= sizeof(mnemonic_t);
          break;
        case OP_JMPREL:
          if(rdst)
            p.pending_interrupt |= rdst;
          else
            (interrupted ? p.irip : p.rip) += dst;
          break;
        case OP_INT:
          if(interrupted)
            p.pending_interrupt |=
              T(1) << (INT_MPU_START <= mnemonic.opidx 
                       ? mnemonic.opidx : invpriv);
          else
            p.pending_interrupt |=
              T(1) << (INT_USER <= mnemonic.opidx == INT_USER
                       ? mnemonic.opidx : invpriv);
          break;
        case OP_IRET:
          if(interrupted) {
            if(p.dblint != T(0)) {
              p.rip = p.dblint;
              p.dblint = T(0);
            } else
              p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
            p.pctr += sizeof(T) * 8 * 2;
          } else
            p.pending_interrupt |= invpriv;
          break;
        case OP_LDPAGEINTCONTROL:
          if(interrupted) {
            p.pending_interrupt |= mem.write(i, pwrt, mnemonic.wrt.ref,
              minterrupted, p.control, invpriv);
            p.pending_interrupt |= mem.write(i, pdst, mnemonic.dst.ref,
              minterrupted, p.page, invpriv);
            p.pending_interrupt |= mem.write(i, psrc, mnemonic.src.ref,
              minterrupted, p.interrupt, invpriv);
          } else
            p.pending_interrupt |= invpriv;
          break;
        case OP_STPAGEINTCONTROL:
          if(interrupted) {
            T work;
            const auto rwrt(mem.read(i, pwrt, mnemonic.wrt.ref,
              minterrupted | (1 << Mem<T, U, psize>::READ),
              work, invpriv));
            if(rwrt)
              p.pending_interrupt |= rwrt;
            else
              p.control   = work;
            if(rdst)
              p.pending_interrupt |= rdst;
            else
              p.page      = dst;
            if(rsrc)
              p.pending_interrupt |= rsrc;
            else
              p.interrupt = src;
          } else
            p.pending_interrupt |= invpriv;
          break;
        case OP_CALLPARA:
          if(i || ! interrupted ||
            mnemonic.wrt.off != T(0) ||
            mnemonic.wrt.ref != T(0) ||
            ! ((mnemonic.src.off == T(0) && mnemonic.src.ref == T(0)) ||
               (mnemonic.dst.off == T(0) && mnemonic.dst.ref == T(0))) ||
            (mnemonic.src.off == T(0) && mnemonic.src.ref == T(0) &&
             mnemonic.dst.off == T(0) && mnemonic.dst.ref == T(0)) )
            p.pending_interrupt |= invpriv;
          else if(mnemonic.src.off == T(0) && mnemonic.src.ref == T(0)) {
            if(rdst)
              p.pending_interrupt |= rdst;
            else
              p.pending_interrupt |= mem.nand(dst, invpriv);
          } else {
            if(rsrc)
              p.pending_interrupt |= rsrc;
            else
              p.pending_interrupt |= mem.cmp(src, invpriv);
          }
          break;
        default:
          assert(0 && "Should not be reached.");;
        }
      }
      p.rip += sizeof(mnemonic_t);
      p.pending_interrupt |= mem.lazyNext(i);
      p.pending_interrupt |= mem.lazyNextOp(i);
    }
    pctr ++;
    return;
  }
  uint8_t countLSBset(const T& pending) const {
    for(int i = 0; i < sizeof(T) * 8; i ++)
      if((i && (pending & (T(1) << i))) || (! i && (pending & T(1))))
        return i;
    return sizeof(T) * 8;
  }
  vector<pu_t> pu;
  vector<pair<T, T> > bop;
  vector<T> bsize;
  Mem<T, U, psize> mem;
  T pctr;
  T msb;
  // XXX: how to make sure not to disclosed to lower layer.
  uint8_t refop[0x10];
};

#define _SIMPLE_MPU_
#endif

