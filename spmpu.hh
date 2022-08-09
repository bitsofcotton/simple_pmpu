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
    for(bs = 0; T(1) << bs != T(0); bs ++);
    bs --;
    ptop.resize(ncpu, T(0));
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
  U   m;
private:
  int bs;
  vector<T> ptop;

  // peripheral:
  // disk i/o (pipe/disk), video i/o, audio i/o, network i/o
  // on shadow mory.
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
    uint8_t icond;
    T pending_interrupt;
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
    OP_CMP_OR_BZERO = 1,
    OP_BMOV = 2,
    OP_JMPREL_OR_INTRET = 3,
    OP_LDIPREGCRYPT = 4,
    OP_STIPREGCRYPT = 5,
    OP_LDPAGEINTCONTROL = 6,
    OP_STPAGEINTCONTROL = 7,
    OP_NORMAL_MAX = 8
  } op_e;
  typedef enum {
    INT_DBLINT    = 0,
    INT_INVPRIV   = 1,
    INT_MPU_START = 2,
    INT_USER      = sizeof(T) * 8 - 2,
    INT_HALT      = sizeof(T) * 8 - 1,
  } interrupt_e;
  typedef struct {
    uint16_t off : 15;
    uint16_t ref : 1;
  } operand_t;
  typedef struct {
    uint8_t   cond;
    uint8_t   op;
    operand_t dst;
    operand_t src;
    operand_t wrt;
  } mnemonic_t;
  // XXX: all of atomic operation is through interrupt to CPU#0.
  // XXX: all of parallel operation is through many-core parallel operation
  //      with firmware we implement later.
  inline SimpleMPU() {
    pctr ^= pctr;
  }
  inline SimpleMPU(const int& npu) {
    pu.resize(npu * sizeof(T) * 8);
    bop.resize(pu.size(), make_pair(T(0), T(0)));
    bsize.resize(pu.size(), T(0));
    m = Mem<T, U, psize>(pu.size());
    T mmsb(1);
    while(bool(mmsb)) { msb = mmsb; mmsb <<= 1; }
  }
  inline ~SimpleMPU() {
    ;
  }
  inline void process() {
    for(int i = 0; i < pu.size() / sizeof(T) / 8; i ++) {
            auto& p(pu[i * sizeof(T) * 8 + (pctr % (sizeof(T) * 8))]);
      assert(((p.cond & (1 << COND_INTERRUPT)) >> COND_INTERRUPT) ^
             ((p.cond & (1 << COND_USER)) >> COND_USER));
      const auto interrupted(p.cond & (1 << COND_INTERRUPT));
      const auto minterrupted(1 << (interrupted ? Mem<T,U,psize>::INT : Mem<T,U,psize>::USER));
      const auto mref(interrupted ? p.irip : p.rip);
      const static auto mnsz((sizeof(mnemonic_t) + sizeof(T) - 1) / sizeof(T));
      const auto invpriv(T(1) << (interrupted ? INT_DBLINT : INT_INVPRIV));
      T mbuf[mnsz];
      for(int j = 0; j < mnsz; j ++)
        p.pending_interrupt |= m.read(i, mref, false,
          minterrupted | (1 << Mem<T, U, psize>::EXEC),
          mbuf[j], invpriv);
      // XXX: cryption:
      const auto& mnemonic(*reinterpret_cast<mnemonic_t*>(mbuf));
      if(p.pending_interrupt && ! interrupted) {
        const auto pirip(p.interrupt + sizeof(T) * countLSBset(p.pending_interrupt));
        p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
        p.pending_interrupt |= m.read(i, pirip, false,
          (1 << Mem<T, U, psize>::INT) | (1 << Mem<T, U, psize>::EXEC),
          p.irip, INT_DBLINT);
        continue;
      } else if((p.pending_interrupt & (T(1) << INT_DBLINT)) && interrupted) {
        if(p.dblint != T(0)) {
          p.pending_interrupt ^= p.pending_interrupt;
          p.pending_interrupt |= T(1) << INT_HALT;
          p.cond |= 1 << COND_HALT;
        } else {
          p.dblint = p.irip;
          p.pending_interrupt |= m.read(i, p.interrupt, false,
            (1 << Mem<T, U, psize>::INT) | (1 << Mem<T, U, psize>::EXEC),
            p.irip, INT_DBLINT);
          p.pending_interrupt &= ~ (T(1) << INT_DBLINT);
        }
        continue;
      } else if(p.cond & (1 << COND_HALT)) ;
      else if((mnemonic.cond & p.cond) == mnemonic.cond) {
        const auto psrc((interrupted ? p.ireg : p.reg) + mnemonic.src.off);
        const auto pdst((interrupted ? p.ireg : p.reg) + mnemonic.dst.off);
        const auto pwrt((interrupted ? p.ireg : p.reg) + mnemonic.wrt.off);
        T src;
        T dst;
        T wrt;
        const auto rsrc(m.read(i, psrc, mnemonic.src.ref, minterrupted | (1 << Mem<T, U, psize>::READ), src, invpriv));
        const auto rdst(m.read(i, pdst, mnemonic.dst.ref, minterrupted | (1 << Mem<T, U, psize>::READ), dst, invpriv));
        const auto rwrt(m.read(i, pwrt, mnemonic.wrt.ref, minterrupted | (1 << Mem<T, U, psize>::READ), wrt, invpriv));
        if(mnemonic.op < OP_NORMAL_MAX) switch(mnemonic.op) {
        case OP_LDOP:
          p.pending_interrupt |= m.read(i, psrc, mnemonic.src.ref,
            minterrupted | (1 << Mem<T, U, psize>::EXEC),
            interrupted ? p.iop : p.op, invpriv);
          break;
        case OP_CMP_OR_BZERO:
          if(mnemonic.wrt.ref || mnemonic.wrt.off) {
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
            p.pending_interrupt |= m.write(i, bop[i].first, false,
              minterrupted, T(0), invpriv);
            bop[i].first += sizeof(T);
            if(bop[i].second <= bop[i].first) {
              bop[i].first = bop[i].second = T(0);
              break;
            }
          } else {
            p.cond &= ~ ((1LL << COND_EQUAL)    |
                         (1LL << COND_NOTEQUAL) |
                         (1LL << COND_LESSER)   |
                         (1LL << COND_GREATER));
            p.cond |= (dst == src ? (1LL << COND_EQUAL)
                                  : (1LL << COND_NOTEQUAL))    |
                      (dst <  src ? (1LL << COND_LESSER)  : 0) |
                      (src <  dst ? (1LL << COND_GREATER) : 0);
            p.pending_interrupt |= rsrc | rdst;
          }
          break;
        case OP_BMOV:
          if(bop[i].first == T(0) && bop[i].second == T(0) && bsize[i] == T(0)) {
            bop[i].first  = src;
            bop[i].second = dst;
            p.pending_interrupt |= rsrc | rdst |
              m.read(i, pwrt, mnemonic.wrt.ref,
                minterrupted | (1 << Mem<T, U, psize>::READ),
                bsize[i], invpriv);
            if(bsize[i] & msb)
              bop[i].first = bop[i].second = bsize[i] = T(0);
          }
          if(bop[i].first == T(0) && bop[i].second == T(0) && bsize[i] == T(0))
            break;
          if(bop[i].second < bop[i].first + bsize[i]) {
            T buf(0);
            p.pending_interrupt |= m.read(i,
              bop[i].first + bsize[i] - sizeof(T), false,
              minterrupted | (1 << Mem<T, U, psize>::READ), buf, invpriv);
            p.pending_interrupt |= m.write(i,
              bop[i].second + bsize[i] - sizeof(T), false,
              minterrupted, buf, invpriv);
          } else {
            T buf(0);
            p.pending_interrupt |= m.read(i, bop[i].first, false,
              minterrupted | (1 << Mem<T, U, psize>::READ), buf, invpriv);
            p.pending_interrupt |= m.write(i, bop[i].second, false,
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
        case OP_JMPREL_OR_INTRET:
          if(mnemonic.wrt.ref || mnemonic.wrt.off) {
            if(rwrt)
              p.pending_interrupt |= rwrt;
            else
              (interrupted ? p.irip : p.rip) += wrt;
          } else if(mnemonic.dst.ref == 0) {
            if(mnemonic.src.ref || mnemonic.dst.off) {
              // XXX impl: inter-cpu operator on mnemonic.src - 1 index cpus.
              ;
            } else {
              if(interrupted)
                p.pending_interrupt |=
                  INT_MPU_START <= mnemonic.dst.off ?
                           T(1) << mnemonic.dst.off : invpriv;
              else {
                p.pending_interrupt |=
                   INT_USER <= mnemonic.dst.off == INT_USER ?
                       T(1) << mnemonic.dst.off : invpriv;
                p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
              }
            }
          } else {
            if(interrupted) {
              if(p.dblint != T(0)) {
                p.rip = p.dblint;
                p.dblint = T(0);
              } else
                p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
            } else
              p.pending_interrupt |= invpriv;
          }
          break;
        case OP_LDIPREGCRYPT:
          if(! interrupted && (mnemonic.wrt.ref | mnemonic.dst.ref | mnemonic.src.ref)) {
            p.pending_interrupt |= invpriv;
            break;
          }
          p.pending_interrupt |= m.write(i, pwrt, false, minterrupted,
            mnemonic.wrt.ref ? p.irip : p.rip, invpriv);
          p.pending_interrupt |= m.write(i, pdst, false, minterrupted,
            mnemonic.dst.ref ? p.ireg : p.reg, invpriv);
          // cryption on src:
          break;
        case OP_STIPREGCRYPT:
          if(! interrupted && (mnemonic.wrt.ref | mnemonic.dst.ref | mnemonic.src.ref)) {
            p.pending_interrupt |= invpriv;
            break;
          }
          p.pending_interrupt |= m.read(i, pwrt, false,
            minterrupted | (1 << Mem<T, U, psize>::READ),
            mnemonic.wrt.ref ? p.irip : p.rip, invpriv);
          p.pending_interrupt |= m.read(i, pdst, false,
            minterrupted | (1 << Mem<T, U, psize>::READ),
            mnemonic.wrt.ref ? p.ireg : p.reg, invpriv);
          // cryption on src:
          break;
        case OP_LDPAGEINTCONTROL:
          if(interrupted) {
            p.pending_interrupt |= m.write(i, pwrt, mnemonic.wrt.ref,
              minterrupted, p.control, invpriv);
            p.pending_interrupt |= m.write(i, pdst, mnemonic.dst.ref,
              minterrupted, p.page, invpriv);
            p.pending_interrupt |= m.write(i, psrc, mnemonic.src.ref,
              minterrupted, p.interrupt, invpriv);
          } else
            p.pending_interrupt |= invpriv;
          break;
        case OP_STPAGEINTCONTROL:
          if(interrupted) {
            if(rwrt)
              p.pending_interrupt |= rwrt;
            else
              p.control   = wrt;
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
        default:
          assert(0 && "Should not be reached.");;
        } else {
          ;
          // XXX: implement with counting 1 on the registers
          //      and index operators in ldop:
/*
          T res(int(0));
          for(int i = 0; i < sizeof(T) * 8; i ++) 
            if(int(i ? (src >> i) : src) & 1)
              res ^= opop[i];
          for(int i = 0; i < sizeof(T) * 8; i ++)
            if(int(i ? (dst >> i) : dst) & 1)
              res ^= opop[i + sizeof(T) * 8];
*/
        }
      }
      p.rip += sizeof(mnemonic_t);
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
  Mem<T, U, psize> m;
  T pctr;
  T msb;
  // N.B. all we can do to fight low layers is to put random address
  //      with such encrypted addresses, and relocate them in some interval.
  //      However, even with PIE such vm hosts, if the attacker has its
  //      source codes and automated the attack, we cannot avoid the attack
  //      without on demand password with cryption methods with illegal tick.
  //      To implement them, we should lock the binary with password.
  //      But even so, if the system has a glitches not shown, they're
  //      not guaranteed. Nor, only with the condition the system has
  //      key logger, they either not be guaranteed.
  mnemonic_t uc;
  mnemonic_t ic;
  T umc;
  T imc;
  uint8_t wipe_cryption_key_state;
};

#define _SIMPLE_MPU_
#endif

