#if !defined(_SIMPLE_MPU_)

using std::vector;
using std::pair;
using std::make_pair;

template <typename T> void in_peripherfal(const T& in) {
  return;
}

template <typename T> void out_peripherfal(const T& in) {
  return;
}

// N.B. Array of MISC paradigm.
template <typename T, typename U, int psize> class SimpleMPU {
public:
  typedef enum {
    COND_USER0    = 0,
    COND_USER     = 1,
    COND_EQUAL    = 2,
    COND_NOTEQUAL = 3,
    COND_LESSER   = 4,
    COND_GREATER  = 5,
    COND_INT_MPU  = 6,
    COND_HALT     = 7
  } cond_e;
  typedef enum {
    OP_LDOP = 0,
    OP_CMP_OR_BZERO = 1,
    OP_BMOV = 2,
    OP_JMPREL_OR_INTRET = 3,
    OP_LDIPREG = 4,
    OP_STIPREG = 5,
    OP_LDCRYPT = 6,
    OP_STCRYPT = 7,
    OP_NORMAL_MAX = 8
  } op_e;
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
  // XXX: use programming language to securing program access violations.
  // XXX: all user-procsss is separate.
  typedef struct {
    T rip;
    T reg;
    T rop;
    uint8_t cond;
    mnemonic_t crypt;
    T cryptaddr;
    Mem<T, U, psize> m;
  } pu_t;
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
    T mmsb(1);
    while(bool(mmsb)) { msb = mmsb; mmsb <<= 1; }
    hlt = false;
  }
  inline ~SimpleMPU() {
    ;
  }
  inline T read(const int& idx, const T& addr) {
    // XXX: for R/W operation, mapping is important.
    // for 0xc..., inter-processor memory operations (send/receive).
    // for 0x8... - 0xc..., peripheral.
    // XXX: however, we can treat peripheral as inter-processor operations.
  }
  inline const T& write(const int& idx, const T& addr, const T& wrt) {
  }
  inline void process() {
    if(hlt) return;
    for(int i = 0; i < pu.size(); i ++) {
            auto& p(pu[i]);
      assert(((p.cond & (1 << COND_USER0)) >> COND_USER0) ^
             ((p.cond & (1 << COND_USER )) >> COND_USER ));
      assert(! ((p.cond & (1 << COND_USER0)) ^ (i ? 1 << COND_USER0 : 0)));
      const auto& mnemonic(*reinterpret_cast<mnemonic_t*>(reinterpret_cast<size_T>rip) ^ reinterpret_cast<size_t>(cryptaddr));
      // XXX: cryption mnemonic_t ^= ...:
      if((mnemonic.cond & p.cond) == mnemonic.cond) {
        const auto psrc(read(i, p.reg + mnemonic.src.off));
        const auto pdst(read(i, p.reg + mnemonic.dst.off));
        const auto pwrt(read(i, p.reg + mnemonic.wrt.off));
        const auto rsrc(mnemonic.src.ref ? read(i, psrc) : psrc);
        const auto rdst(mnemonic.dst.ref ? read(i, pdst) : pdst);
        const auto rwrt(mnemonic.wrt.ref ? read(i, pwrt) : pwrt);
        if(mnemonic.op < OP_NORMAL_MAX) siwtch(mnemonic.op) {
        case OP_LDOP:
          p.rop = rsrc;
          break;
        case OP_CMP_OR_BZERO:
          if(mnemonic.wrt.ref || mnemonic.wrt.off) {
            if(bop[i].first == T(0) && bop[i].second == T(0)) {
              bop[i].first  = src;
              bop[i].second = dst;
            }
            if(bop[i].first == T(0) && bop[i].second == T(0)) break;
            if(bop[i].second < bop[i].first) {
              bop[i].first = bop[i].second = T(0);
              break;
            }
            write(i, bop[i].first, T(int(0)));
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
          }
          break;
        case OP_BMOV:
          if(bop[i].first == T(0) && bop[i].second == T(0) && bsize[i] == T(0)) {
            bop[i].first  = src;
            bop[i].second = dst;
            write(i, pwrt, read(i, bsize[i]));
            if(bsize[i] & msb)
              bop[i].first = bop[i].second = bsize[i] = T(0);
          }
          if(bop[i].first == T(0) && bop[i].second == T(0) && bsize[i] == T(0))
            break;
          if(bop[i].second < bop[i].first + bsize[i]) {
            write(i, bop[i].second + bsize[i] - sizeof(T),
              read(i, bop[i].first + bsize[i] - sizeof(T)));
          } else {
            write(i, bop[i].second, read(i, bop[i].first));
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
            p.rip += wrt;
          } else if(mnemonic.dst.ref == 0) {
            if(i) {
              interrupt |= T(int(1)) << i;
              // XXX: pole.
            } else {
              // XXX: pole.
              ;
            }
          } else ; // XXX:
          break;
        case OP_LDIPREG:
          if(i) {
            write(i, pwrt, p.rip);
            write(i, pdst, p.reg);
          } else {
            write(i, pwrt, pu[/**/].rip);
            write(i, pdst, pu[/**/].reg);
          }
          break;
        case OP_STIPREG:
          if(i) {
            p.rip = read(i, pwrt);
            p.reg = read(i, pdst);
          } else {
            pu[/**/].rip = read(i, pwrt);
            pu[/**/].reg = read(i, pdst);
          }
          break;
        case OP_LDCRYPT:
          /**/
          break;
        case OP_STCRYPT:
          /**/
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
  T interrupt;
  bool hlt;
  
  T pctr;
  T msb;
  uint8_t wipe_cryption_key_state;
};

#define _SIMPLE_MPU_
#endif

