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


template <typename T, int pages, int ipages> class SimpleMPU {
public:
  typedef struct {
    uint8_t r : 1;
    uint8_t w : 1;
    uint8_t x : 1;
    uint8_t u : 1;
    uint8_t i : 1;
    uint8_t in  : 1;
    uint8_t out : 1;
    uint8_t reserve : 1;
    T* rel0;
    T* rel1;
  } paging_t;
  typedef struct {
    uint8_t r : 1;
    uint8_t w : 1;
    uint8_t x : 1;
    uint8_t i : 5;
  } interrupt_t;
  typedef struct {
    T rip;
    T irip;
    T reg;
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
    IOP_NOT   = 0,
    IOP_PLUS  = 1,
    IOP_INC   = 2,
    IOP_DEC   = 3,
    IOP_XOR   = 4,
    IOP_AND   = 5,
    IOP_OR    = 6,
    IOP_SLEFT = 7,
    IOP_SRIGHT = 8,
    IOP_BTEST = 9,
    IOP_BXOR  = 10,
    IOP_CMP   = 11,
    IOP_LDIP  = 12,
    IOP_STIP  = 13,
    IOP_NOP   = 14,
    IOP_INVOP = 15
  } int_op_e;
  typedef enum {
    OOP_INTERRUPT = 0,
    OOP_INTERRUPT_MPU = 1,
    OOP_IRET   = 2,
    OOP_STPAGE = 3,
    OOP_STINT  = 4,
    OOP_LDCONTROL = 5,
    OOP_STCONTROL = 6,
    OOP_CALLPARALELL = 7,
  } other_op_e;
  typedef struct {
    uint8_t cond : 7;
    uint8_t type : 1;
    union {
      struct {
        uint32_t op  : 4;
        uint32_t ref : 1;
        uint32_t dst : 9;
        uint32_t src : 9;
        uint32_t wrt : 9;
      } int_op;
      struct {
        uint32_t op  : 3;
        uint32_t ref : 1;
        uint32_t wrt : 9;
        uint32_t src : 9;
        uint32_t interrupt : 5;
        uint32_t dummy : 5;
      } other_op;
    } op;
  } mnemonic_t;
  inline SimpleMPU();
  inline SimpleMPU(const int& npu);
  inline ~SimpleMPU();
  inline void process();
  vector<pu_t> pu;
  vector<SimpleAddr<T> > alu;
  int pctr;
};

template <typename T, int pages, int ipages> inline SimpleMPU<T,pages,ipages>::SimpleMPU() {
  pctr ^= pctr;
}

template <typename T, int pages, int ipages> inline SimpleMPU<T,pages,ipages>::SimpleMPU(const int& npu) {
  pu.resize(npu);
  alu.resize(npu);
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
    alu[i].next();
    if((mnemonic.cond & p.cond) == mnemonic.cond) {
      if(mnemonic.type) {
        const auto& oop(mnemonic.op.other_op);
        if(oop.op == OOP_INTERRUPT) {
          ;
        } else if(interrupted) {
          switch(oop.op) {
          case OOP_INTERRUPT_MPU:
            break;
          case OOP_IRET:
            p.cond ^= (1 << COND_INTERRUPT) | (1 << COND_USER);
            break;
          case OOP_STPAGE:
            break;
          case OOP_STINT:
            break;
          case OOP_LDCONTROL:
            break;
          case OOP_STCONTROL:
            break;
          default:
            // interrupt.
            ;
          }
        } else {
          // interrupt.
          ;
        }
      } else [
        const auto& iop(mnemonic.op.int_op);
        //const auto  top(static_cast<T*>(interrupted ? p.ireg : p.reg));
              T*    rtop(0);
        if(iop.ref) {
          // page ref.
          ;
        }
        const auto& dst(*((iop.ref ? rtop : top) +
                          static_cast<T*>(iop.dst * sizeof(T))));
        const auto& src(*((iop.ref ? rtop : top) +
                          static_cast<T*>(iop.src * sizeof(T))));
        const auto& wrt(*((iop.ref ? rtop : top) +
                          static_cast<T*>(iop.wrt * sizeof(T))));
        switch(iop.op) {
        case IOP_NOT:
          wrt = ~ dst;
          break;
        case IOP_PLUS:
          alu[i].write(wrt, dst, src);
          break;
        case IOP_INC:
          alu[i].write(wrt, dst, 1);
          break;
        case IOP_DEC:
          alu[i].write(wrt, dst, - 1);
          break;
        case IOP_XOR:
          wrt = dst ^ src;
          break;
        case IOP_AND:
          wrt = dst & src;
          break;
        case IOP_OR:
          wrt = dst | src;
          break;
        case IOP_SLEFT:
          wrt = dst << src;
          break;
        case IOP_SRIGHT:
          wrt = dst >> src;
          break;
        case IOP_BTEST:
          //
          break;
        case IOP_BXOR:
          //
          break;
        case IOP_CMP:
          p.cond &= ~ ((1LL << COND_EQUAL)    |
                       (1LL << COND_NOTEQUAL) |
                       (1LL << COND_LESSER)   |
                       (1LL << COND_GREATER));
          p.cond |= (dst == src ? (1LL << COND_EQUAL)
                                : (1LL << COND_NOTEQUAL))    |
                    (dst <  src ? (1LL << COND_LESSER)  : 0) |
                    (src <  dst ? (1LL << COND_GREATER) : 0);
          break;
        case IOP_LDIP:
          wrt = interrupted ? p.irip : p.rip;
          break;
        case IOP_STIP:
          (interrupted ? p.irip : p.rip) = dst;
          break;
        case IOP_NOP:
          break;
        default:
          // interrupt:
          ;
        }
      }
    }
    p.rip += sizeof(mnemonic_t);
  }
  pctr ++;
  return;
}

#define _SIMPLE_MPU_
#endif

