#if !defined(_SIMPLE_MPU_)


template <typename T, int bits> class NInt {
public:
  inline NInt();
  inline NInt(const T& src);
  inline ~NInt();
  
  inline NInt<T,bits>& operator ++ ();
  inline NInt<T,bits>  operator ++ (int);
  inline NInt<T,bits>& operator -- ();
  inline NInt<T,bits>  operator -- (int);
  inline NInt<T,bits>  operator -  () const;
  inline NInt<T,bits>  operator +  (const NInt<T,bits>& src) const;
  inline NInt<T,bits>& operator += (const NInt<T,bits>& src);
  inline NInt<T,bits>  operator -  (const NInt<T,bits>& src) const;
  inline NInt<T,bits>& operator -= (const NInt<T,bits>& src);
  inline NInt<T,bits>  operator *  (const NInt<T,bits>& src) const;
  inline NInt<T,bits>& operator *= (const NInt<T,bits>& src);
  inline NInt<T,bits>  operator /  (const NInt<T,bits>& src) const;
  inline NInt<T,bits>& operator /= (const NInt<T,bits>& src);
  inline NInt<T,bits>  operator %  (const NInt<T,bits>& src) const;
  inline NInt<T,bits>& operator %= (const NInt<T,bits>& src);
  inline NInt<T,bits>  operator << ( const int& b)            const;
  inline NInt<T,bits>& operator <<= (const int& b);
  inline NInt<T,bits>  operator >> ( const int& b)            const;
  inline NInt<T,bits>& operator >>= (const int& b);
  inline NInt<T,bits>  operator &  (const NInt<T,bits>& src) const;
  inline NInt<T,bits>& operator &= (const NInt<T,bits>& src);
  inline NInt<T,bits>  operator |  (const NInt<T,bits>& src) const;
  inline NInt<T,bits>& operator |= (const NInt<T,bits>& src);
  inline NInt<T,bits>  operator ^  (const NInt<T,bits>& src) const;
  inline NInt<T,bits>& operator ^= (const NInt<T,bits>& src);
  inline NInt<T,bits>  operator ~  ()                         const;
  inline NInt<T,bits>& operator =  (const NInt<T,bits>& src);
  inline NInt<T,bits>& operator =  (const NInt<NInt<T,bits>,bits*2>& src);
  inline NInt<T,bits>& operator =  (const int& src);
  inline NInt<T,bits>& operator =  (NInt<T,bits>&& src);
  inline bool           operator <  (const NInt<T,bits>& src) const;
  inline bool           operator <= (const NInt<T,bits>& src) const;
  inline bool           operator >  (const NInt<T,bits>& src) const;
  inline bool           operator >= (const NInt<T,bits>& src) const;
  inline bool           operator == (const NInt<T,bits>& src) const;
  inline bool           operator != (const NInt<T,bits>& src) const;
  inline bool           operator && (const NInt<T,bits>& src) const;
  inline bool           operator || (const NInt<T,bits>& src) const;
  inline bool           operator !    () const;
  inline                operator bool () const;
  inline                operator int  () const;
  inline NInt<T,bits>&  alu(const vector<vector<NInt<T,bits> > >& a) const;
};

template <typename T, int bits> inline NInt<T,bits>& NInt<T,bits>::alu(const vector<vector<NInt<T,bits> > >& a) const {
  assert(0 < a.size());
  NInt<T, bits> r(- 1);
  for(int i = 0; i < a.size(); i ++) {
    assert(0 < a[i].size() && a[i].size() <= bits);
    NInt<T, bits> l(a[0]);
    for(int i = 1; i < a.size(); i ++)
      if(int(*this >> (i - 1)) & 1)
        l ^= a[i];
    r &= l;
  }
  return r;
}


template <typename T, int bits> class SimplePMPU {
public:
  inline SimplePMPU();
  inline ~SimplePMPU();
  inline void nand(const int& dst, const int& src, const int& blksize, const int& cnt, const int& dist);
  template <int nits> inline void bid(const NInt<T,nits>& x, const int& start, const int& cnt, const int& dist, const int& offset);
  template <int nits> inline void bmov(const NInt<T,nits>& x, const int& start, const int& cnt, const int& dist);
  template <int nits> inline void bmov(const vector<NInt<T,nits> >& x, const int& start, const int& cnt, const int& dist);
  template <int nits> inline void bmovrev(vector<NInt<T,nits> >& x, const int& start, const int& cnt, const int& dist);
  template <int nits> inline void call(const int& start, const int& cnt, void (*func)(NInt<T,nits>& x));
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


template <typename T, int pages, int ipages> class SimpleMPU {
public:
  typedef struct {
    uint8_t r : 1;
    uint8_t w : 1;
    uint8_t x : 1;
    uint8_t u : 1;
    uint8_t i : 1;
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
    T  rip;
    T  irip;
    T* reg;
    T* ireg;
    T  control;
    uint8_t  cond;
    paging_t page[pages];
    paging_t ipage[pages];
    interrupt_t interrupt[ipages];
  } pu_t;
  typedef enum {
    COND_USER      = 0,
    COND_INTERRUPT = 1,
    COND_EQUAL     = 2,
    COND_NOTEQUAL  = 3,
    COND_LESSER    = 4,
    COND_GREATER   = 5
  } cond_t;
  typedef enum {
    IOP_NOT,
    IOP_PLUS,
    IOP_INC,
    IOP_DEC,
    IOP_XOR,
    IOP_AND,
    IOP_OR,
    IOP_SLEFT,
    IOP_SRIGHT,
    IOP_CMP,
    IOP_LDCONTROL,
    IOP_STCONTROL,
    IOP_LDIP,
    IOP_STIP,
  } int_op_t;
  typedef enum {
    OOP_INTERRUPT,
    OOP_INTERRUPT_MPU,
    OOP_IRET,
    OOP_STPAGE,
    OOP_STINT,
    OOP_NOP,
  } other_op;
  typedef struct {
    uint32_t cond : 6;
    uint32_t type : 1;
    union {
      struct {
        uint32_t op  : 5;
        uint32_t dst : 6;
        uint32_t src : 6;
        uint32_t wrt : 6;
        uint32_t ref : 1;
        uint32_t dummy : 1;
      } int_op;
      struct {
        uint32_t op  : 4;
        uint32_t pad : 1;
        uint32_t wrt : 6;
        uint32_t src : 6;
        uint32_t interrupt : 6;
        uint32_t dummy : 2;
      } other_op;
    } op;
  } mnemonic_t;
  inline SimpleMPU(const int& npu);
  inline ~SimpleMPU();
  inline process();
  std::vector<pu_t> pu;
  int pctr;
};

template <typename T> inline SimpleMPU::SimpleMPU() {
  pctr ^= pctr;
}

template <typename T> inline SimpleMPU::SimpleMPU(const int& npu) {
  pu.resize(npu);
  pctr ^= pctr;
}

template <typename T> inline SimpleMPU::~SimpleMPU() {
  ;
}

template <typename T> inline SimpleMPU::process() {
  for(int i = 0; i < pu.size(); i ++) {
          auto& p(pu[i]);
    const auto& mnemonic(*static_cast<const mnemonic_t*>(&p.rip);
    const auto  interrupted(p.cond & COND_INTERRUPT);
    if(mnemonic.cond & cond == mnemonic.cond) {
      if(mnemonic.type) {
        const auto& oop(mnemonic.oop.other_op);
        if(oop.op == OOP_NOP) {
          ;
        } else if(oop.op == OOP_INTERRUPT) {
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
          case OOP_NOP:
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
        const auto  top(static_cast<T*>(interrupted ? p.ireg : p.reg));
              T*    rtop;
        if(ref) {
          // page ref.
          ;
        }
        const auto& dst(*((ref ? rtop : top) + static_cast<T*>(iop.dst * sizeof(T))));
        const auto& src(*((ref ? rtop : top) + static_cast<T*>(iop.src * sizeof(T))));
        const auto& wrt(*((ref ? rtop : top) + static_cast<T*>(iop.wrt * sizeof(T))));
        switch(iop.op) {
        case IOP_NOT:
          wrt = ~ dst;
          break;
        case IOP_PLUS:
          wrt = dst + src;
          break;
        case IOP_INC:
          wrt = dst;
          ++ wrt;
          break;
        case IOP_DEC:
          wrt = dst;
          -- wrt;
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
        case IOP_CMP:
          
          break;
        case IOP_LDCONTROL:
          if(interrupted)
            wrt = p.control;
          else
            ; // interrupt.
          break;
        case IOP_STCONTROL:
          if(interrupted)
            p.control = dst;
          else
            ; // interrupt.
          break;
        case IOP_LDIP:
          wrt = interrupted ? p.irip : p.rip;
          break;
        case IOP_STIP:
          (interrupted ? p.irip : p.rip) = dst;
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

