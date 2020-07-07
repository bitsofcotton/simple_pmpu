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

#define _SIMPLE_MPU_
#endif

