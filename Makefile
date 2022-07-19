CXX=	clang++

# compiler flags.
CXXFLAGS+=	-std=c++11
#CXXFLAGS+=	-fopenmp -L/usr/local/lib -lomp
CXXFLAGS+=	-Ofast -mtune=native -gfull
#CXXFLAGS+=	-Ofast -mno-sse2 -mno-sse -mno-3dnow -mno-mmx -msoft-float -gfull
LDFLAGS+=	-lc++

CXXFLAGS+=	-D_FLOAT_BITS_=32
#CXXFLAGS+=	-D_FLOAT_BITS_=64
#CXXFLAGS+=	-D_FLOAT_BITS_=128
#CXXFLAGS+=	-D_FLOAT_BITS_=256
#CXXFLAGS+=	-D_FLOAT_BITS_=512

