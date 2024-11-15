/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) clf_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real float
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[26] = {22, 1, 0, 22, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21};
static const casadi_int casadi_s1[31] = {27, 1, 0, 27, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26};
static const casadi_int casadi_s2[5] = {1, 1, 0, 1, 0};

/* lyapunov_f:(i0[22],i1[27])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a4, a5, a6, a7, a8, a9;
  a0=25.;
  a1=arg[0]? arg[0][6] : 0;
  a2=arg[1]? arg[1][6] : 0;
  a1=(a1-a2);
  a2=(a0*a1);
  a2=(a2*a1);
  a3=arg[0]? arg[0][7] : 0;
  a4=arg[1]? arg[1][7] : 0;
  a3=(a3-a4);
  a4=(a0*a3);
  a4=(a4*a3);
  a2=(a2+a4);
  a4=arg[0]? arg[0][8] : 0;
  a5=arg[1]? arg[1][8] : 0;
  a4=(a4-a5);
  a0=(a0*a4);
  a0=(a0*a4);
  a2=(a2+a0);
  a0=4.0899999999999997e-01;
  a5=arg[0]? arg[0][9] : 0;
  a6=arg[1]? arg[1][9] : 0;
  a5=(a5-a6);
  a6=(a0*a5);
  a6=(a6*a5);
  a7=arg[0]? arg[0][10] : 0;
  a8=arg[1]? arg[1][10] : 0;
  a7=(a7-a8);
  a8=(a0*a7);
  a8=(a8*a7);
  a6=(a6+a8);
  a8=arg[0]? arg[0][11] : 0;
  a9=arg[1]? arg[1][11] : 0;
  a8=(a8-a9);
  a0=(a0*a8);
  a0=(a0*a8);
  a6=(a6+a0);
  a2=(a2+a6);
  a1=(a1*a5);
  a3=(a3*a7);
  a1=(a1+a3);
  a4=(a4*a8);
  a1=(a1+a4);
  a2=(a2+a1);
  a1=7.3857668067226907e+00;
  a4=2.;
  a8=arg[1]? arg[1][13] : 0;
  a3=arg[1]? arg[1][15] : 0;
  a7=(a8*a3);
  a5=arg[1]? arg[1][12] : 0;
  a6=arg[1]? arg[1][14] : 0;
  a0=(a5*a6);
  a7=(a7+a0);
  a7=(a4*a7);
  a0=arg[0]? arg[0][13] : 0;
  a9=arg[0]? arg[0][14] : 0;
  a10=(a0*a9);
  a11=arg[0]? arg[0][12] : 0;
  a12=arg[0]? arg[0][15] : 0;
  a13=(a11*a12);
  a10=(a10-a13);
  a10=(a4*a10);
  a13=(a7*a10);
  a14=(a6*a3);
  a15=(a5*a8);
  a14=(a14-a15);
  a14=(a4*a14);
  a15=casadi_sq(a11);
  a16=casadi_sq(a9);
  a15=(a15+a16);
  a16=casadi_sq(a0);
  a15=(a15-a16);
  a16=casadi_sq(a12);
  a15=(a15-a16);
  a16=(a14*a15);
  a13=(a13+a16);
  a16=casadi_sq(a5);
  a17=casadi_sq(a3);
  a16=(a16+a17);
  a17=casadi_sq(a8);
  a16=(a16-a17);
  a17=casadi_sq(a6);
  a16=(a16-a17);
  a17=(a9*a12);
  a18=(a11*a0);
  a17=(a17+a18);
  a17=(a4*a17);
  a18=(a16*a17);
  a13=(a13+a18);
  a18=(a0*a12);
  a19=(a11*a9);
  a18=(a18+a19);
  a18=(a4*a18);
  a19=(a8*a6);
  a20=(a5*a3);
  a19=(a19-a20);
  a19=(a4*a19);
  a20=(a18*a19);
  a21=(a9*a12);
  a22=(a11*a0);
  a21=(a21-a22);
  a21=(a4*a21);
  a22=casadi_sq(a5);
  a23=casadi_sq(a6);
  a22=(a22+a23);
  a23=casadi_sq(a8);
  a22=(a22-a23);
  a23=casadi_sq(a3);
  a22=(a22-a23);
  a23=(a21*a22);
  a20=(a20+a23);
  a23=casadi_sq(a11);
  a24=casadi_sq(a12);
  a23=(a23+a24);
  a24=casadi_sq(a0);
  a23=(a23-a24);
  a24=casadi_sq(a9);
  a23=(a23-a24);
  a24=(a6*a3);
  a25=(a5*a8);
  a24=(a24+a25);
  a24=(a4*a24);
  a25=(a23*a24);
  a20=(a20+a25);
  a13=(a13-a20);
  a13=(a13/a4);
  a20=(a1*a13);
  a20=(a20*a13);
  a25=casadi_sq(a5);
  a26=casadi_sq(a8);
  a25=(a25+a26);
  a26=casadi_sq(a6);
  a25=(a25-a26);
  a26=casadi_sq(a3);
  a25=(a25-a26);
  a18=(a25*a18);
  a26=(a8*a6);
  a27=(a5*a3);
  a26=(a26+a27);
  a26=(a4*a26);
  a21=(a26*a21);
  a18=(a18+a21);
  a21=(a8*a3);
  a27=(a5*a6);
  a21=(a21-a27);
  a21=(a4*a21);
  a23=(a21*a23);
  a18=(a18+a23);
  a23=casadi_sq(a11);
  a27=casadi_sq(a0);
  a23=(a23+a27);
  a27=casadi_sq(a9);
  a23=(a23-a27);
  a27=casadi_sq(a12);
  a23=(a23-a27);
  a7=(a23*a7);
  a27=(a0*a9);
  a28=(a11*a12);
  a27=(a27+a28);
  a27=(a4*a27);
  a14=(a27*a14);
  a7=(a7+a14);
  a14=(a0*a12);
  a28=(a11*a9);
  a14=(a14-a28);
  a14=(a4*a14);
  a16=(a14*a16);
  a7=(a7+a16);
  a18=(a18-a7);
  a18=(a18/a4);
  a7=(a1*a18);
  a7=(a7*a18);
  a20=(a20+a7);
  a19=(a19*a23);
  a22=(a22*a27);
  a19=(a19+a22);
  a24=(a24*a14);
  a19=(a19+a24);
  a10=(a10*a25);
  a15=(a15*a26);
  a10=(a10+a15);
  a17=(a17*a21);
  a10=(a10+a17);
  a19=(a19-a10);
  a19=(a19/a4);
  a1=(a1*a19);
  a1=(a1*a19);
  a20=(a20+a1);
  a1=2.4039999999999999e-03;
  a10=5.0000000000000000e-01;
  a17=arg[0]? arg[0][16] : 0;
  a21=casadi_sq(a11);
  a15=casadi_sq(a0);
  a21=(a21+a15);
  a15=casadi_sq(a9);
  a21=(a21-a15);
  a15=casadi_sq(a12);
  a21=(a21-a15);
  a15=casadi_sq(a5);
  a26=casadi_sq(a8);
  a15=(a15+a26);
  a26=casadi_sq(a6);
  a15=(a15-a26);
  a26=casadi_sq(a3);
  a15=(a15-a26);
  a26=(a21*a15);
  a25=(a0*a9);
  a24=(a11*a12);
  a25=(a25+a24);
  a25=(a4*a25);
  a24=(a8*a6);
  a14=(a5*a3);
  a24=(a24+a14);
  a24=(a4*a24);
  a14=(a25*a24);
  a26=(a26+a14);
  a14=(a0*a12);
  a22=(a11*a9);
  a14=(a14-a22);
  a14=(a4*a14);
  a22=(a8*a3);
  a27=(a5*a6);
  a22=(a22-a27);
  a22=(a4*a22);
  a27=(a14*a22);
  a26=(a26+a27);
  a27=arg[1]? arg[1][16] : 0;
  a26=(a26*a27);
  a23=(a8*a6);
  a7=(a5*a3);
  a23=(a23-a7);
  a23=(a4*a23);
  a7=(a21*a23);
  a16=casadi_sq(a5);
  a28=casadi_sq(a6);
  a16=(a16+a28);
  a28=casadi_sq(a8);
  a16=(a16-a28);
  a28=casadi_sq(a3);
  a16=(a16-a28);
  a28=(a25*a16);
  a7=(a7+a28);
  a28=(a6*a3);
  a29=(a5*a8);
  a28=(a28+a29);
  a28=(a4*a28);
  a29=(a14*a28);
  a7=(a7+a29);
  a29=arg[1]? arg[1][17] : 0;
  a7=(a7*a29);
  a26=(a26+a7);
  a7=(a8*a3);
  a30=(a5*a6);
  a7=(a7+a30);
  a7=(a4*a7);
  a21=(a21*a7);
  a30=(a6*a3);
  a31=(a5*a8);
  a30=(a30-a31);
  a30=(a4*a30);
  a25=(a25*a30);
  a21=(a21+a25);
  a5=casadi_sq(a5);
  a3=casadi_sq(a3);
  a5=(a5+a3);
  a8=casadi_sq(a8);
  a5=(a5-a8);
  a6=casadi_sq(a6);
  a5=(a5-a6);
  a14=(a14*a5);
  a21=(a21+a14);
  a14=arg[1]? arg[1][18] : 0;
  a21=(a21*a14);
  a26=(a26+a21);
  a17=(a17-a26);
  a26=(a10*a17);
  a1=(a1*a26);
  a1=(a1*a17);
  a26=2.3800000000000002e-03;
  a21=arg[0]? arg[0][17] : 0;
  a6=(a0*a9);
  a8=(a11*a12);
  a6=(a6-a8);
  a6=(a4*a6);
  a8=(a6*a15);
  a3=casadi_sq(a11);
  a25=casadi_sq(a9);
  a3=(a3+a25);
  a25=casadi_sq(a0);
  a3=(a3-a25);
  a25=casadi_sq(a12);
  a3=(a3-a25);
  a25=(a3*a24);
  a8=(a8+a25);
  a25=(a9*a12);
  a31=(a11*a0);
  a25=(a25+a31);
  a25=(a4*a25);
  a31=(a25*a22);
  a8=(a8+a31);
  a8=(a8*a27);
  a31=(a6*a23);
  a32=(a3*a16);
  a31=(a31+a32);
  a32=(a25*a28);
  a31=(a31+a32);
  a31=(a31*a29);
  a8=(a8+a31);
  a6=(a6*a7);
  a3=(a3*a30);
  a6=(a6+a3);
  a25=(a25*a5);
  a6=(a6+a25);
  a6=(a6*a14);
  a8=(a8+a6);
  a21=(a21-a8);
  a8=(a10*a21);
  a26=(a26*a8);
  a26=(a26*a21);
  a1=(a1+a26);
  a26=2.8000000000000000e-03;
  a8=arg[0]? arg[0][18] : 0;
  a6=(a0*a12);
  a25=(a11*a9);
  a6=(a6+a25);
  a6=(a4*a6);
  a15=(a6*a15);
  a25=(a9*a12);
  a3=(a11*a0);
  a25=(a25-a3);
  a4=(a4*a25);
  a24=(a4*a24);
  a15=(a15+a24);
  a11=casadi_sq(a11);
  a12=casadi_sq(a12);
  a11=(a11+a12);
  a0=casadi_sq(a0);
  a11=(a11-a0);
  a9=casadi_sq(a9);
  a11=(a11-a9);
  a22=(a11*a22);
  a15=(a15+a22);
  a15=(a15*a27);
  a23=(a6*a23);
  a16=(a4*a16);
  a23=(a23+a16);
  a28=(a11*a28);
  a23=(a23+a28);
  a23=(a23*a29);
  a15=(a15+a23);
  a6=(a6*a7);
  a4=(a4*a30);
  a6=(a6+a4);
  a11=(a11*a5);
  a6=(a6+a11);
  a6=(a6*a14);
  a15=(a15+a6);
  a8=(a8-a15);
  a15=(a10*a8);
  a26=(a26*a15);
  a26=(a26*a8);
  a1=(a1+a26);
  a20=(a20+a1);
  a1=5.0000000000000003e-02;
  a13=(a1*a13);
  a13=(a13*a17);
  a18=(a1*a18);
  a18=(a18*a21);
  a13=(a13+a18);
  a1=(a1*a19);
  a1=(a1*a8);
  a13=(a13+a1);
  a20=(a20+a13);
  a2=(a2+a20);
  a20=4.2748786407766964e+01;
  a13=arg[0]? arg[0][0] : 0;
  a1=arg[1]? arg[1][0] : 0;
  a13=(a13-a1);
  a1=(a20*a13);
  a10=(a10*a1);
  a10=(a10*a13);
  a1=arg[0]? arg[0][1] : 0;
  a8=arg[1]? arg[1][1] : 0;
  a1=(a1-a8);
  a8=(a20*a1);
  a8=(a8*a1);
  a10=(a10+a8);
  a8=arg[0]? arg[0][2] : 0;
  a19=arg[1]? arg[1][2] : 0;
  a8=(a8-a19);
  a20=(a20*a8);
  a20=(a20*a8);
  a10=(a10+a20);
  a20=4.1199999999999998e-01;
  a19=arg[0]? arg[0][3] : 0;
  a18=arg[1]? arg[1][3] : 0;
  a19=(a19-a18);
  a18=(a20*a19);
  a18=(a18*a19);
  a21=arg[0]? arg[0][4] : 0;
  a17=arg[1]? arg[1][4] : 0;
  a21=(a21-a17);
  a17=(a20*a21);
  a17=(a17*a21);
  a18=(a18+a17);
  a17=arg[0]? arg[0][5] : 0;
  a26=arg[1]? arg[1][5] : 0;
  a17=(a17-a26);
  a20=(a20*a17);
  a20=(a20*a17);
  a18=(a18+a20);
  a10=(a10+a18);
  a13=(a13*a19);
  a1=(a1*a21);
  a13=(a13+a1);
  a8=(a8*a17);
  a13=(a13+a8);
  a10=(a10+a13);
  a2=(a2+a10);
  if (res[0]!=0) res[0][0]=a2;
  return 0;
}

CASADI_SYMBOL_EXPORT int lyapunov_f(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int lyapunov_f_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int lyapunov_f_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void lyapunov_f_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int lyapunov_f_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void lyapunov_f_release(int mem) {
}

CASADI_SYMBOL_EXPORT void lyapunov_f_incref(void) {
}

CASADI_SYMBOL_EXPORT void lyapunov_f_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int lyapunov_f_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int lyapunov_f_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real lyapunov_f_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* lyapunov_f_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* lyapunov_f_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* lyapunov_f_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* lyapunov_f_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int lyapunov_f_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
