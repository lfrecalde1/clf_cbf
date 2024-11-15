/* This file was automatically generated by CasADi 3.6.7.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) clf_dot_ ## ID
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
#define casadi_s3 CASADI_PREFIX(s3)
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
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[31] = {27, 1, 0, 27, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26};
static const casadi_int casadi_s3[5] = {1, 1, 0, 1, 0};

/* V_dot_f:(i0[22],i1[4],i2[27])->(o0) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a83, a84, a85, a86, a87, a88, a89, a9, a90, a91;
  a0=arg[0]? arg[0][3] : 0;
  a1=arg[2]? arg[2][3] : 0;
  a1=(a0-a1);
  a2=5.0000000000000000e-01;
  a3=4.2748786407766964e+01;
  a4=arg[0]? arg[0][0] : 0;
  a5=arg[2]? arg[2][0] : 0;
  a5=(a4-a5);
  a6=(a3*a5);
  a6=(a2*a6);
  a6=(a1+a6);
  a7=(a2*a5);
  a7=(a3*a7);
  a6=(a6+a7);
  a6=(a6*a0);
  a7=arg[0]? arg[0][4] : 0;
  a8=arg[2]? arg[2][4] : 0;
  a8=(a7-a8);
  a9=arg[0]? arg[0][1] : 0;
  a10=arg[2]? arg[2][1] : 0;
  a10=(a9-a10);
  a11=(a3*a10);
  a11=(a8+a11);
  a12=(a3*a10);
  a11=(a11+a12);
  a11=(a11*a7);
  a6=(a6+a11);
  a11=arg[0]? arg[0][5] : 0;
  a12=arg[2]? arg[2][5] : 0;
  a12=(a11-a12);
  a13=arg[0]? arg[0][2] : 0;
  a14=arg[2]? arg[2][2] : 0;
  a14=(a13-a14);
  a15=(a3*a14);
  a15=(a12+a15);
  a3=(a3*a14);
  a15=(a15+a3);
  a15=(a15*a11);
  a6=(a6+a15);
  a15=4.1199999999999998e-01;
  a3=(a15*a1);
  a16=(a5+a3);
  a17=(a15*a1);
  a16=(a16+a17);
  a17=arg[2]? arg[2][26] : 0;
  a18=1.2591687041564792e-01;
  a19=4.0040000000000003e-01;
  a20=arg[0]? arg[0][7] : 0;
  a21=(a9-a20);
  a22=arg[0]? arg[0][6] : 0;
  a23=(a4-a22);
  a23=casadi_sq(a23);
  a24=(a9-a20);
  a24=casadi_sq(a24);
  a23=(a23+a24);
  a24=arg[0]? arg[0][8] : 0;
  a25=(a13-a24);
  a25=casadi_sq(a25);
  a23=(a23+a25);
  a23=sqrt(a23);
  a21=(a21/a23);
  a25=arg[0]? arg[0][11] : 0;
  a11=(a11-a25);
  a26=5.6000000000000005e-01;
  a11=(a11/a26);
  a27=(a21*a11);
  a28=(a13-a24);
  a28=(a28/a23);
  a29=arg[0]? arg[0][10] : 0;
  a7=(a7-a29);
  a7=(a7/a26);
  a30=(a28*a7);
  a27=(a27-a30);
  a27=casadi_sq(a27);
  a30=arg[0]? arg[0][9] : 0;
  a0=(a0-a30);
  a0=(a0/a26);
  a26=(a28*a0);
  a31=(a4-a22);
  a31=(a31/a23);
  a11=(a31*a11);
  a26=(a26-a11);
  a26=casadi_sq(a26);
  a27=(a27+a26);
  a7=(a31*a7);
  a0=(a21*a0);
  a7=(a7-a0);
  a7=casadi_sq(a7);
  a27=(a27+a7);
  a19=(a19*a27);
  a19=(a18*a19);
  a31=(a19*a31);
  a31=(a17*a31);
  a27=1.0299999999999999e-01;
  a7=(a31/a27);
  a16=(a16*a7);
  a6=(a6-a16);
  a16=(a15*a8);
  a7=(a10+a16);
  a0=(a15*a8);
  a7=(a7+a0);
  a21=(a19*a21);
  a21=(a17*a21);
  a0=(a21/a27);
  a7=(a7*a0);
  a6=(a6-a7);
  a7=(a15*a12);
  a0=(a14+a7);
  a26=(a15*a12);
  a0=(a0+a26);
  a19=(a19*a28);
  a19=(a17*a19);
  a27=(a19/a27);
  a28=9.8100000000000005e+00;
  a27=(a27+a28);
  a0=(a0*a27);
  a6=(a6-a0);
  a0=arg[2]? arg[2][9] : 0;
  a0=(a30-a0);
  a27=25.;
  a26=arg[2]? arg[2][6] : 0;
  a26=(a22-a26);
  a11=(a27*a26);
  a11=(a0+a11);
  a23=(a27*a26);
  a11=(a11+a23);
  a11=(a11*a30);
  a6=(a6+a11);
  a11=arg[2]? arg[2][10] : 0;
  a11=(a29-a11);
  a30=arg[2]? arg[2][7] : 0;
  a30=(a20-a30);
  a23=(a27*a30);
  a23=(a11+a23);
  a32=(a27*a30);
  a23=(a23+a32);
  a23=(a23*a29);
  a6=(a6+a23);
  a23=arg[2]? arg[2][11] : 0;
  a23=(a25-a23);
  a29=arg[2]? arg[2][8] : 0;
  a29=(a24-a29);
  a32=(a27*a29);
  a32=(a23+a32);
  a27=(a27*a29);
  a32=(a32+a27);
  a32=(a32*a25);
  a6=(a6+a32);
  a32=4.0899999999999997e-01;
  a25=(a32*a0);
  a27=(a26+a25);
  a33=(a32*a0);
  a27=(a27+a33);
  a33=7.1499999999999997e-01;
  a31=(a31/a33);
  a27=(a27*a31);
  a6=(a6+a27);
  a27=(a32*a11);
  a31=(a30+a27);
  a34=(a32*a11);
  a31=(a31+a34);
  a21=(a21/a33);
  a31=(a31*a21);
  a6=(a6+a31);
  a31=(a32*a23);
  a21=(a29+a31);
  a34=(a32*a23);
  a21=(a21+a34);
  a19=(a19/a33);
  a19=(a19-a28);
  a21=(a21*a19);
  a6=(a6+a21);
  a21=arg[0]? arg[0][14] : 0;
  a19=2.;
  a28=5.0000000000000003e-02;
  a33=arg[2]? arg[2][13] : 0;
  a34=arg[2]? arg[2][15] : 0;
  a35=(a33*a34);
  a36=arg[2]? arg[2][12] : 0;
  a37=arg[2]? arg[2][14] : 0;
  a38=(a36*a37);
  a35=(a35+a38);
  a35=(a19*a35);
  a38=arg[0]? arg[0][13] : 0;
  a39=(a38*a21);
  a40=arg[0]? arg[0][12] : 0;
  a41=arg[0]? arg[0][15] : 0;
  a42=(a40*a41);
  a39=(a39-a42);
  a39=(a19*a39);
  a42=(a35*a39);
  a43=(a37*a34);
  a44=(a36*a33);
  a43=(a43-a44);
  a43=(a19*a43);
  a44=casadi_sq(a40);
  a45=casadi_sq(a21);
  a44=(a44+a45);
  a45=casadi_sq(a38);
  a44=(a44-a45);
  a45=casadi_sq(a41);
  a44=(a44-a45);
  a45=(a43*a44);
  a42=(a42+a45);
  a45=casadi_sq(a36);
  a46=casadi_sq(a34);
  a45=(a45+a46);
  a46=casadi_sq(a33);
  a45=(a45-a46);
  a46=casadi_sq(a37);
  a45=(a45-a46);
  a46=(a21*a41);
  a47=(a40*a38);
  a46=(a46+a47);
  a46=(a19*a46);
  a47=(a45*a46);
  a42=(a42+a47);
  a47=(a38*a41);
  a48=(a40*a21);
  a47=(a47+a48);
  a47=(a19*a47);
  a48=(a33*a37);
  a49=(a36*a34);
  a48=(a48-a49);
  a48=(a19*a48);
  a49=(a47*a48);
  a50=(a21*a41);
  a51=(a40*a38);
  a50=(a50-a51);
  a50=(a19*a50);
  a51=casadi_sq(a36);
  a52=casadi_sq(a37);
  a51=(a51+a52);
  a52=casadi_sq(a33);
  a51=(a51-a52);
  a52=casadi_sq(a34);
  a51=(a51-a52);
  a52=(a50*a51);
  a49=(a49+a52);
  a52=casadi_sq(a40);
  a53=casadi_sq(a41);
  a52=(a52+a53);
  a53=casadi_sq(a38);
  a52=(a52-a53);
  a53=casadi_sq(a21);
  a52=(a52-a53);
  a53=(a37*a34);
  a54=(a36*a33);
  a53=(a53+a54);
  a53=(a19*a53);
  a54=(a52*a53);
  a49=(a49+a54);
  a42=(a42-a49);
  a42=(a42/a19);
  a49=(a28*a42);
  a54=2.4039999999999999e-03;
  a55=arg[0]? arg[0][16] : 0;
  a56=casadi_sq(a40);
  a57=casadi_sq(a38);
  a56=(a56+a57);
  a57=casadi_sq(a21);
  a56=(a56-a57);
  a57=casadi_sq(a41);
  a56=(a56-a57);
  a57=casadi_sq(a36);
  a58=casadi_sq(a33);
  a57=(a57+a58);
  a58=casadi_sq(a37);
  a57=(a57-a58);
  a58=casadi_sq(a34);
  a57=(a57-a58);
  a58=(a56*a57);
  a59=(a38*a21);
  a60=(a40*a41);
  a59=(a59+a60);
  a59=(a19*a59);
  a60=(a33*a37);
  a61=(a36*a34);
  a60=(a60+a61);
  a60=(a19*a60);
  a61=(a59*a60);
  a58=(a58+a61);
  a61=(a38*a41);
  a62=(a40*a21);
  a61=(a61-a62);
  a61=(a19*a61);
  a62=(a33*a34);
  a63=(a36*a37);
  a62=(a62-a63);
  a62=(a19*a62);
  a63=(a61*a62);
  a58=(a58+a63);
  a63=arg[2]? arg[2][16] : 0;
  a58=(a58*a63);
  a64=(a33*a37);
  a65=(a36*a34);
  a64=(a64-a65);
  a64=(a19*a64);
  a65=(a56*a64);
  a66=casadi_sq(a36);
  a67=casadi_sq(a37);
  a66=(a66+a67);
  a67=casadi_sq(a33);
  a66=(a66-a67);
  a67=casadi_sq(a34);
  a66=(a66-a67);
  a67=(a59*a66);
  a65=(a65+a67);
  a67=(a37*a34);
  a68=(a36*a33);
  a67=(a67+a68);
  a67=(a19*a67);
  a68=(a61*a67);
  a65=(a65+a68);
  a68=arg[2]? arg[2][17] : 0;
  a65=(a65*a68);
  a58=(a58+a65);
  a65=(a33*a34);
  a69=(a36*a37);
  a65=(a65+a69);
  a65=(a19*a65);
  a56=(a56*a65);
  a69=(a37*a34);
  a70=(a36*a33);
  a69=(a69-a70);
  a69=(a19*a69);
  a59=(a59*a69);
  a56=(a56+a59);
  a59=casadi_sq(a36);
  a70=casadi_sq(a34);
  a59=(a59+a70);
  a70=casadi_sq(a33);
  a59=(a59-a70);
  a70=casadi_sq(a37);
  a59=(a59-a70);
  a61=(a61*a59);
  a56=(a56+a61);
  a61=arg[2]? arg[2][18] : 0;
  a56=(a56*a61);
  a58=(a58+a56);
  a58=(a55-a58);
  a56=(a2*a58);
  a56=(a54*a56);
  a70=(a49+a56);
  a71=(a54*a58);
  a71=(a2*a71);
  a70=(a70+a71);
  a71=(a70*a63);
  a72=(a33*a34);
  a73=(a36*a37);
  a72=(a72-a73);
  a72=(a19*a72);
  a73=(a71*a72);
  a74=(a70*a68);
  a75=(a37*a34);
  a76=(a36*a33);
  a75=(a75+a76);
  a75=(a19*a75);
  a76=(a74*a75);
  a73=(a73+a76);
  a76=(a70*a61);
  a77=casadi_sq(a36);
  a78=casadi_sq(a34);
  a77=(a77+a78);
  a78=casadi_sq(a33);
  a77=(a77-a78);
  a78=casadi_sq(a37);
  a77=(a77-a78);
  a78=(a76*a77);
  a73=(a73+a78);
  a73=(a19*a73);
  a78=(a21*a73);
  a79=(a19*a40);
  a80=casadi_sq(a40);
  a81=casadi_sq(a38);
  a80=(a80+a81);
  a81=casadi_sq(a21);
  a80=(a80-a81);
  a81=casadi_sq(a41);
  a80=(a80-a81);
  a48=(a48*a80);
  a81=(a38*a21);
  a82=(a40*a41);
  a81=(a81+a82);
  a81=(a19*a81);
  a51=(a51*a81);
  a48=(a48+a51);
  a51=(a38*a41);
  a82=(a40*a21);
  a51=(a51-a82);
  a51=(a19*a51);
  a53=(a53*a51);
  a48=(a48+a53);
  a53=casadi_sq(a36);
  a82=casadi_sq(a33);
  a53=(a53+a82);
  a82=casadi_sq(a37);
  a53=(a53-a82);
  a82=casadi_sq(a34);
  a53=(a53-a82);
  a39=(a39*a53);
  a82=(a33*a37);
  a83=(a36*a34);
  a82=(a82+a83);
  a82=(a19*a82);
  a44=(a44*a82);
  a39=(a39+a44);
  a44=(a33*a34);
  a83=(a36*a37);
  a44=(a44-a83);
  a44=(a19*a44);
  a46=(a46*a44);
  a39=(a39+a46);
  a48=(a48-a39);
  a48=(a48/a19);
  a39=(a28*a48);
  a46=2.8000000000000000e-03;
  a83=arg[0]? arg[0][18] : 0;
  a84=(a38*a41);
  a85=(a40*a21);
  a84=(a84+a85);
  a84=(a19*a84);
  a85=(a84*a57);
  a86=(a21*a41);
  a87=(a40*a38);
  a86=(a86-a87);
  a86=(a19*a86);
  a87=(a86*a60);
  a85=(a85+a87);
  a87=casadi_sq(a40);
  a88=casadi_sq(a41);
  a87=(a87+a88);
  a88=casadi_sq(a38);
  a87=(a87-a88);
  a88=casadi_sq(a21);
  a87=(a87-a88);
  a88=(a87*a62);
  a85=(a85+a88);
  a85=(a85*a63);
  a88=(a84*a64);
  a89=(a86*a66);
  a88=(a88+a89);
  a89=(a87*a67);
  a88=(a88+a89);
  a88=(a88*a68);
  a85=(a85+a88);
  a84=(a84*a65);
  a86=(a86*a69);
  a84=(a84+a86);
  a87=(a87*a59);
  a84=(a84+a87);
  a84=(a84*a61);
  a85=(a85+a84);
  a85=(a83-a85);
  a84=(a2*a85);
  a84=(a46*a84);
  a87=(a39+a84);
  a86=(a46*a85);
  a86=(a2*a86);
  a87=(a87+a86);
  a86=(a87*a63);
  a88=(a86*a72);
  a89=(a87*a68);
  a90=(a89*a75);
  a88=(a88+a90);
  a90=(a87*a61);
  a91=(a90*a77);
  a88=(a88+a91);
  a79=(a79*a88);
  a53=(a53*a47);
  a82=(a82*a50);
  a53=(a53+a82);
  a44=(a44*a52);
  a53=(a53+a44);
  a80=(a80*a35);
  a81=(a81*a43);
  a80=(a80+a81);
  a51=(a51*a45);
  a80=(a80+a51);
  a53=(a53-a80);
  a53=(a53/a19);
  a80=(a28*a53);
  a51=2.3800000000000002e-03;
  a45=arg[0]? arg[0][17] : 0;
  a81=(a38*a21);
  a43=(a40*a41);
  a81=(a81-a43);
  a81=(a19*a81);
  a57=(a81*a57);
  a43=casadi_sq(a40);
  a35=casadi_sq(a21);
  a43=(a43+a35);
  a35=casadi_sq(a38);
  a43=(a43-a35);
  a35=casadi_sq(a41);
  a43=(a43-a35);
  a60=(a43*a60);
  a57=(a57+a60);
  a60=(a21*a41);
  a35=(a40*a38);
  a60=(a60+a35);
  a60=(a19*a60);
  a62=(a60*a62);
  a57=(a57+a62);
  a57=(a57*a63);
  a64=(a81*a64);
  a66=(a43*a66);
  a64=(a64+a66);
  a67=(a60*a67);
  a64=(a64+a67);
  a64=(a64*a68);
  a57=(a57+a64);
  a81=(a81*a65);
  a43=(a43*a69);
  a81=(a81+a43);
  a60=(a60*a59);
  a81=(a81+a60);
  a81=(a81*a61);
  a57=(a57+a81);
  a57=(a45-a57);
  a81=(a2*a57);
  a81=(a51*a81);
  a60=(a80+a81);
  a59=(a51*a57);
  a59=(a2*a59);
  a60=(a60+a59);
  a63=(a60*a63);
  a72=(a63*a72);
  a68=(a60*a68);
  a75=(a68*a75);
  a72=(a72+a75);
  a61=(a60*a61);
  a77=(a61*a77);
  a72=(a72+a77);
  a72=(a19*a72);
  a77=(a38*a72);
  a79=(a79+a77);
  a78=(a78-a79);
  a79=(a33*a37);
  a77=(a36*a34);
  a79=(a79+a77);
  a79=(a19*a79);
  a77=(a86*a79);
  a75=casadi_sq(a36);
  a59=casadi_sq(a37);
  a75=(a75+a59);
  a59=casadi_sq(a33);
  a75=(a75-a59);
  a59=casadi_sq(a34);
  a75=(a75-a59);
  a59=(a89*a75);
  a77=(a77+a59);
  a59=(a37*a34);
  a43=(a36*a33);
  a59=(a59-a43);
  a59=(a19*a59);
  a43=(a90*a59);
  a77=(a77+a43);
  a77=(a19*a77);
  a43=(a38*a77);
  a78=(a78+a43);
  a43=(a19*a40);
  a69=(a63*a79);
  a65=(a68*a75);
  a69=(a69+a65);
  a65=(a61*a59);
  a69=(a69+a65);
  a43=(a43*a69);
  a78=(a78-a43);
  a79=(a71*a79);
  a75=(a74*a75);
  a79=(a79+a75);
  a59=(a76*a59);
  a79=(a79+a59);
  a79=(a19*a79);
  a59=(a41*a79);
  a78=(a78-a59);
  a59=casadi_sq(a36);
  a75=casadi_sq(a33);
  a59=(a59+a75);
  a75=casadi_sq(a37);
  a59=(a59-a75);
  a75=casadi_sq(a34);
  a59=(a59-a75);
  a86=(a86*a59);
  a75=(a33*a37);
  a43=(a36*a34);
  a75=(a75-a43);
  a75=(a19*a75);
  a89=(a89*a75);
  a86=(a86+a89);
  a89=(a33*a34);
  a43=(a36*a37);
  a89=(a89+a43);
  a89=(a19*a89);
  a90=(a90*a89);
  a86=(a86+a90);
  a86=(a19*a86);
  a90=(a21*a86);
  a78=(a78-a90);
  a63=(a63*a59);
  a68=(a68*a75);
  a63=(a63+a68);
  a61=(a61*a89);
  a63=(a63+a61);
  a63=(a19*a63);
  a61=(a41*a63);
  a78=(a78+a61);
  a61=(a19*a40);
  a71=(a71*a59);
  a74=(a74*a75);
  a71=(a71+a74);
  a76=(a76*a89);
  a71=(a71+a76);
  a61=(a61*a71);
  a78=(a78-a61);
  a61=(a19*a40);
  a76=(a33*a34);
  a89=(a36*a37);
  a76=(a76-a89);
  a76=(a19*a76);
  a89=(a28*a57);
  a74=7.3857668067226907e+00;
  a75=(a74*a53);
  a89=(a89+a75);
  a53=(a74*a53);
  a89=(a89+a53);
  a89=(a2*a89);
  a53=(a76*a89);
  a75=(a28*a58);
  a59=(a74*a42);
  a75=(a75+a59);
  a42=(a74*a42);
  a75=(a75+a42);
  a75=(a2*a75);
  a42=(a37*a34);
  a59=(a36*a33);
  a42=(a42+a59);
  a42=(a19*a42);
  a59=(a75*a42);
  a53=(a53-a59);
  a61=(a61*a53);
  a59=casadi_sq(a36);
  a68=casadi_sq(a34);
  a59=(a59+a68);
  a68=casadi_sq(a33);
  a59=(a59-a68);
  a68=casadi_sq(a37);
  a59=(a59-a68);
  a68=(a59*a75);
  a28=(a28*a85);
  a90=(a74*a48);
  a28=(a28+a90);
  a74=(a74*a48);
  a28=(a28+a74);
  a28=(a2*a28);
  a76=(a28*a76);
  a68=(a68-a76);
  a68=(a19*a68);
  a76=(a38*a68);
  a61=(a61+a76);
  a42=(a42*a28);
  a59=(a89*a59);
  a42=(a42-a59);
  a42=(a19*a42);
  a59=(a21*a42);
  a61=(a61-a59);
  a59=(a33*a37);
  a76=(a36*a34);
  a59=(a59+a76);
  a59=(a19*a59);
  a76=(a59*a89);
  a74=casadi_sq(a36);
  a48=casadi_sq(a37);
  a74=(a74+a48);
  a48=casadi_sq(a33);
  a74=(a74-a48);
  a48=casadi_sq(a34);
  a74=(a74-a48);
  a48=(a75*a74);
  a76=(a76-a48);
  a76=(a19*a76);
  a48=(a38*a76);
  a61=(a61-a48);
  a48=(a19*a40);
  a90=(a37*a34);
  a43=(a36*a33);
  a90=(a90-a43);
  a90=(a19*a90);
  a43=(a90*a75);
  a59=(a28*a59);
  a43=(a43-a59);
  a48=(a48*a43);
  a61=(a61+a48);
  a74=(a74*a28);
  a90=(a89*a90);
  a74=(a74-a90);
  a74=(a19*a74);
  a90=(a41*a74);
  a61=(a61+a90);
  a90=casadi_sq(a36);
  a48=casadi_sq(a33);
  a90=(a90+a48);
  a48=casadi_sq(a37);
  a90=(a90-a48);
  a48=casadi_sq(a34);
  a90=(a90-a48);
  a48=(a90*a89);
  a59=(a33*a37);
  a65=(a36*a34);
  a59=(a59-a65);
  a59=(a19*a59);
  a65=(a75*a59);
  a48=(a48-a65);
  a48=(a19*a48);
  a65=(a21*a48);
  a61=(a61+a65);
  a33=(a33*a34);
  a36=(a36*a37);
  a33=(a33+a36);
  a33=(a19*a33);
  a75=(a33*a75);
  a90=(a28*a90);
  a75=(a75-a90);
  a75=(a19*a75);
  a90=(a41*a75);
  a61=(a61-a90);
  a90=(a19*a40);
  a59=(a59*a28);
  a89=(a89*a33);
  a59=(a59-a89);
  a90=(a90*a59);
  a61=(a61+a90);
  a78=(a78+a61);
  a61=1.;
  a90=casadi_sq(a40);
  a89=casadi_sq(a38);
  a90=(a90+a89);
  a89=casadi_sq(a21);
  a90=(a90+a89);
  a89=casadi_sq(a41);
  a90=(a90+a89);
  a90=sqrt(a90);
  a90=(a61-a90);
  a90=(a19*a90);
  a89=(a90*a40);
  a33=(a2*a55);
  a33=(a33*a38);
  a28=(a2*a45);
  a28=(a28*a21);
  a33=(a33+a28);
  a28=(a2*a83);
  a28=(a28*a41);
  a33=(a33+a28);
  a89=(a89-a33);
  a78=(a78*a89);
  a6=(a6+a78);
  a78=(a19*a38);
  a78=(a78*a88);
  a89=(a40*a72);
  a78=(a78-a89);
  a89=(a41*a73);
  a78=(a78-a89);
  a89=(a40*a77);
  a78=(a78+a89);
  a89=(a19*a38);
  a89=(a89*a69);
  a78=(a78+a89);
  a89=(a21*a79);
  a78=(a78-a89);
  a89=(a41*a86);
  a78=(a78-a89);
  a89=(a21*a63);
  a78=(a78-a89);
  a89=(a19*a38);
  a89=(a89*a71);
  a78=(a78-a89);
  a89=(a40*a68);
  a33=(a19*a38);
  a33=(a33*a53);
  a89=(a89-a33);
  a33=(a41*a42);
  a89=(a89+a33);
  a33=(a40*a76);
  a89=(a89-a33);
  a33=(a19*a38);
  a33=(a33*a43);
  a89=(a89-a33);
  a33=(a21*a74);
  a89=(a89+a33);
  a33=(a41*a48);
  a89=(a89+a33);
  a33=(a21*a75);
  a89=(a89+a33);
  a33=(a19*a38);
  a33=(a33*a59);
  a89=(a89+a33);
  a78=(a78+a89);
  a89=(a2*a55);
  a89=(a89*a40);
  a33=(a2*a83);
  a33=(a33*a21);
  a89=(a89+a33);
  a33=(a2*a45);
  a33=(a33*a41);
  a89=(a89-a33);
  a33=(a90*a38);
  a89=(a89+a33);
  a78=(a78*a89);
  a6=(a6+a78);
  a78=(a19*a21);
  a78=(a78*a88);
  a89=(a41*a72);
  a78=(a78-a89);
  a89=(a40*a73);
  a78=(a78+a89);
  a89=(a41*a77);
  a78=(a78-a89);
  a89=(a19*a21);
  a89=(a89*a69);
  a78=(a78-a89);
  a89=(a38*a79);
  a78=(a78-a89);
  a89=(a40*a86);
  a78=(a78-a89);
  a89=(a38*a63);
  a78=(a78-a89);
  a89=(a19*a21);
  a89=(a89*a71);
  a78=(a78+a89);
  a89=(a41*a68);
  a33=(a19*a21);
  a33=(a33*a53);
  a89=(a89-a33);
  a33=(a40*a42);
  a89=(a89-a33);
  a33=(a41*a76);
  a89=(a89+a33);
  a33=(a19*a21);
  a33=(a33*a43);
  a89=(a89+a33);
  a33=(a38*a74);
  a89=(a89+a33);
  a33=(a40*a48);
  a89=(a89+a33);
  a33=(a38*a75);
  a89=(a89+a33);
  a33=(a19*a21);
  a33=(a33*a59);
  a89=(a89-a33);
  a78=(a78+a89);
  a89=(a2*a45);
  a89=(a89*a40);
  a33=(a2*a83);
  a33=(a33*a38);
  a89=(a89-a33);
  a33=(a2*a55);
  a33=(a33*a41);
  a89=(a89+a33);
  a33=(a90*a21);
  a89=(a89+a33);
  a78=(a78*a89);
  a6=(a6+a78);
  a78=(a19*a41);
  a78=(a78*a69);
  a69=(a19*a41);
  a69=(a69*a88);
  a72=(a21*a72);
  a69=(a69+a72);
  a73=(a38*a73);
  a69=(a69+a73);
  a77=(a21*a77);
  a69=(a69+a77);
  a78=(a78-a69);
  a79=(a40*a79);
  a78=(a78-a79);
  a86=(a38*a86);
  a78=(a78-a86);
  a63=(a40*a63);
  a78=(a78+a63);
  a63=(a19*a41);
  a63=(a63*a71);
  a78=(a78+a63);
  a63=(a19*a41);
  a63=(a63*a53);
  a68=(a21*a68);
  a63=(a63+a68);
  a42=(a38*a42);
  a63=(a63+a42);
  a76=(a21*a76);
  a63=(a63+a76);
  a76=(a19*a41);
  a76=(a76*a43);
  a63=(a63-a76);
  a74=(a40*a74);
  a63=(a63+a74);
  a48=(a38*a48);
  a63=(a63+a48);
  a75=(a40*a75);
  a63=(a63-a75);
  a75=(a19*a41);
  a75=(a75*a59);
  a63=(a63-a75);
  a78=(a78+a63);
  a63=(a2*a83);
  a63=(a63*a40);
  a75=(a2*a45);
  a75=(a75*a38);
  a63=(a63+a75);
  a75=(a2*a55);
  a75=(a75*a21);
  a63=(a63-a75);
  a90=(a90*a41);
  a63=(a63+a90);
  a78=(a78*a63);
  a6=(a6+a78);
  a78=4.1597337770382700e+02;
  a63=(a46*a83);
  a90=(a45*a63);
  a75=(a51*a45);
  a59=(a83*a75);
  a90=(a90-a59);
  a78=(a78*a90);
  a70=(a70*a78);
  a6=(a6-a70);
  a70=4.2016806722689074e+02;
  a78=(a54*a55);
  a83=(a83*a78);
  a63=(a55*a63);
  a83=(a83-a63);
  a70=(a70*a83);
  a60=(a60*a70);
  a6=(a6-a60);
  a60=3.5714285714285717e+02;
  a55=(a55*a75);
  a45=(a45*a78);
  a55=(a55-a45);
  a60=(a60*a55);
  a87=(a87*a60);
  a6=(a6-a87);
  a5=(a5+a3);
  a1=(a15*a1);
  a5=(a5+a1);
  a1=9.7087378640776709e+00;
  a3=(a4-a22);
  a4=(a4-a22);
  a4=casadi_sq(a4);
  a22=(a9-a20);
  a22=casadi_sq(a22);
  a4=(a4+a22);
  a22=(a13-a24);
  a22=casadi_sq(a22);
  a4=(a4+a22);
  a4=sqrt(a4);
  a3=(a3/a4);
  a40=(a19*a40);
  a22=(a40*a21);
  a87=(a41*a38);
  a87=(a19*a87);
  a22=(a22+a87);
  a87=(a3*a22);
  a9=(a9-a20);
  a9=(a9/a4);
  a41=(a41*a21);
  a41=(a19*a41);
  a40=(a40*a38);
  a41=(a41-a40);
  a40=(a9*a41);
  a87=(a87+a40);
  a13=(a13-a24);
  a13=(a13/a4);
  a21=casadi_sq(a21);
  a38=casadi_sq(a38);
  a21=(a21+a38);
  a19=(a19*a21);
  a61=(a61-a19);
  a19=(a13*a61);
  a87=(a87+a19);
  a87=(a18*a87);
  a19=(a3*a87);
  a19=(a17*a19);
  a21=(a1*a19);
  a21=(a5*a21);
  a10=(a10+a16);
  a8=(a15*a8);
  a10=(a10+a8);
  a8=(a9*a87);
  a8=(a17*a8);
  a16=(a1*a8);
  a16=(a10*a16);
  a21=(a21+a16);
  a14=(a14+a7);
  a15=(a15*a12);
  a14=(a14+a15);
  a87=(a13*a87);
  a87=(a17*a87);
  a15=(a1*a87);
  a15=(a14*a15);
  a21=(a21+a15);
  a26=(a26+a25);
  a0=(a32*a0);
  a26=(a26+a0);
  a0=1.3986013986013988e+00;
  a19=(a22-a19);
  a19=(a0*a19);
  a19=(a26*a19);
  a21=(a21+a19);
  a30=(a30+a27);
  a11=(a32*a11);
  a30=(a30+a11);
  a8=(a41-a8);
  a8=(a0*a8);
  a8=(a30*a8);
  a21=(a21+a8);
  a29=(a29+a31);
  a32=(a32*a23);
  a29=(a29+a32);
  a87=(a61-a87);
  a87=(a0*a87);
  a87=(a29*a87);
  a21=(a21+a87);
  a87=4.3677204658901836e+01;
  a49=(a49+a56);
  a54=(a54*a58);
  a54=(a2*a54);
  a49=(a49+a54);
  a54=(a87*a49);
  a21=(a21+a54);
  a54=-3.2941176470588232e+01;
  a80=(a80+a81);
  a51=(a51*a57);
  a51=(a2*a51);
  a80=(a80+a51);
  a51=(a54*a80);
  a21=(a21+a51);
  a51=5.4383116883116891e+00;
  a39=(a39+a84);
  a46=(a46*a85);
  a2=(a2*a46);
  a39=(a39+a2);
  a2=(a51*a39);
  a21=(a21+a2);
  a2=arg[1]? arg[1][0] : 0;
  a21=(a21*a2);
  a2=(a3*a22);
  a46=(a9*a41);
  a2=(a2+a46);
  a46=(a13*a61);
  a2=(a2+a46);
  a2=(a18*a2);
  a46=(a3*a2);
  a46=(a17*a46);
  a85=(a1*a46);
  a85=(a5*a85);
  a84=(a9*a2);
  a84=(a17*a84);
  a57=(a1*a84);
  a57=(a10*a57);
  a85=(a85+a57);
  a2=(a13*a2);
  a2=(a17*a2);
  a57=(a1*a2);
  a57=(a14*a57);
  a85=(a85+a57);
  a46=(a22-a46);
  a46=(a0*a46);
  a46=(a26*a46);
  a85=(a85+a46);
  a84=(a41-a84);
  a84=(a0*a84);
  a84=(a30*a84);
  a85=(a85+a84);
  a2=(a61-a2);
  a2=(a0*a2);
  a2=(a29*a2);
  a85=(a85+a2);
  a87=(a87*a49);
  a85=(a85+a87);
  a87=3.4285714285714285e+01;
  a2=(a87*a80);
  a85=(a85+a2);
  a2=-5.4383116883116891e+00;
  a84=(a2*a39);
  a85=(a85+a84);
  a84=arg[1]? arg[1][1] : 0;
  a85=(a85*a84);
  a21=(a21+a85);
  a85=(a3*a22);
  a84=(a9*a41);
  a85=(a85+a84);
  a84=(a13*a61);
  a85=(a85+a84);
  a85=(a18*a85);
  a84=(a3*a85);
  a84=(a17*a84);
  a46=(a1*a84);
  a46=(a5*a46);
  a57=(a9*a85);
  a57=(a17*a57);
  a81=(a1*a57);
  a81=(a10*a81);
  a46=(a46+a81);
  a85=(a13*a85);
  a85=(a17*a85);
  a81=(a1*a85);
  a81=(a14*a81);
  a46=(a46+a81);
  a84=(a22-a84);
  a84=(a0*a84);
  a84=(a26*a84);
  a46=(a46+a84);
  a57=(a41-a57);
  a57=(a0*a57);
  a57=(a30*a57);
  a46=(a46+a57);
  a85=(a61-a85);
  a85=(a0*a85);
  a85=(a29*a85);
  a46=(a46+a85);
  a85=-4.3677204658901836e+01;
  a57=(a85*a49);
  a46=(a46+a57);
  a87=(a87*a80);
  a46=(a46+a87);
  a51=(a51*a39);
  a46=(a46+a51);
  a51=arg[1]? arg[1][2] : 0;
  a46=(a46*a51);
  a21=(a21+a46);
  a46=(a3*a22);
  a51=(a9*a41);
  a46=(a46+a51);
  a51=(a13*a61);
  a46=(a46+a51);
  a18=(a18*a46);
  a3=(a3*a18);
  a3=(a17*a3);
  a46=(a1*a3);
  a5=(a5*a46);
  a9=(a9*a18);
  a9=(a17*a9);
  a46=(a1*a9);
  a10=(a10*a46);
  a5=(a5+a10);
  a13=(a13*a18);
  a17=(a17*a13);
  a1=(a1*a17);
  a14=(a14*a1);
  a5=(a5+a14);
  a22=(a22-a3);
  a22=(a0*a22);
  a26=(a26*a22);
  a5=(a5+a26);
  a41=(a41-a9);
  a41=(a0*a41);
  a30=(a30*a41);
  a5=(a5+a30);
  a61=(a61-a17);
  a0=(a0*a61);
  a29=(a29*a0);
  a5=(a5+a29);
  a85=(a85*a49);
  a5=(a5+a85);
  a54=(a54*a80);
  a5=(a5+a54);
  a2=(a2*a39);
  a5=(a5+a2);
  a2=arg[1]? arg[1][3] : 0;
  a5=(a5*a2);
  a21=(a21+a5);
  a6=(a6+a21);
  if (res[0]!=0) res[0][0]=a6;
  return 0;
}

CASADI_SYMBOL_EXPORT int V_dot_f(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int V_dot_f_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int V_dot_f_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void V_dot_f_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int V_dot_f_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void V_dot_f_release(int mem) {
}

CASADI_SYMBOL_EXPORT void V_dot_f_incref(void) {
}

CASADI_SYMBOL_EXPORT void V_dot_f_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int V_dot_f_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int V_dot_f_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real V_dot_f_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* V_dot_f_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* V_dot_f_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* V_dot_f_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* V_dot_f_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int V_dot_f_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int V_dot_f_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
