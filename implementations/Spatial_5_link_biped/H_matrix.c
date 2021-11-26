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
  #define CASADI_PREFIX(ID) H_matrix_ ## ID
#endif

#include <math.h>
#include <stdio.h>
#include <string.h>
#ifdef MATLAB_MEX_FILE
#include <mex.h>
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_from_mex CASADI_PREFIX(from_mex)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_to_mex CASADI_PREFIX(to_mex)

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

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

#ifdef MATLAB_MEX_FILE
casadi_real* casadi_from_mex(const mxArray* p, casadi_real* y, const casadi_int* sp, casadi_real* w) {
  casadi_int nrow, ncol, nnz, is_sparse, c, k;
  const casadi_int *colind, *row;
  size_t p_nrow, p_ncol;
  mwIndex *Jc, *Ir;
  const double* p_data;
  if (!mxIsDouble(p) || mxGetNumberOfDimensions(p)!=2)
    mexErrMsgIdAndTxt("Casadi:RuntimeError",
      "\"from_mex\" failed: Not a two-dimensional matrix of double precision.");
  nrow = *sp++;
  ncol = *sp++;
  nnz = sp[ncol];
  colind = sp;
  row = sp+ncol+1;
  p_nrow = mxGetM(p);
  p_ncol = mxGetN(p);
  is_sparse = mxIsSparse(p);
  if (is_sparse) {
#ifndef CASADI_MEX_NO_SPARSE
    Jc = mxGetJc(p);
    Ir = mxGetIr(p);
#else /* CASADI_MEX_NO_SPARSE */
    mexErrMsgIdAndTxt("Casadi:RuntimeError",
      "\"from_mex\" failed: Sparse inputs disabled.");
#endif /* CASADI_MEX_NO_SPARSE */
  }
  p_data = (const double*)mxGetData(p);
  if (p_nrow==1 && p_ncol==1) {
    double v = is_sparse && Jc[1]==0 ? 0 : *p_data;
    casadi_fill(y, nnz, v);
  } else {
    casadi_int tr = 0;
    if (nrow!=p_nrow || ncol!=p_ncol) {
      tr = nrow==p_ncol && ncol==p_nrow && (nrow==1 || ncol==1);
      if (!tr) mexErrMsgIdAndTxt("Casadi:RuntimeError",
                                 "\"from_mex\" failed: Dimension mismatch.");
    }
    if (is_sparse) {
      if (tr) {
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]+c*nrow]=0;
        for (c=0; c<p_ncol; ++c)
          for (k=Jc[c]; k<Jc[c+1]; ++k) w[c+Ir[k]*p_ncol] = p_data[k];
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) y[k] = w[row[k]+c*nrow];
      } else {
        for (c=0; c<ncol; ++c) {
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]]=0;
          for (k=Jc[c]; k<Jc[c+1]; ++k) w[Ir[k]]=p_data[k];
          for (k=colind[c]; k<colind[c+1]; ++k) y[k]=w[row[k]];
        }
      }
    } else {
      for (c=0; c<ncol; ++c) {
        for (k=colind[c]; k<colind[c+1]; ++k) {
          y[k] = p_data[row[k]+c*nrow];
        }
      }
    }
  }
  return y;
}

#endif

#define casadi_to_double(x) ((double) x)

#ifdef MATLAB_MEX_FILE
mxArray* casadi_to_mex(const casadi_int* sp, const casadi_real* x) {
  casadi_int nrow, ncol, nnz, c, k;
  const casadi_int *colind, *row;
  mxArray *p;
  double *d;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int i;
  mwIndex *j;
#endif /* CASADI_MEX_NO_SPARSE */
  nrow = *sp++;
  ncol = *sp++;
  nnz = sp[ncol];
  colind = sp;
  row = sp+ncol+1;
#ifndef CASADI_MEX_NO_SPARSE
  if (nnz!=nrow*ncol) {
    p = mxCreateSparse(nrow, ncol, nnz, mxREAL);
    for (i=0, j=mxGetJc(p); i<=ncol; ++i) *j++ = *colind++;
    for (i=0, j=mxGetIr(p); i<nnz; ++i) *j++ = *row++;
    if (x) {
      d = (double*)mxGetData(p);
      for (i=0; i<nnz; ++i) *d++ = casadi_to_double(*x++);
    }
    return p;
  }
#endif /* CASADI_MEX_NO_SPARSE */
  p = mxCreateDoubleMatrix(nrow, ncol, mxREAL);
  if (x) {
    d = (double*)mxGetData(p);
    for (c=0; c<ncol; ++c) {
      for (k=colind[c]; k<colind[c+1]; ++k) {
        d[row[k]+c*nrow] = casadi_to_double(*x++);
      }
    }
  }
  return p;
}

#endif

#ifndef CASADI_PRINTF
#ifdef MATLAB_MEX_FILE
  #define CASADI_PRINTF mexPrintf
#else
  #define CASADI_PRINTF printf
#endif
#endif

static const casadi_int casadi_s0[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s1[75] = {8, 8, 0, 8, 16, 24, 32, 40, 48, 56, 64, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};

/* f:(i0[8])->(o0[8x8]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a56, a57, a58, a59, a6, a60, a61, a62, a63, a64, a65, a66, a67, a68, a69, a7, a70, a71, a72, a73, a74, a75, a76, a77, a78, a79, a8, a80, a81, a82, a9;
  a0=arg[0]? arg[0][1] : 0;
  a1=sin(a0);
  a2=arg[0]? arg[0][2] : 0;
  a3=cos(a2);
  a4=arg[0]? arg[0][3] : 0;
  a5=cos(a4);
  a6=arg[0]? arg[0][4] : 0;
  a7=cos(a6);
  a8=1.4999999999999999e-01;
  a9=arg[0]? arg[0][5] : 0;
  a10=cos(a9);
  a11=arg[0]? arg[0][6] : 0;
  a12=cos(a11);
  a13=1.0666666666666669e-01;
  a14=arg[0]? arg[0][7] : 0;
  a15=sin(a14);
  a16=(a13*a15);
  a17=(a16*a15);
  a18=(a12*a17);
  a11=sin(a11);
  a14=cos(a14);
  a19=(a13*a14);
  a20=1.6000000000000003e-01;
  a19=(a19+a20);
  a20=(a19*a15);
  a21=(a11*a20);
  a18=(a18+a21);
  a21=(a18*a12);
  a16=(a16*a14);
  a22=-4.0000000000000002e-01;
  a23=(a22*a15);
  a24=(a22*a23);
  a16=(a16+a24);
  a24=(a12*a16);
  a19=(a19*a14);
  a25=(a22*a14);
  a26=-8.0000000000000004e-01;
  a25=(a25+a26);
  a27=(a22*a25);
  a19=(a19+a27);
  a19=(a13+a19);
  a27=(a11*a19);
  a24=(a24+a27);
  a27=(a24*a11);
  a21=(a21+a27);
  a27=(a10*a21);
  a28=(a27*a10);
  a9=sin(a9);
  a29=4.0000000000000002e-01;
  a30=(a29*a14);
  a31=(a29*a30);
  a31=(a13+a31);
  a32=2.;
  a33=(a29*a15);
  a34=(a32*a33);
  a35=(a34*a33);
  a31=(a31+a35);
  a35=(a32*a30);
  a35=(a29+a35);
  a36=(a35*a30);
  a31=(a31+a36);
  a31=(a13+a31);
  a36=(a9*a31);
  a37=(a36*a9);
  a28=(a28+a37);
  a8=(a8+a28);
  a28=(a7*a8);
  a6=sin(a6);
  a37=9.3750000000000000e-02;
  a38=(a10*a31);
  a39=2.5000000000000000e-01;
  a40=(a32*a14);
  a41=(a40*a33);
  a42=(a29*a15);
  a41=(a41-a42);
  a42=(a32*a15);
  a43=(a42*a30);
  a41=(a41-a43);
  a43=(a11*a41);
  a44=(a29*a14);
  a45=(a32*a15);
  a33=(a45*a33);
  a44=(a44+a33);
  a33=(a32*a14);
  a46=(a33*a30);
  a44=(a44+a46);
  a44=(a29+a44);
  a46=(a12*a44);
  a43=(a43+a46);
  a46=(a39*a43);
  a38=(a38+a46);
  a46=(a38*a9);
  a21=(a9*a21);
  a47=(a21*a10);
  a46=(a46-a47);
  a46=(a37+a46);
  a47=(a6*a46);
  a28=(a28-a47);
  a47=(a28*a7);
  a36=(a36*a10);
  a27=(a27*a9);
  a36=(a36-a27);
  a27=(a34*a14);
  a48=(a35*a15);
  a27=(a27-a48);
  a48=(a27*a11);
  a34=(a34*a15);
  a35=(a35*a14);
  a34=(a34+a35);
  a34=(a29+a34);
  a35=(a34*a12);
  a48=(a48+a35);
  a35=(a9*a48);
  a49=(a39*a35);
  a36=(a36+a49);
  a37=(a37+a36);
  a36=(a7*a37);
  a49=1.1979166666666666e-01;
  a21=(a21*a9);
  a38=(a38*a10);
  a21=(a21+a38);
  a48=(a10*a48);
  a38=(a40*a14);
  a50=(a42*a15);
  a38=(a38+a50);
  a38=(a32+a38);
  a50=(a11*a38);
  a51=(a45*a14);
  a52=(a33*a15);
  a51=(a51-a52);
  a52=(a12*a51);
  a50=(a50+a52);
  a52=(a50*a11);
  a40=(a40*a15);
  a42=(a42*a14);
  a40=(a40-a42);
  a42=(a11*a40);
  a45=(a45*a15);
  a33=(a33*a14);
  a45=(a45+a33);
  a45=(a32+a45);
  a33=(a12*a45);
  a42=(a42+a33);
  a33=(a42*a12);
  a52=(a52+a33);
  a33=(a39*a52);
  a48=(a48+a33);
  a33=(a39*a48);
  a21=(a21+a33);
  a49=(a49+a21);
  a21=(a6*a49);
  a36=(a36-a21);
  a21=(a36*a6);
  a47=(a47-a21);
  a21=(a5*a47);
  a4=sin(a4);
  a33=-2.5000000000000000e-01;
  a53=(a33*a10);
  a54=(a22*a15);
  a55=(a54*a12);
  a56=(a22*a14);
  a56=(a56+a26);
  a56=(a22+a56);
  a26=(a56*a11);
  a55=(a55+a26);
  a26=(a53*a55);
  a17=(a11*a17);
  a20=(a12*a20);
  a17=(a17-a20);
  a20=(a17*a12);
  a19=(a12*a19);
  a16=(a11*a16);
  a19=(a19-a16);
  a16=(a19*a11);
  a20=(a20-a16);
  a26=(a26-a20);
  a20=(a26*a10);
  a33=(a33*a9);
  a16=(a12*a41);
  a57=(a11*a44);
  a16=(a16-a57);
  a57=(a33*a16);
  a58=(a57*a9);
  a20=(a20+a58);
  a58=(a20*a7);
  a57=(a57*a10);
  a26=(a26*a9);
  a57=(a57-a26);
  a38=(a12*a38);
  a51=(a11*a51);
  a38=(a38-a51);
  a51=(a38*a11);
  a40=(a12*a40);
  a45=(a11*a45);
  a40=(a40-a45);
  a45=(a40*a12);
  a51=(a51+a45);
  a45=(a33*a51);
  a26=(a39*a45);
  a57=(a57+a26);
  a26=(a57*a6);
  a58=(a58-a26);
  a26=(a4*a58);
  a21=(a21-a26);
  a26=(a21*a5);
  a59=(a12*a23);
  a25=(a22+a25);
  a60=(a11*a25);
  a59=(a59+a60);
  a60=(a10*a59);
  a61=(a60*a53);
  a18=(a18*a11);
  a24=(a24*a12);
  a18=(a18-a24);
  a24=(a10*a18);
  a34=(a34*a11);
  a27=(a27*a12);
  a34=(a34-a27);
  a27=(a9*a34);
  a62=(a27*a33);
  a24=(a24+a62);
  a61=(a61-a24);
  a24=(a7*a61);
  a62=(a9*a18);
  a50=(a50*a12);
  a42=(a42*a11);
  a50=(a50-a42);
  a42=(a39*a50);
  a34=(a10*a34);
  a42=(a42-a34);
  a34=(a42*a33);
  a62=(a62+a34);
  a59=(a9*a59);
  a34=(a59*a53);
  a62=(a62-a34);
  a34=(a6*a62);
  a24=(a24-a34);
  a34=(a5*a24);
  a63=2.6979166666666665e-01;
  a38=(a38*a12);
  a40=(a40*a11);
  a38=(a38-a40);
  a40=(a33*a38);
  a64=(a40*a33);
  a54=(a54*a11);
  a56=(a56*a12);
  a54=(a54-a56);
  a56=(a53*a54);
  a17=(a17*a11);
  a19=(a19*a12);
  a17=(a17+a19);
  a56=(a56-a17);
  a64=(a64-a56);
  a25=(a12*a25);
  a23=(a11*a23);
  a25=(a25-a23);
  a23=4.;
  a56=(a23*a53);
  a25=(a25+a56);
  a56=(a25*a53);
  a64=(a64+a56);
  a63=(a63+a64);
  a64=(a4*a63);
  a34=(a34-a64);
  a64=(a34*a4);
  a26=(a26-a64);
  a64=(a3*a26);
  a2=sin(a2);
  a47=(a4*a47);
  a58=(a5*a58);
  a47=(a47+a58);
  a58=(a10*a55);
  a56=(a58*a10);
  a19=(a9*a16);
  a65=(a19*a9);
  a56=(a56+a65);
  a65=(a7*a56);
  a16=(a10*a16);
  a66=(a16*a9);
  a55=(a9*a55);
  a67=(a55*a10);
  a66=(a66-a67);
  a67=(a6*a66);
  a65=(a65-a67);
  a67=(a65*a7);
  a55=(a55*a9);
  a16=(a16*a10);
  a55=(a55+a16);
  a16=(a10*a51);
  a68=(a39*a16);
  a55=(a55+a68);
  a68=(a6*a55);
  a58=(a58*a9);
  a19=(a19*a10);
  a58=(a58-a19);
  a51=(a9*a51);
  a19=(a39*a51);
  a58=(a58-a19);
  a19=(a7*a58);
  a68=(a68+a19);
  a19=(a68*a6);
  a67=(a67+a19);
  a19=(a22*a67);
  a47=(a47+a19);
  a19=(a47*a5);
  a24=(a4*a24);
  a69=(a5*a63);
  a24=(a24+a69);
  a69=7.5000000000000000e-01;
  a70=(a9*a54);
  a71=(a10*a38);
  a72=(a71*a33);
  a70=(a70+a72);
  a72=(a23*a9);
  a73=(a72*a53);
  a70=(a70-a73);
  a70=(a69+a70);
  a73=(a6*a70);
  a74=8.7500000000000000e-01;
  a75=(a10*a54);
  a38=(a9*a38);
  a76=(a38*a33);
  a75=(a75-a76);
  a23=(a23*a10);
  a76=(a23*a53);
  a75=(a75-a76);
  a75=(a74+a75);
  a76=(a7*a75);
  a73=(a73+a76);
  a76=(a22*a73);
  a24=(a24-a76);
  a76=(a24*a4);
  a19=(a19-a76);
  a76=(a2*a19);
  a64=(a64-a76);
  a76=(a64*a3);
  a21=(a21*a4);
  a34=(a34*a5);
  a21=(a21+a34);
  a34=(a27*a10);
  a77=(a60*a9);
  a34=(a34+a77);
  a77=(a7*a34);
  a78=(a42*a10);
  a79=(a59*a9);
  a78=(a78+a79);
  a79=(a6*a78);
  a77=(a77+a79);
  a79=(a77*a6);
  a27=(a27*a9);
  a60=(a60*a10);
  a27=(a27-a60);
  a60=(a7*a27);
  a59=(a59*a10);
  a42=(a42*a9);
  a59=(a59-a42);
  a42=(a6*a59);
  a60=(a60-a42);
  a42=(a60*a7);
  a79=(a79-a42);
  a42=(a5*a79);
  a80=(a40*a10);
  a81=(a25*a9);
  a80=(a80-a81);
  a69=(a69+a80);
  a80=(a69*a6);
  a40=(a40*a9);
  a25=(a25*a10);
  a40=(a40+a25);
  a74=(a74-a40);
  a40=(a74*a7);
  a80=(a80+a40);
  a40=(a4*a80);
  a42=(a42+a40);
  a40=(a22*a42);
  a21=(a21+a40);
  a40=(a3*a21);
  a47=(a47*a4);
  a24=(a24*a5);
  a47=(a47+a24);
  a79=(a4*a79);
  a80=(a5*a80);
  a79=(a79-a80);
  a80=7.;
  a24=(a71*a10);
  a25=(a72*a9);
  a24=(a24+a25);
  a24=(a80+a24);
  a25=(a6*a24);
  a81=(a23*a9);
  a82=(a38*a10);
  a81=(a81-a82);
  a82=(a7*a81);
  a25=(a25+a82);
  a82=(a25*a6);
  a72=(a72*a10);
  a71=(a71*a9);
  a72=(a72-a71);
  a71=(a6*a72);
  a38=(a38*a9);
  a23=(a23*a10);
  a38=(a38+a23);
  a38=(a80+a38);
  a23=(a7*a38);
  a71=(a71+a23);
  a23=(a71*a7);
  a82=(a82+a23);
  a23=(a22*a82);
  a79=(a79+a23);
  a23=(a22*a79);
  a47=(a47+a23);
  a47=(a13+a47);
  a23=(a2*a47);
  a40=(a40-a23);
  a23=(a40*a2);
  a76=(a76-a23);
  a76=(a1*a76);
  a0=cos(a0);
  a26=(a2*a26);
  a19=(a3*a19);
  a26=(a26+a19);
  a19=(a67*a5);
  a23=(a73*a4);
  a19=(a19+a23);
  a19=(a22*a19);
  a26=(a26+a19);
  a19=(a26*a3);
  a21=(a2*a21);
  a47=(a3*a47);
  a21=(a21+a47);
  a67=(a67*a4);
  a73=(a73*a5);
  a67=(a67-a73);
  a73=(a22*a82);
  a67=(a67+a73);
  a67=(a22+a67);
  a67=(a22*a67);
  a21=(a21+a67);
  a67=(a21*a2);
  a19=(a19-a67);
  a19=(a0*a19);
  a76=(a76+a19);
  a76=(a76*a1);
  a64=(a64*a2);
  a40=(a40*a3);
  a64=(a64+a40);
  a40=(a3*a42);
  a79=(a22+a79);
  a19=(a2*a79);
  a40=(a40-a19);
  a40=(a22*a40);
  a64=(a64+a40);
  a64=(a1*a64);
  a26=(a26*a2);
  a21=(a21*a3);
  a26=(a26+a21);
  a42=(a2*a42);
  a79=(a3*a79);
  a42=(a42+a79);
  a82=(a32+a82);
  a82=(a22*a82);
  a42=(a42+a82);
  a42=(a22*a42);
  a26=(a26+a42);
  a26=(a13+a26);
  a26=(a0*a26);
  a64=(a64+a26);
  a64=(a64*a0);
  a76=(a76+a64);
  if (res[0]!=0) res[0][0]=a76;
  a20=(a20*a6);
  a57=(a57*a7);
  a20=(a20+a57);
  a57=(a4*a20);
  a28=(a28*a6);
  a36=(a36*a7);
  a28=(a28+a36);
  a36=(a5*a28);
  a57=(a57-a36);
  a77=(a77*a7);
  a60=(a60*a6);
  a77=(a77+a60);
  a60=(a5*a77);
  a69=(a69*a7);
  a74=(a74*a6);
  a69=(a69-a74);
  a74=(a4*a69);
  a60=(a60+a74);
  a74=(a29*a4);
  a36=(a60*a74);
  a57=(a57-a36);
  a36=-7.5000000000000000e-01;
  a35=(a36-a35);
  a76=(a7*a35);
  a64=-8.7500000000000000e-01;
  a48=(a64-a48);
  a26=(a6*a48);
  a76=(a76-a26);
  a26=(a5*a76);
  a42=(a4*a45);
  a26=(a26+a42);
  a42=(a29*a5);
  a82=(a26*a42);
  a57=(a57+a82);
  a82=(a3*a57);
  a68=(a68*a7);
  a65=(a65*a6);
  a68=(a68-a65);
  a65=(a22*a68);
  a79=(a4*a28);
  a21=(a5*a20);
  a79=(a79+a21);
  a65=(a65-a79);
  a69=(a5*a69);
  a77=(a4*a77);
  a69=(a69-a77);
  a71=(a71*a6);
  a25=(a25*a7);
  a71=(a71-a25);
  a25=(a22*a71);
  a69=(a69+a25);
  a25=(a69*a74);
  a65=(a65+a25);
  a76=(a4*a76);
  a45=(a5*a45);
  a76=(a76-a45);
  a45=(a6*a16);
  a25=(a7*a51);
  a45=(a45-a25);
  a25=(a22*a45);
  a76=(a76+a25);
  a25=(a76*a42);
  a65=(a65+a25);
  a25=(a2*a65);
  a82=(a82-a25);
  a25=(a60*a5);
  a77=(a26*a4);
  a25=(a25+a77);
  a77=(a3*a25);
  a79=(a69*a5);
  a21=(a76*a4);
  a79=(a79-a21);
  a21=(a2*a79);
  a77=(a77+a21);
  a21=(a29*a2);
  a77=(a77*a21);
  a82=(a82-a77);
  a26=(a26*a5);
  a60=(a60*a4);
  a26=(a26-a60);
  a60=(a3*a26);
  a69=(a69*a4);
  a76=(a76*a5);
  a69=(a69+a76);
  a76=(a2*a69);
  a60=(a60-a76);
  a76=(a29*a3);
  a60=(a60*a76);
  a82=(a82+a60);
  a82=(a1*a82);
  a60=(a2*a57);
  a77=(a3*a65);
  a60=(a60+a77);
  a77=(a71*a74);
  a77=(a68+a77);
  a40=(a45*a42);
  a77=(a77+a40);
  a40=(a22*a77);
  a60=(a60+a40);
  a79=(a3*a79);
  a25=(a2*a25);
  a79=(a79-a25);
  a25=(a71*a5);
  a40=(a45*a4);
  a25=(a25-a40);
  a25=(a22*a25);
  a79=(a79+a25);
  a79=(a79*a21);
  a60=(a60+a79);
  a26=(a2*a26);
  a69=(a3*a69);
  a26=(a26+a69);
  a71=(a71*a4);
  a45=(a45*a5);
  a71=(a71+a45);
  a71=(a22*a71);
  a26=(a26+a71);
  a26=(a26*a76);
  a60=(a60+a26);
  a60=(a0*a60);
  a82=(a82+a60);
  if (res[0]!=0) res[0][1]=a82;
  a60=(a3*a57);
  a26=(a2*a65);
  a60=(a60-a26);
  a60=(a1*a60);
  a57=(a2*a57);
  a65=(a3*a65);
  a57=(a57+a65);
  a77=(a22*a77);
  a57=(a57+a77);
  a57=(a0*a57);
  a60=(a60+a57);
  if (res[0]!=0) res[0][2]=a60;
  a57=(a4*a20);
  a77=(a5*a28);
  a57=(a57-a77);
  a77=(a3*a57);
  a65=(a22*a68);
  a28=(a4*a28);
  a20=(a5*a20);
  a28=(a28+a20);
  a65=(a65-a28);
  a28=(a2*a65);
  a77=(a77-a28);
  a77=(a1*a77);
  a57=(a2*a57);
  a65=(a3*a65);
  a57=(a57+a65);
  a68=(a22*a68);
  a57=(a57+a68);
  a57=(a0*a57);
  a77=(a77+a57);
  if (res[0]!=0) res[0][3]=a77;
  a57=(a7*a61);
  a68=(a6*a62);
  a57=(a57-a68);
  a68=(a5*a57);
  a65=(a4*a63);
  a68=(a68-a65);
  a65=(a3*a68);
  a57=(a4*a57);
  a28=(a5*a63);
  a57=(a57+a28);
  a28=(a6*a70);
  a20=(a7*a75);
  a28=(a28+a20);
  a20=(a22*a28);
  a57=(a57-a20);
  a20=(a2*a57);
  a65=(a65-a20);
  a65=(a1*a65);
  a68=(a2*a68);
  a57=(a3*a57);
  a68=(a68+a57);
  a28=(a22*a28);
  a68=(a68-a28);
  a68=(a0*a68);
  a65=(a65+a68);
  if (res[0]!=0) res[0][4]=a65;
  a68=(a10*a18);
  a28=(a7*a68);
  a18=(a9*a18);
  a57=(a6*a18);
  a28=(a28+a57);
  a57=(a5*a28);
  a53=(a53*a54);
  a53=(a53-a17);
  a20=(a4*a53);
  a57=(a57-a20);
  a20=(a3*a57);
  a28=(a4*a28);
  a26=(a5*a53);
  a28=(a28+a26);
  a26=(a9*a54);
  a71=(a6*a26);
  a54=(a10*a54);
  a45=(a7*a54);
  a71=(a71+a45);
  a45=(a22*a71);
  a28=(a28+a45);
  a45=(a2*a28);
  a20=(a20-a45);
  a20=(a1*a20);
  a57=(a2*a57);
  a28=(a3*a28);
  a57=(a57+a28);
  a71=(a22*a71);
  a57=(a57+a71);
  a57=(a0*a57);
  a20=(a20+a57);
  if (res[0]!=0) res[0][5]=a20;
  a57=(a9*a31);
  a71=(a7*a57);
  a28=(a10*a31);
  a45=(a11*a41);
  a69=(a12*a44);
  a45=(a45+a69);
  a69=(a39*a45);
  a28=(a28+a69);
  a69=(a6*a28);
  a71=(a71-a69);
  a69=(a5*a71);
  a41=(a12*a41);
  a44=(a11*a44);
  a41=(a41-a44);
  a44=(a33*a41);
  a79=(a4*a44);
  a69=(a69-a79);
  a79=(a3*a69);
  a71=(a4*a71);
  a25=(a5*a44);
  a71=(a71+a25);
  a25=(a9*a41);
  a40=(a7*a25);
  a41=(a10*a41);
  a19=(a6*a41);
  a40=(a40-a19);
  a19=(a22*a40);
  a71=(a71+a19);
  a19=(a2*a71);
  a79=(a79-a19);
  a79=(a1*a79);
  a69=(a2*a69);
  a71=(a3*a71);
  a69=(a69+a71);
  a40=(a22*a40);
  a69=(a69+a40);
  a69=(a0*a69);
  a79=(a79+a69);
  if (res[0]!=0) res[0][6]=a79;
  a30=(a29*a30);
  a30=(a13+a30);
  a69=(a9*a30);
  a40=(a7*a69);
  a71=(a10*a30);
  a14=(a29*a14);
  a19=(a12*a14);
  a15=(a29*a15);
  a67=(a11*a15);
  a19=(a19-a67);
  a67=(a39*a19);
  a71=(a71+a67);
  a67=(a6*a71);
  a40=(a40-a67);
  a67=(a5*a40);
  a12=(a12*a15);
  a11=(a11*a14);
  a12=(a12+a11);
  a11=(a33*a12);
  a14=(a4*a11);
  a67=(a67+a14);
  a14=(a3*a67);
  a40=(a4*a40);
  a15=(a5*a11);
  a40=(a40-a15);
  a15=(a10*a12);
  a73=(a6*a15);
  a12=(a9*a12);
  a47=(a7*a12);
  a73=(a73-a47);
  a47=(a22*a73);
  a40=(a40+a47);
  a47=(a2*a40);
  a14=(a14-a47);
  a1=(a1*a14);
  a2=(a2*a67);
  a3=(a3*a40);
  a2=(a2+a3);
  a22=(a22*a73);
  a2=(a2+a22);
  a0=(a0*a2);
  a1=(a1+a0);
  if (res[0]!=0) res[0][7]=a1;
  if (res[0]!=0) res[0][8]=a82;
  a8=(a6*a8);
  a46=(a7*a46);
  a8=(a8+a46);
  a8=(a8*a6);
  a37=(a6*a37);
  a49=(a7*a49);
  a37=(a37+a49);
  a37=(a37*a7);
  a8=(a8+a37);
  a66=(a7*a66);
  a56=(a6*a56);
  a66=(a66+a56);
  a66=(a66*a6);
  a55=(a7*a55);
  a58=(a6*a58);
  a55=(a55-a58);
  a55=(a55*a7);
  a66=(a66+a55);
  a55=(a74*a66);
  a55=(a8-a55);
  a58=(a43*a9);
  a36=(a36-a58);
  a36=(a36*a6);
  a43=(a43*a10);
  a39=(a39*a52);
  a43=(a43+a39);
  a64=(a64-a43);
  a64=(a64*a7);
  a36=(a36+a64);
  a64=(a42*a36);
  a55=(a55-a64);
  a34=(a6*a34);
  a78=(a7*a78);
  a34=(a34-a78);
  a34=(a34*a7);
  a27=(a6*a27);
  a59=(a7*a59);
  a27=(a27+a59);
  a27=(a27*a6);
  a34=(a34+a27);
  a24=(a7*a24);
  a81=(a6*a81);
  a24=(a24-a81);
  a24=(a24*a7);
  a72=(a7*a72);
  a38=(a6*a38);
  a72=(a72-a38);
  a72=(a72*a6);
  a24=(a24-a72);
  a72=(a74*a24);
  a34=(a34+a72);
  a10=(a50*a10);
  a10=(a10*a7);
  a9=(a50*a9);
  a9=(a9*a6);
  a10=(a10+a9);
  a9=(a42*a10);
  a34=(a34-a9);
  a9=(a34*a74);
  a55=(a55+a9);
  a80=(a80+a52);
  a52=(a42*a80);
  a35=(a6*a35);
  a48=(a7*a48);
  a35=(a35+a48);
  a16=(a7*a16);
  a51=(a6*a51);
  a16=(a16+a51);
  a51=(a74*a16);
  a35=(a35+a51);
  a52=(a52-a35);
  a35=(a52*a42);
  a55=(a55+a35);
  a55=(a13+a55);
  a35=(a4*a36);
  a51=(a5*a66);
  a35=(a35-a51);
  a51=(a5*a24);
  a48=(a4*a10);
  a51=(a51+a48);
  a48=(a51*a74);
  a35=(a35+a48);
  a48=(a5*a16);
  a9=(a4*a80);
  a48=(a48+a9);
  a9=(a48*a42);
  a35=(a35-a9);
  a9=(a21*a35);
  a9=(a55+a9);
  a24=(a4*a24);
  a10=(a5*a10);
  a24=(a24-a10);
  a10=(a24*a74);
  a72=(a4*a66);
  a38=(a5*a36);
  a72=(a72+a38);
  a10=(a10-a72);
  a80=(a5*a80);
  a16=(a4*a16);
  a80=(a80-a16);
  a16=(a80*a42);
  a10=(a10+a16);
  a10=(a29+a10);
  a16=(a76*a10);
  a9=(a9+a16);
  a16=(a34*a5);
  a72=(a52*a4);
  a16=(a16-a72);
  a72=(a51*a5);
  a38=(a48*a4);
  a72=(a72+a38);
  a72=(a32+a72);
  a72=(a21*a72);
  a16=(a16+a72);
  a72=(a24*a5);
  a38=(a80*a4);
  a72=(a72-a38);
  a72=(a76*a72);
  a16=(a16+a72);
  a16=(a16*a21);
  a9=(a9+a16);
  a34=(a34*a4);
  a52=(a52*a5);
  a34=(a34+a52);
  a29=(a29+a34);
  a51=(a51*a4);
  a48=(a48*a5);
  a51=(a51-a48);
  a51=(a21*a51);
  a29=(a29+a51);
  a24=(a24*a4);
  a80=(a80*a5);
  a24=(a24+a80);
  a32=(a32+a24);
  a32=(a76*a32);
  a29=(a29+a32);
  a29=(a29*a76);
  a9=(a9+a29);
  a9=(a13+a9);
  if (res[0]!=0) res[0][9]=a9;
  a35=(a21*a35);
  a35=(a55+a35);
  a10=(a76*a10);
  a35=(a35+a10);
  if (res[0]!=0) res[0][10]=a35;
  a10=(a74*a66);
  a10=(a8-a10);
  a9=(a42*a36);
  a10=(a10-a9);
  a9=(a4*a36);
  a29=(a5*a66);
  a9=(a9-a29);
  a9=(a21*a9);
  a9=(a10+a9);
  a66=(a4*a66);
  a36=(a5*a36);
  a66=(a66+a36);
  a66=(a76*a66);
  a9=(a9-a66);
  if (res[0]!=0) res[0][11]=a9;
  a70=(a7*a70);
  a75=(a6*a75);
  a70=(a70-a75);
  a75=(a74*a70);
  a61=(a6*a61);
  a62=(a7*a62);
  a61=(a61+a62);
  a75=(a75-a61);
  a50=(a50*a33);
  a33=(a42*a50);
  a75=(a75-a33);
  a33=(a5*a70);
  a62=(a4*a50);
  a33=(a33+a62);
  a33=(a21*a33);
  a33=(a75+a33);
  a70=(a4*a70);
  a50=(a5*a50);
  a70=(a70-a50);
  a70=(a76*a70);
  a33=(a33+a70);
  if (res[0]!=0) res[0][12]=a33;
  a18=(a7*a18);
  a68=(a6*a68);
  a18=(a18-a68);
  a54=(a6*a54);
  a26=(a7*a26);
  a54=(a54-a26);
  a26=(a74*a54);
  a26=(a18+a26);
  a68=(a5*a54);
  a68=(a21*a68);
  a68=(a26+a68);
  a54=(a4*a54);
  a54=(a76*a54);
  a68=(a68+a54);
  if (res[0]!=0) res[0][13]=a68;
  a41=(a7*a41);
  a25=(a6*a25);
  a41=(a41+a25);
  a25=(a74*a41);
  a57=(a6*a57);
  a28=(a7*a28);
  a57=(a57+a28);
  a25=(a25-a57);
  a28=(a42*a45);
  a25=(a25-a28);
  a28=(a5*a41);
  a54=(a4*a45);
  a28=(a28+a54);
  a28=(a21*a28);
  a28=(a25+a28);
  a41=(a4*a41);
  a45=(a5*a45);
  a41=(a41-a45);
  a41=(a76*a41);
  a28=(a28+a41);
  if (res[0]!=0) res[0][14]=a28;
  a41=(a4*a19);
  a15=(a7*a15);
  a12=(a6*a12);
  a15=(a15+a12);
  a12=(a5*a15);
  a41=(a41-a12);
  a21=(a21*a41);
  a6=(a6*a69);
  a7=(a7*a71);
  a6=(a6+a7);
  a74=(a74*a15);
  a74=(a6+a74);
  a42=(a42*a19);
  a74=(a74+a42);
  a21=(a21-a74);
  a4=(a4*a15);
  a5=(a5*a19);
  a4=(a4+a5);
  a76=(a76*a4);
  a21=(a21-a76);
  if (res[0]!=0) res[0][15]=a21;
  if (res[0]!=0) res[0][16]=a60;
  if (res[0]!=0) res[0][17]=a35;
  if (res[0]!=0) res[0][18]=a55;
  if (res[0]!=0) res[0][19]=a10;
  if (res[0]!=0) res[0][20]=a75;
  if (res[0]!=0) res[0][21]=a26;
  if (res[0]!=0) res[0][22]=a25;
  a74=(-a74);
  if (res[0]!=0) res[0][23]=a74;
  if (res[0]!=0) res[0][24]=a77;
  if (res[0]!=0) res[0][25]=a9;
  if (res[0]!=0) res[0][26]=a10;
  if (res[0]!=0) res[0][27]=a8;
  a61=(-a61);
  if (res[0]!=0) res[0][28]=a61;
  if (res[0]!=0) res[0][29]=a18;
  a57=(-a57);
  if (res[0]!=0) res[0][30]=a57;
  a6=(-a6);
  if (res[0]!=0) res[0][31]=a6;
  if (res[0]!=0) res[0][32]=a65;
  if (res[0]!=0) res[0][33]=a33;
  if (res[0]!=0) res[0][34]=a75;
  if (res[0]!=0) res[0][35]=a61;
  if (res[0]!=0) res[0][36]=a63;
  if (res[0]!=0) res[0][37]=a53;
  if (res[0]!=0) res[0][38]=a44;
  a11=(-a11);
  if (res[0]!=0) res[0][39]=a11;
  if (res[0]!=0) res[0][40]=a20;
  if (res[0]!=0) res[0][41]=a68;
  if (res[0]!=0) res[0][42]=a26;
  if (res[0]!=0) res[0][43]=a18;
  if (res[0]!=0) res[0][44]=a53;
  if (res[0]!=0) res[0][45]=a17;
  a17=0.;
  if (res[0]!=0) res[0][46]=a17;
  if (res[0]!=0) res[0][47]=a17;
  if (res[0]!=0) res[0][48]=a79;
  if (res[0]!=0) res[0][49]=a28;
  if (res[0]!=0) res[0][50]=a25;
  if (res[0]!=0) res[0][51]=a57;
  if (res[0]!=0) res[0][52]=a44;
  if (res[0]!=0) res[0][53]=a17;
  if (res[0]!=0) res[0][54]=a31;
  if (res[0]!=0) res[0][55]=a30;
  if (res[0]!=0) res[0][56]=a1;
  if (res[0]!=0) res[0][57]=a21;
  if (res[0]!=0) res[0][58]=a74;
  if (res[0]!=0) res[0][59]=a6;
  if (res[0]!=0) res[0][60]=a11;
  if (res[0]!=0) res[0][61]=a17;
  if (res[0]!=0) res[0][62]=a30;
  if (res[0]!=0) res[0][63]=a13;
  return 0;
}

CASADI_SYMBOL_EXPORT int f(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int f_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int f_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int f_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f_release(int mem) {
}

CASADI_SYMBOL_EXPORT void f_incref(void) {
}

CASADI_SYMBOL_EXPORT void f_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int f_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int f_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real f_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int f_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

#ifdef MATLAB_MEX_FILE
void mex_f(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  casadi_int i, j;
  casadi_real w[155];
  casadi_int *iw = 0;
  const casadi_real* arg[1] = {0};
  casadi_real* res[1] = {0};
  if (argc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f\" failed. Too many input arguments (%d, max 1)", argc);
  if (resc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f\" failed. Too many output arguments (%d, max 1)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+72);
  --resc;
  res[0] = w+8;
  i = f(arg, res, iw, w+72, 0);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f\" failed.");
  if (res[0]) resv[0] = casadi_to_mex(casadi_s1, res[0]);
}
#endif

casadi_int main_f(casadi_int argc, char* argv[]) {
  casadi_int *iw = 0;
  casadi_real w[155];
  const casadi_real* arg[1] = {w+0};
  casadi_real* res[1] = {w+8};
  casadi_int j;
  casadi_real* a = w;
  for (j=0; j<8; ++j) scanf("%lg", a++);
  casadi_int flag = f(arg, res, iw, w+72, 0);
  if (flag) return flag;
  const casadi_real* r = w+8;
  for (j=0; j<64; ++j) CASADI_PRINTF("%g ", *r++);
  CASADI_PRINTF("\n");
  return 0;
}


#ifdef MATLAB_MEX_FILE
void mexFunction(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  char buf[2];
  int buf_ok = --argc >= 0 && !mxGetString(*argv++, buf, sizeof(buf));
  if (!buf_ok) {
    /* name error */
  } else if (strcmp(buf, "f")==0) {
    mex_f(resc, resv, argc, argv);
    return;
  }
  mexErrMsgTxt("First input should be a command string. Possible values: 'f'");
}
#endif
int main(int argc, char* argv[]) {
  if (argc<2) {
    /* name error */
  } else if (strcmp(argv[1], "f")==0) {
    return main_f(argc-2, argv+2);
  }
  fprintf(stderr, "First input should be a command string. Possible values: 'f'\nNote: you may use function.generate_input to create a command string.\n");
  return 1;
}
#ifdef __cplusplus
} /* extern "C" */
#endif
