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
  #define CASADI_PREFIX(ID) Pcom_ ## ID
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

static const casadi_int casadi_s0[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};

/* f:(i0[5])->(o0[3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a3, a4, a5, a6, a7, a8, a9;
  a0=5.0000000000000000e-01;
  a1=arg[0]? arg[0][0] : 0;
  a2=sin(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=cos(a3);
  a5=arg[0]? arg[0][2] : 0;
  a6=cos(a5);
  a7=arg[0]? arg[0][3] : 0;
  a8=cos(a7);
  a9=-4.0000000000000002e-01;
  a10=arg[0]? arg[0][4] : 0;
  a11=sin(a10);
  a12=(a9*a11);
  a13=(a8*a12);
  a7=sin(a7);
  a10=cos(a10);
  a14=(a9*a10);
  a15=-8.0000000000000004e-01;
  a14=(a14+a15);
  a14=(a9+a14);
  a15=(a7*a14);
  a13=(a13+a15);
  a15=(a6*a13);
  a5=sin(a5);
  a16=-7.5000000000000000e-01;
  a14=(a8*a14);
  a12=(a7*a12);
  a14=(a14-a12);
  a16=(a16+a14);
  a14=(a5*a16);
  a15=(a15+a14);
  a14=(a4*a15);
  a3=sin(a3);
  a16=(a6*a16);
  a13=(a5*a13);
  a16=(a16-a13);
  a13=-3.6000000000000001e+00;
  a16=(a16+a13);
  a16=(a9+a16);
  a13=(a3*a16);
  a14=(a14+a13);
  a13=(a2*a14);
  a1=cos(a1);
  a16=(a4*a16);
  a15=(a3*a15);
  a16=(a16-a15);
  a15=-4.4000000000000004e+00;
  a16=(a16+a15);
  a9=(a9+a16);
  a16=(a1*a9);
  a13=(a13-a16);
  a16=2.;
  a15=4.0000000000000002e-01;
  a12=(a15*a11);
  a12=(a16*a12);
  a17=(a12*a10);
  a18=(a15*a10);
  a18=(a16*a18);
  a18=(a15+a18);
  a19=(a18*a11);
  a17=(a17-a19);
  a19=(a17*a8);
  a12=(a12*a11);
  a18=(a18*a10);
  a12=(a12+a18);
  a12=(a15+a12);
  a18=(a12*a7);
  a19=(a19-a18);
  a18=(a15*a5);
  a20=5.;
  a21=(a16*a10);
  a22=(a21*a10);
  a23=(a16*a11);
  a24=(a23*a11);
  a22=(a22+a24);
  a22=(a16+a22);
  a24=(a8*a22);
  a25=(a16*a11);
  a26=(a25*a10);
  a27=(a16*a10);
  a28=(a27*a11);
  a26=(a26-a28);
  a28=(a7*a26);
  a24=(a24-a28);
  a28=(a24*a8);
  a21=(a21*a11);
  a23=(a23*a10);
  a21=(a21-a23);
  a23=(a8*a21);
  a25=(a25*a11);
  a27=(a27*a10);
  a25=(a25+a27);
  a25=(a16+a25);
  a27=(a7*a25);
  a23=(a23-a27);
  a27=(a23*a7);
  a28=(a28-a27);
  a28=(a20+a28);
  a27=(a18*a28);
  a19=(a19+a27);
  a27=(a15*a6);
  a22=(a7*a22);
  a26=(a8*a26);
  a22=(a22+a26);
  a26=(a22*a8);
  a21=(a7*a21);
  a25=(a8*a25);
  a21=(a21+a25);
  a25=(a21*a7);
  a26=(a26-a25);
  a25=(a27*a26);
  a19=(a19+a25);
  a25=(a19*a6);
  a10=7.5000000000000000e-01;
  a17=(a17*a7);
  a12=(a12*a8);
  a17=(a17+a12);
  a10=(a10+a17);
  a24=(a24*a7);
  a23=(a23*a8);
  a24=(a24+a23);
  a18=(a18*a24);
  a10=(a10+a18);
  a22=(a22*a7);
  a21=(a21*a8);
  a22=(a22+a21);
  a20=(a20+a22);
  a27=(a27*a20);
  a10=(a10+a27);
  a27=(a10*a5);
  a25=(a25-a27);
  a27=(a15*a3);
  a22=(a6*a28);
  a21=(a5*a26);
  a22=(a22-a21);
  a21=(a22*a6);
  a8=(a6*a24);
  a7=(a5*a20);
  a8=(a8-a7);
  a7=(a8*a5);
  a21=(a21-a7);
  a21=(a16+a21);
  a7=(a27*a21);
  a25=(a25+a7);
  a7=(a15*a4);
  a28=(a5*a28);
  a26=(a6*a26);
  a28=(a28+a26);
  a26=(a28*a6);
  a24=(a5*a24);
  a20=(a6*a20);
  a24=(a24+a20);
  a20=(a24*a5);
  a26=(a26-a20);
  a20=(a7*a26);
  a25=(a25+a20);
  a20=(a25*a4);
  a19=(a19*a5);
  a10=(a10*a6);
  a19=(a19+a10);
  a19=(a15+a19);
  a22=(a22*a5);
  a8=(a8*a6);
  a22=(a22+a8);
  a27=(a27*a22);
  a19=(a19+a27);
  a28=(a28*a5);
  a24=(a24*a6);
  a28=(a28+a24);
  a28=(a16+a28);
  a7=(a7*a28);
  a19=(a19+a7);
  a7=(a19*a3);
  a20=(a20-a7);
  a7=(a20*a2);
  a25=(a25*a3);
  a19=(a19*a4);
  a25=(a25+a19);
  a15=(a15+a25);
  a25=(a15*a1);
  a7=(a7+a25);
  a13=(a13+a7);
  a13=(a0*a13);
  a7=(a4*a21);
  a25=(a3*a26);
  a7=(a7-a25);
  a25=(a7*a4);
  a19=(a4*a22);
  a24=(a3*a28);
  a19=(a19-a24);
  a24=(a19*a3);
  a25=(a25-a24);
  a25=(a16+a25);
  a25=(a2*a25);
  a21=(a3*a21);
  a26=(a4*a26);
  a21=(a21+a26);
  a26=(a21*a4);
  a22=(a3*a22);
  a28=(a4*a28);
  a22=(a22+a28);
  a28=(a22*a3);
  a26=(a26-a28);
  a26=(a1*a26);
  a25=(a25+a26);
  a25=(a25*a2);
  a7=(a7*a3);
  a19=(a19*a4);
  a7=(a7+a19);
  a7=(a2*a7);
  a21=(a21*a3);
  a22=(a22*a4);
  a21=(a21+a22);
  a16=(a16+a21);
  a16=(a1*a16);
  a7=(a7+a16);
  a7=(a7*a1);
  a25=(a25+a7);
  a13=(a13/a25);
  if (res[0]!=0) res[0][0]=a13;
  a13=0.;
  if (res[0]!=0) res[0][1]=a13;
  a20=(a20*a1);
  a15=(a15*a2);
  a20=(a20-a15);
  a1=(a1*a14);
  a2=(a2*a9);
  a1=(a1+a2);
  a20=(a20+a1);
  a0=(a0*a20);
  a0=(a0/a25);
  if (res[0]!=0) res[0][2]=a0;
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
  casadi_real w[37];
  casadi_int *iw = 0;
  const casadi_real* arg[1] = {0};
  casadi_real* res[1] = {0};
  if (argc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f\" failed. Too many input arguments (%d, max 1)", argc);
  if (resc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f\" failed. Too many output arguments (%d, max 1)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+8);
  --resc;
  res[0] = w+5;
  i = f(arg, res, iw, w+8, 0);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"f\" failed.");
  if (res[0]) resv[0] = casadi_to_mex(casadi_s1, res[0]);
}
#endif

casadi_int main_f(casadi_int argc, char* argv[]) {
  casadi_int *iw = 0;
  casadi_real w[37];
  const casadi_real* arg[1] = {w+0};
  casadi_real* res[1] = {w+5};
  casadi_int j;
  casadi_real* a = w;
  for (j=0; j<5; ++j) scanf("%lg", a++);
  casadi_int flag = f(arg, res, iw, w+8, 0);
  if (flag) return flag;
  const casadi_real* r = w+5;
  for (j=0; j<3; ++j) CASADI_PRINTF("%g ", *r++);
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
