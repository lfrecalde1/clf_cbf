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

#ifndef casadi_real
#define casadi_real float
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int d_dot_f(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int d_dot_f_alloc_mem(void);
int d_dot_f_init_mem(int mem);
void d_dot_f_free_mem(int mem);
int d_dot_f_checkout(void);
void d_dot_f_release(int mem);
void d_dot_f_incref(void);
void d_dot_f_decref(void);
casadi_int d_dot_f_n_in(void);
casadi_int d_dot_f_n_out(void);
casadi_real d_dot_f_default_in(casadi_int i);
const char* d_dot_f_name_in(casadi_int i);
const char* d_dot_f_name_out(casadi_int i);
const casadi_int* d_dot_f_sparsity_in(casadi_int i);
const casadi_int* d_dot_f_sparsity_out(casadi_int i);
int d_dot_f_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int d_dot_f_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define d_dot_f_SZ_ARG 1
#define d_dot_f_SZ_RES 1
#define d_dot_f_SZ_IW 0
#define d_dot_f_SZ_W 1
#ifdef __cplusplus
} /* extern "C" */
#endif
