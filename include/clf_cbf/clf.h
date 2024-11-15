/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real float
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int lyapunov_f(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int lyapunov_f_alloc_mem(void);
int lyapunov_f_init_mem(int mem);
void lyapunov_f_free_mem(int mem);
int lyapunov_f_checkout(void);
void lyapunov_f_release(int mem);
void lyapunov_f_incref(void);
void lyapunov_f_decref(void);
casadi_int lyapunov_f_n_in(void);
casadi_int lyapunov_f_n_out(void);
casadi_real lyapunov_f_default_in(casadi_int i);
const char* lyapunov_f_name_in(casadi_int i);
const char* lyapunov_f_name_out(casadi_int i);
const casadi_int* lyapunov_f_sparsity_in(casadi_int i);
const casadi_int* lyapunov_f_sparsity_out(casadi_int i);
int lyapunov_f_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#ifdef __cplusplus
} /* extern "C" */
#endif
