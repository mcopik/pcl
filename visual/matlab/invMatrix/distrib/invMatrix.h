//
// MATLAB Compiler: 4.14 (R2010b)
// Date: Fri Nov 16 14:39:13 2012
// Arguments: "-B" "macro_default" "-W" "cpplib:invMatrix" "-T" "link:lib" "-d"
// "C:\iterative_closest_point\matlab\invMatrix\src" "-w"
// "enable:specified_file_mismatch" "-w" "enable:repeated_file" "-w"
// "enable:switch_ignored" "-w" "enable:missing_lib_sentinel" "-w"
// "enable:demo_license" "-v" "C:\adam\Amberg.m" 
//

#ifndef __invMatrix_h
#define __invMatrix_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

#if defined(__SUNPRO_CC)
/* Solaris shared libraries use __global, rather than mapfiles
 * to define the API exported from a shared library. __global is
 * only necessary when building the library -- files including
 * this header file to use the library do not need the __global
 * declaration; hence the EXPORTING_<library> logic.
 */

#ifdef EXPORTING_invMatrix
#define PUBLIC_invMatrix_C_API __global
#else
#define PUBLIC_invMatrix_C_API /* No import statement needed. */
#endif

#define LIB_invMatrix_C_API PUBLIC_invMatrix_C_API

#elif defined(_HPUX_SOURCE)

#ifdef EXPORTING_invMatrix
#define PUBLIC_invMatrix_C_API __declspec(dllexport)
#else
#define PUBLIC_invMatrix_C_API __declspec(dllimport)
#endif

#define LIB_invMatrix_C_API PUBLIC_invMatrix_C_API


#else

#define LIB_invMatrix_C_API

#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_invMatrix_C_API 
#define LIB_invMatrix_C_API /* No special import/export declaration */
#endif

extern LIB_invMatrix_C_API 
bool MW_CALL_CONV invMatrixInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_invMatrix_C_API 
bool MW_CALL_CONV invMatrixInitialize(void);

extern LIB_invMatrix_C_API 
void MW_CALL_CONV invMatrixTerminate(void);



extern LIB_invMatrix_C_API 
void MW_CALL_CONV invMatrixPrintStackTrace(void);

extern LIB_invMatrix_C_API 
bool MW_CALL_CONV mlxAmberg(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

extern LIB_invMatrix_C_API 
long MW_CALL_CONV invMatrixGetMcrID();


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__BORLANDC__)

#ifdef EXPORTING_invMatrix
#define PUBLIC_invMatrix_CPP_API __declspec(dllexport)
#else
#define PUBLIC_invMatrix_CPP_API __declspec(dllimport)
#endif

#define LIB_invMatrix_CPP_API PUBLIC_invMatrix_CPP_API

#else

#if !defined(LIB_invMatrix_CPP_API)
#if defined(LIB_invMatrix_C_API)
#define LIB_invMatrix_CPP_API LIB_invMatrix_C_API
#else
#define LIB_invMatrix_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_invMatrix_CPP_API void MW_CALL_CONV Amberg(int nargout, mwArray& X, const mwArray& A, const mwArray& B);

#endif
#endif
