#ifndef GENERIC_FI_H
#define GENERIC_FI_H

/* Compilers may use another function within the assert macro to exit the
 * program. QAC thus does not see an "exit" function and assumes the
 * function returns, which it does not.
 * The below pragma tells QAC the compiler supplied function does not
 * return.
 */

/* examples - GCC, ICC, BCC 3.1, BCC 5.5, CYGWIN gcc, Watcom 1.6 */
#pragma PRQA_NO_RETURN __assert_fail
#pragma PRQA_NO_RETURN __assert_perror_fail
#pragma PRQA_NO_RETURN __assert

/* library functions that does not return */
#pragma PRQA_NO_RETURN pthread_exit
#pragma PRQA_NO_RETURN __pthread_unwind_next
#pragma PRQA_NO_RETURN abort
#pragma PRQA_NO_RETURN _exit
#pragma PRQA_NO_RETURN exit
#pragma PRQA_NO_RETURN quick_exit
#pragma PRQA_NO_RETURN _Exit
#pragma PRQA_NO_RETURN longjmp
#pragma PRQA_NO_RETURN _longjmp
#pragma PRQA_NO_RETURN siglongjmp

/* xalloc usage is common thus placing this pragma outside  */
#pragma PRQA_NO_RETURN xalloc_die
#pragma PRQA_NO_RETURN yy_fatal_error

/* Depending on the code that the compiler maker uses within the assert macro,
 * QAC may issue warnings. It is also possible that when the NDEBUG macro is used
 * to suppress the assertion code, QAC may issue a "no side-effects" warning.
 * To eliminate these, we turn off all warnings for the assert macro.
 */
#pragma PRQA_MACRO_MESSAGES_OFF "assert"
#pragma PRQA_MACRO_MESSAGES_OFF "errno"


/* Any other items that may be required in a "force-include" file should
 * be placed below.
 */

/* to avoid problems with GCC using builtin functions for va_start
 * and QAC issuing unset variable warnings.
 */
typedef void *           __builtin_va_list;
#define                  __builtin_va_start(ap, parmN)	((void)((ap)= &(parmN)))
#define                  __builtin_va_arg(ap, type)	(*(type*)(ap))
#define                  __builtin_va_end(ap)		((void)((ap)=0))
#define                  __builtin_va_copy(d,s)
#pragma PRQA_MACRO_MESSAGES_OFF "va_end"
#pragma PRQA_MACRO_MESSAGES_OFF "__va_copy"

#define                  __builtin_offsetof(type, member) ((size_t) &((type *)0)->member)

/* GCC builtin function equivalents that require prototypes
 * for QAC.
 */

#define DUMMY_ELLPISIS_PARAMETER float,

extern long              __builtin_expect (long, long);
extern void *            __builtin_memcpy(void *, const void *, __SIZE_TYPE__);
extern void *            __builtin_mempcpy(void *, const void *, __SIZE_TYPE__);
extern void *            __builtin_memset(void *, int, __SIZE_TYPE__);
extern int               __builtin_memcmp(const void *, const void *, __SIZE_TYPE__);
extern char *            __builtin_strcat(char *, const char *);
extern char *            __builtin_strncat(char *, const char *, __SIZE_TYPE__);
extern int               __builtin_strcmp(const char *, const char *);
extern char *            __builtin_strchr(const char *, int);
extern char *            __builtin_strcpy(char *, const char *);
extern char *            __builtin_strncpy(char *, const char *, __SIZE_TYPE__);
extern __SIZE_TYPE__     __builtin_strspn(const char *, const char *);
extern __SIZE_TYPE__     __builtin_strcspn(const char *, const char *);
extern char *            __builtin_strpbrk(const char *, const char *);
extern __SIZE_TYPE__     __builtin_strlen(const char *);
extern int               __builtin_clz(unsigned int);
extern int               __builtin_clzl(unsigned long);
extern int               __builtin_clzll(unsigned long long);
extern int               __builtin_ctz(unsigned int);
extern int               __builtin_ctzl(unsigned long);
extern int               __builtin_ctzll(unsigned long long);
extern int               __builtin_ffs(int);
extern int               __builtin_ffsl(long);
extern int               __builtin_ffsll(long long);
extern int               __builtin_popcount(unsigned int);
extern int               __builtin_popcountl(unsigned long);
extern int               __builtin_popcountll(unsigned long long);
extern void *            __builtin_return_address(const unsigned int);
extern void *            __builtin_extract_return_addr(void *);
extern void *            __builtin_frame_address(const unsigned int);
extern void              __builtin_trap(void);
extern void *            __builtin_alloca(__SIZE_TYPE__);
extern void              __builtin_prefetch (const void *, ...);
extern long              __builtin_alpha_extbl(long, long);
extern long              __builtin_alpha_extwl(long, long);
extern long              __builtin_alpha_insbl(long, long);
extern long              __builtin_alpha_inswl(long, long);
extern long              __builtin_alpha_insql(long, long);
extern long              __builtin_alpha_inslh(long, long);
extern long              __builtin_alpha_cmpbge(long, long);
extern long              __builtin_labs(long);
extern int               __builtin_isgreater(DUMMY_ELLPISIS_PARAMETER ...);
extern int               __builtin_isgreaterequal(DUMMY_ELLPISIS_PARAMETER ...);
extern int               __builtin_isless(DUMMY_ELLPISIS_PARAMETER ...);
extern int               __builtin_islessequal(DUMMY_ELLPISIS_PARAMETER ...);
extern int               __builtin_islessgreater(DUMMY_ELLPISIS_PARAMETER ...);
extern int               __builtin_isunordered(DUMMY_ELLPISIS_PARAMETER ...);
extern __SIZE_TYPE__     __builtin_object_size(const void *, int);
extern void *            __builtin___memcpy_chk(void *, const void *, __SIZE_TYPE__, __SIZE_TYPE__);
extern void *            __builtin___memmove_chk(void *, const void *, __SIZE_TYPE__, __SIZE_TYPE__);
extern void *            __builtin___mempcpy_chk(void *, const void *, __SIZE_TYPE__, __SIZE_TYPE__);
extern void *            __builtin___memset_chk(void *, int, __SIZE_TYPE__, __SIZE_TYPE__);
extern int               __builtin___sprintf_chk(char *, int, __SIZE_TYPE__, const char *, ...);
extern int               __builtin___snprintf_chk(char *, __SIZE_TYPE__, int , __SIZE_TYPE__, const char *, ...);
extern char *            __builtin___stpcpy_chk(char *, const char *, __SIZE_TYPE__);
extern char *            __builtin___strcat_chk(char *, const char *, __SIZE_TYPE__);
extern char *            __builtin___strcpy_chk(char *, const char *, __SIZE_TYPE__);
extern char *            __builtin___strncat_chk(char *, const char *, __SIZE_TYPE__, __SIZE_TYPE__);
extern char *            __builtin___strncpy_chk(char *, const char *, __SIZE_TYPE__, __SIZE_TYPE__);

/* extension keywords */
#define __FUNCTION__ _PRL_STRINGIFY(__FUNCTION__)
#define __PRETTY_FUNCTION__ _PRL_STRINGIFY(__PRETTY_FUNCTION__)
#define __builtin_constant_p(exp) 1
#define __builtin_types_compatible_p(a,b) 1
#define __func__ _PRL_STRINGIFY(__func__)
#define __g77_ulongint unsigned long
#define __must_be_array(x) 0
#define __thread
#define __typeof typeof
#define _thread 
#define typeof(x) __typeof__(x)
#define _Complex 
#define _Static_assert _ignore_semi
#define __alignof __alignof__
//#define __asm asm
//#define __asm__ asm
#define __attribute __attribute__
#define __attribute__ _ignore_paren
#define __complex__ 
#define __const const
#define __const__ const
#define __extension__ 
#define __g77_integer int
#define __g77_longint long
#define __g77_uinteger unsigned
#define __imag__ 
#define __inline inline
#define __inline__ inline
#define __null 0
#define __real__ 
#define __signed signed
#define __signed__ signed
#define __volatile volatile
#define __volatile__ volatile
#define __int128 long long

/* C99 */
#define _Pragma _ignore_paren

/* statement expressions */
#define __WAIT_INT(status) (status)

#if defined (__CYGWIN__) || defined (__MINGW32__) || defined (__WIN32)
/* gcc on windows requires additional workarounds */
#include "prlgcc_windows.h"
#endif

/* Support for restrict added to QAC 9.1.0 onwards */
#if __QAC_MAJOR__ < 91
#  define restrict
#else
#  define restrict restrict
#endif

#define __restrict restrict
#define __restrict__ restrict

/* C11 */
#define _Noreturn 

#else
#error "Multiple include"
#endif  /* ifndef GENERIC_FI_H */
