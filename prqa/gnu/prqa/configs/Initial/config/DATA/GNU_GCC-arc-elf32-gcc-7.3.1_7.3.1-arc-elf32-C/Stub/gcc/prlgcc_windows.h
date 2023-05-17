// prlgcc_windows.h
//

#ifndef INCLUDED_PRLGCC_WINDOWS
#define INCLUDED_PRLGCC_WINDOWS

/* Special handling for GNU/GCC variants on windows platforms */

#  ifdef __cdecl
#    undef __cdecl
#  endif
#  define __cdecl 
#  define  _cdecl __cdecl 
#  define   cdecl  __cdecl 

#  ifdef __far
#    undef __far
#  endif
#  define __far 
#  define  _far __far 
#  define   far  __far 

#  ifdef __near
#    undef __near
#  endif
#  define __near 
#  define  _near __near 
#  define   near  __near 

#  ifdef __pascal
#    undef __pascal
#  endif
#  define __pascal 
#  define  _pascal __pascal 
#  define   pascal  __pascal 

#  ifdef __stdcall
#    undef __stdcall
#  endif
#  define __stdcall
#  define  _stdcall  __stdcall
#  define   stdcall  __stdcall

#  ifdef __fastcall
#    undef __fastcall
#  endif
#  define __fastcall
#  define  _fastcall  __fastcall
#  define  _fastcall  __fastcall

#  ifdef __syscall
#    undef __syscall
#  endif
#  define __syscall
#  define  _syscall  __syscall
#  define  _syscall  __syscall

#  ifdef __oldcall
#    undef __oldcall
#  endif
#  define __oldcall
#  define  _oldcall  __oldcall
#  define   oldcall  __oldcall

#  ifdef __thiscall
#    undef __thiscall
#  endif
#  define __thiscall
#  define  _thiscall __thiscall
#  define   thiscall __thiscall

#  ifdef __declspec
#    undef __declspec
#  endif
#  define __declspec(x) __attribute__((x))

#  if __QAC_MINOR__ <= 90

#    define __cdecl _ignore_modification
#    define  _cdecl _ignore_modification
#    define   cdecl _ignore_modification
#    define __far _ignore_modification
#    define  _far _ignore_modification
#    define   far _ignore_modification
#    define __near _ignore_modification
#    define  _near _ignore_modification
#    define   near _ignore_modification
#    define __pascal _ignore_modification
#    define  _pascal _ignore_modification
#    define   pascal _ignore_modification
#    define __stdcall _ignore_modification
#    define  _stdcall _ignore_modification
#    define __fastcall _ignore_modification
#    define  _fastcall _ignore_modification
#    define __syscall _ignore_modification
#    define  _syscall _ignore_modification
#    define __oldcall _ignore_modification
#    define  _oldcall _ignore_modification
#    define __thiscall _ignore_modification
#    define  _thiscall _ignore_modification
#    define __declspec _ignore_modification

#  endif


#endif
