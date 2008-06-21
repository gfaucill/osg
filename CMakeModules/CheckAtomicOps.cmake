# Check for availability of atomic operations 
# This module defines
# OPENTHREADS_HAVE_ATOMIC_OPS

INCLUDE(CheckCXXSourceRuns)

# Do step by step checking, 
CHECK_CXX_SOURCE_RUNS("
#include <cstdlib>

int main()
{
#ifdef __i386__
   // Bad, gcc behaves dependent on the compilers -march=... flags.
   // Since the osg::Referenced stuff is code distributed in headers, it is
   // unclear if we will have this feature available at compile time of the
   // headers. So just do not use this feature for 32 bit code.
   // May be we can implement around that limitation at some time..
   return EXIT_FAILURE;
#else
   unsigned value = 0;
   void* ptr = &value;
   __sync_add_and_fetch(&value, 1);
   __sync_synchronize();
   __sync_sub_and_fetch(&value, 1);
   if (!__sync_bool_compare_and_swap(&value, 0, 1))
      return EXIT_FAILURE;
   
   if (!__sync_bool_compare_and_swap(&ptr, ptr, ptr))
      return EXIT_FAILURE;

   return EXIT_SUCCESS;
#endif
}
" _OPENTHREADS_ATOMIC_USE_GCC_BUILTINS)

CHECK_CXX_SOURCE_RUNS("
#include <cstdlib>

int main(int, const char**)
{
   unsigned value = 0;
   void* ptr = &value;
   __add_and_fetch(&value, 1);
   __synchronize(value);
   __sub_and_fetch(&value, 1);
   if (!__sync_compare_and_swap(&value, 0, 1))
      return EXIT_FAILURE;
   
   if (!__sync_compare_and_swap(&ptr, ptr, ptr))
      return EXIT_FAILURE;

   return EXIT_SUCCESS;
}
" _OPENTHREADS_ATOMIC_USE_MIPOSPRO_BUILTINS)

CHECK_CXX_SOURCE_RUNS("
#include <atomic.h>
#include <cstdlib>

int main(int, const char**)
{
   uint_t value = 0;
   void* ptr = &value;
   atomic_inc_uint_nv(&value);
   membar_consumer();
   atomic_dec_uint_nv(&value);
   if (0 != atomic_cas_uint(&value, 0, 1))
      return EXIT_FAILURE;
   
   if (ptr != atomic_cas_ptr(&ptr, ptr, ptr))
      return EXIT_FAILURE;

   return EXIT_SUCCESS;
}
" _OPENTHREADS_ATOMIC_USE_SUN)

CHECK_CXX_SOURCE_RUNS("
#include <windows.h>
#include <cstdlib>

int main(int, const char**)
{
   __declspec(align(32)) volatile long value = 0;
   __declspec(align(32)) volatile void* ptr = &value;

   InterlockedIncrement(&value);
   InterlockedDecrement(&value);

   if (0 != InterlockedCompareExchange(&value, 1, 0))
      return EXIT_FAILURE;

   if (ptr != InterlockedCompareExchangePointer(&ptr, ptr, ptr))
      return EXIT_FAILURE;

   return EXIT_SUCCESS;
}
" _OPENTHREADS_ATOMIC_USE_WIN32_INTERLOCKED)

IF(NOT _OPENTHREADS_ATOMIC_USE_GCC_BUILTINS AND NOT _OPENTHREADS_ATOMIC_USE_MIPOSPRO_BUILTINS AND NOT _OPENTHREADS_ATOMIC_USE_SUN AND NOT _OPENTHREADS_ATOMIC_USE_WIN32_INTERLOCKED)
  SET(_OPENTHREADS_ATOMIC_USE_MUTEX)
ENDIF(NOT _OPENTHREADS_ATOMIC_USE_GCC_BUILTINS AND NOT _OPENTHREADS_ATOMIC_USE_MIPOSPRO_BUILTINS AND NOT _OPENTHREADS_ATOMIC_USE_SUN AND NOT _OPENTHREADS_ATOMIC_USE_WIN32_INTERLOCKED)