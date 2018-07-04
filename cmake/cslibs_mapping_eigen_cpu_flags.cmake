# Determine the host CPU feature set and determine the best set of compiler
# flags to enable all supported SIMD relevant features. Alternatively, the
# target CPU can be explicitly selected (for generating more generic binaries
# or for targeting a different system).
# Compilers provide e.g. the -march=native flag to achieve a similar result.
# This fails to address the need for building for a different microarchitecture
# than the current host.
# The script tries to deduce all settings from the model and family numbers of
# the CPU instead of reading the CPUID flags from e.g. /proc/cpuinfo. This makes
# the detection more independent from the CPUID code in the kernel (e.g. avx2 is
# not listed on older kernels).
#
# Usage:
# OptimizeForArchitecture()
# If either of Vc_SSE_INTRINSICS_BROKEN, Vc_AVX_INTRINSICS_BROKEN,
# Vc_AVX2_INTRINSICS_BROKEN is defined and set, the OptimizeForArchitecture
# macro will consequently disable the relevant features via compiler flags.

#=============================================================================
# Copyright 2010-2016 Matthias Kretz <kretz@kde.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the names of contributing organizations nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS ``AS IS''
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#=============================================================================

### Check for eigen CPU flags
set(_vendor_id)
set(_cpu_family)
set(_cpu_model)
file(READ "/proc/cpuinfo" _cpuinfo)
string(REGEX REPLACE ".*vendor_id[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _vendor_id "${_cpuinfo}")
string(REGEX REPLACE ".*cpu family[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_family "${_cpuinfo}")
string(REGEX REPLACE ".*model[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_model "${_cpuinfo}")
string(REGEX REPLACE ".*flags[ \t]*:[ \t]+([^\n]+).*" "\\1" _cpu_flags "${_cpuinfo}")

if(${_vendor_id} STREQUAL "GenuineIntel")
    if(${_cpu_family} EQUAL 6)
        # taken from the Intel ORM
        # http://www.intel.com/content/www/us/en/processors/architectures-software-developer-manuals.html
        # CPUID Signature Values of Of Recent Intel Microarchitectures
        # 4E 5E       | Skylake microarchitecture
        # 3D 47 56    | Broadwell microarchitecture
        # 3C 45 46 3F | Haswell microarchitecture
        # 3A 3E       | Ivy Bridge microarchitecture
        # 2A 2D       | Sandy Bridge microarchitecture
        # 25 2C 2F    | Intel microarchitecture Westmere
        # 1A 1E 1F 2E | Intel microarchitecture Nehalem
        # 17 1D       | Enhanced Intel Core microarchitecture
        # 0F          | Intel Core microarchitecture
        #
        # Intel SDM Vol. 3C 35-1 / December 2016:
        # 57          | Xeon Phi 3200, 5200, 7200  [Knights Landing]
        # 85          | Future Xeon Phi
        # 8E 9E       | 7th gen. Core              [Kaby Lake]
        # 55          | Future Xeon                [Skylake w/ AVX512]
        # 4E 5E       | 6th gen. Core / E3 v5      [Skylake w/o AVX512]
        # 56          | Xeon D-1500                [Broadwell]
        # 4F          | Xeon E5 v4, E7 v4, i7-69xx [Broadwell]
        # 47          | 5th gen. Core / Xeon E3 v4 [Broadwell]
        # 3D          | M-5xxx / 5th gen.          [Broadwell]
        # 3F          | Xeon E5 v3, E7 v3, i7-59xx [Haswell-E]
        # 3C 45 46    | 4th gen. Core, Xeon E3 v3  [Haswell]
        # 3E          | Xeon E5 v2, E7 v2, i7-49xx [Ivy Bridge-E]
        # 3A          | 3rd gen. Core, Xeon E3 v2  [Ivy Bridge]
        # 2D          | Xeon E5, i7-39xx           [Sandy Bridge]
        # 2F          | Xeon E7
        # 2A          | Xeon E3, 2nd gen. Core     [Sandy Bridge]
        # 2E          | Xeon 7500, 6500 series
        # 25 2C       | Xeon 3600, 5600 series, Core i7, i5 and i3
        #
        # Values from the Intel SDE:
        # 5C | Goldmont
        # 5A | Silvermont
        # 57 | Knights Landing
        # 66 | Cannonlake
        # 55 | Skylake Server
        # 4E | Skylake Client
        # 3C | Broadwell (likely a bug in the SDE)
        # 3C | Haswell        

        if(${_cpu_model} EQUAL 60)
            message("[cslibs_mapping]: Found Xeon E3 v3, disabling eigen vectorization.")
            string(REPLACE "-march=native" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
            string(REPLACE "-mavx" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
            add_definitions(-DEIGEN_DONT_ALIGN -fvisibility-inlines-hidden)
#            add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DONT_ALIGN)
            add_definitions(-msse -mfpmath=sse)
        endif()
    endif()
endif()
