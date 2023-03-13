/*
BSD 2-Clause License

Copyright (c) 2020, Andrea Zoppi
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "YM7128B_emu.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "endian.h"

#ifdef __WINDOWS__
#include <io.h>
#endif


static char const* USAGE = ("\
YM7128B_pipe (c) 2020, Andrea Zoppi. All rights reserved.\n\
\n\
This program emulates a YM7128B Surround Processor, made by Yamaha.\n\
It reads a sample stream from standard input, processes data, and writes\n\
to the standard output.\n\
The sample format is as specified by the --format option.\n\
The output is always stereo, with the same sample format as per the input.\n\
In case of fixed and float engines, the output rate is doubled\n\
(2x oversampling).\n\
\n\
\n\
USAGE:\n\
  pipe [OPTION]...\n\
\n\
\n\
OPTION (evaluated as per command line argument order):\n\
\n\
--dry DECIBEL\n\
    Dry (unprocessed) output volume multiplier [dB]; default: 0.\n\
    Values outside range (-128; +128) do mute.\n\
\n\
-f, --format FORMAT\n\
    Sample format name; default: U8.\n\
    See FORMAT table.\n\
\n\
-h, --help\n\
    Prints this help message and quits.\n\
\n\
-e, --engine ENGINE\n\
    Chip engine; default: fixed.\n\
    See ENGINE table.\n\
\n\
-r, --rate RATE\n\
    Sample rate [Hz]; default: 23550.\n\
\n\
--preset PRESET\n\
    Register preset; default: off. See PRESET table.\n\
\n\
--reg-<REGISTER> [0x]HEX\n\
    Value of <REGISTER> register; hexadecimal string.\n\
\n\
--regdump HEX...\n\
    Hexadecimal string of all the registers to overwrite, starting from\n\
    address zero.\n\
\n\
--wet DECIBEL\n\
    Wet (processed) output volume multiplier [dB]; default: 0.\n\
    Values outside range (-128; +128) do mute.\n\
\n\
\n\
ENGINE:\n\
\n\
- fixed:  Fixed-point (default).\n\
- float:  Floating-point.\n\
- ideal:  Ideal float model.\n\
- short:  Ideal short model.\n\
\n\
\n\
FORMAT:\n\
\n\
| Name       | Bits | Sign | Endian |\n\
|------------|------|------|--------|\n\
| dummy      |    0 | no   | same   |\n\
| U8         |    8 | no   | same   |\n\
| S8         |    8 | yes  | same   |\n\
| U16_LE     |   16 | no   | little |\n\
| U16_BE     |   16 | no   | big    |\n\
| S16_LE     |   16 | yes  | little |\n\
| S16_BE     |   16 | yes  | big    |\n\
| U32_LE     |   32 | no   | little |\n\
| U32_BE     |   32 | no   | big    |\n\
| S32_LE     |   32 | yes  | little |\n\
| S32_BE     |   32 | yes  | big    |\n\
| FLOAT_LE   |   32 | yes  | little |\n\
| FLOAT_BE   |   32 | yes  | big    |\n\
| FLOAT64_LE |   64 | yes  | little |\n\
| FLOAT64_BE |   64 | yes  | big    |\n\
\n\
\n\
PRESET:\n\
\n\
- off\n\
- direct\n\
- gold/recital_hall\n\
- gold/concert_hall\n\
- gold/chapel\n\
- gold/jazz_club\n\
- gold/movie_theater\n\
- gold/stadium\n\
- gold/cavern\n\
- gold/deep_space\n\
- dune/arrakis\n\
- dune/baghdad\n\
- dune/morning\n\
- dune/sequence\n\
- dune/sietch\n\
- dune/warsong\n\
- dune/water\n\
- dune/wormintro\n\
- dune/wormsuit\n\
\n\
\n\
LICENSE:\n\
\n\
BSD 2-Clause License\n\
\n\
Copyright (c) 2020, Andrea Zoppi\n\
All rights reserved.\n\
\n\
Redistribution and use in source and binary forms, with or without\n\
modification, are permitted provided that the following conditions are met:\n\
\n\
1. Redistributions of source code must retain the above copyright notice, this\n\
   list of conditions and the following disclaimer.\n\
\n\
2. Redistributions in binary form must reproduce the above copyright notice,\n\
   this list of conditions and the following disclaimer in the documentation\n\
   and/or other materials provided with the distribution.\n\
\n\
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\"\n\
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE\n\
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE\n\
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE\n\
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL\n\
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR\n\
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER\n\
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,\n\
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE\n\
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.\n\
");


#if __BYTE_ORDER == __LITTLE_ENDIAN
#define READ_LE( dst_, size_)  (fread ((dst_), (size_), 1, stdin ) == 1)
#define WRITE_LE(src_, size_)  (fwrite((src_), (size_), 1, stdout) == 1)
#define READ_BE( dst_, size_)  (fread ((dst_), 1, (size_), stdin ) == (size_))
#define WRITE_BE(src_, size_)  (fwrite((src_), 1, (size_), stdout) == (size_))
#elif __BYTE_ORDER == __BIG_ENDIAN
#define READ_LE( dst_, size_)  (fread ((dst_), 1, (size_), stdin ) == (size_))
#define WRITE_LE(src_, size_)  (fwrite((src_), 1, (size_), stdout) == (size_))
#define READ_BE( dst_, size_)  (fread ((dst_), (size_), 1, stdin ) == 1)
#define WRITE_BE(src_, size_)  (fwrite((src_), (size_), 1, stdout) == 1)
#else
#error "Unsupported __BYTE_ORDER"
#endif


int ReadDummy(YM7128B_Float* dst) {
    *dst = 0;
    return 1;
}

int ReadU8(YM7128B_Float* dst) {
    int8_t src;
    if (READ_LE(&src, sizeof(src))) {
        src += INT8_MIN;
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT8_MIN;
        return 1;
    }
    return 0;
}

int ReadS8(YM7128B_Float* dst) {
    int8_t src;
    if (READ_LE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT8_MIN;
        return 1;
    }
    return 0;
}

int ReadU16L(YM7128B_Float* dst) {
    int16_t src;
    if (READ_LE(&src, sizeof(src))) {
        src += INT16_MIN;
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT16_MIN;
        return 1;
    }
    return 0;
}

int ReadU16B(YM7128B_Float* dst) {
    int16_t src;
    if (READ_BE(&src, sizeof(src))) {
        src += INT16_MIN;
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT16_MIN;
        return 1;
    }
    return 0;
}

int ReadS16L(YM7128B_Float* dst) {
    int16_t src;
    if (READ_LE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT16_MIN;
        return 1;
    }
    return 0;
}

int ReadS16B(YM7128B_Float* dst) {
    int16_t src;
    if (READ_BE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT16_MIN;
        return 1;
    }
    return 0;
}

int ReadU32L(YM7128B_Float* dst) {
    int32_t src;
    if (READ_LE(&src, sizeof(src))) {
        src += INT32_MIN;
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT32_MIN;
        return 1;
    }
    return 0;
}

int ReadU32B(YM7128B_Float* dst) {
    int32_t src;
    if (READ_BE(&src, sizeof(src))) {
        src += INT32_MIN;
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT32_MIN;
        return 1;
    }
    return 0;
}

int ReadS32L(YM7128B_Float* dst) {
    int32_t src;
    if (READ_LE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT32_MIN;
        return 1;
    }
    return 0;
}

int ReadS32B(YM7128B_Float* dst) {
    int32_t src;
    if (READ_BE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT32_MIN;
        return 1;
    }
    return 0;
}

int ReadU64L(YM7128B_Float* dst) {
    int64_t src;
    if (READ_LE(&src, sizeof(src))) {
        src += INT64_MIN;
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT64_MIN;
        return 1;
    }
    return 0;
}

int ReadU64B(YM7128B_Float* dst) {
    int64_t src;
    if (READ_BE(&src, sizeof(src))) {
        src += INT64_MIN;
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT64_MIN;
        return 1;
    }
    return 0;
}

int ReadS64L(YM7128B_Float* dst) {
    int64_t src;
    if (READ_LE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT64_MIN;
        return 1;
    }
    return 0;
}

int ReadS64B(YM7128B_Float* dst) {
    int64_t src;
    if (READ_BE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src / -(YM7128B_Float)INT64_MIN;
        return 1;
    }
    return 0;
}

int ReadF32L(YM7128B_Float* dst) {
    float src;
    if (READ_LE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src;
        return 1;
    }
    return 0;
}

int ReadF32B(YM7128B_Float* dst) {
    float src;
    if (READ_BE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src;
        return 1;
    }
    return 0;
}

int ReadF64L(YM7128B_Float* dst) {
    double src;
    if (READ_LE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src;
        return 1;
    }
    return 0;
}

int ReadF64B(YM7128B_Float* dst) {
    double src;
    if (READ_BE(&src, sizeof(src))) {
        *dst = (YM7128B_Float)src;
        return 1;
    }
    return 0;
}


int WriteDummy(YM7128B_Float src) {
    (void)src;
    return 1;
}

int WriteU8(YM7128B_Float src) {
    double scaled = src * -(double)INT8_MIN;
    int8_t dst = (int8_t)fmin(fmax(scaled, INT8_MIN), INT8_MAX);
    dst -= INT8_MIN;
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteS8(YM7128B_Float src) {
    double scaled = src * -(double)INT8_MIN;
    int8_t dst = (int8_t)fmin(fmax(scaled, INT8_MIN), INT8_MAX);
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteU16L(YM7128B_Float src) {
    double scaled = src * -(double)INT16_MIN;
    int16_t dst = (int16_t)fmin(fmax(scaled, INT16_MIN), INT16_MAX);
    dst -= INT16_MIN;
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteU16B(YM7128B_Float src) {
    double scaled = src * -(double)INT16_MIN;
    int16_t dst = (int16_t)fmin(fmax(scaled, INT16_MIN), INT16_MAX);
    dst -= INT16_MIN;
    return WRITE_BE(&dst, sizeof(dst));
}

int WriteS16L(YM7128B_Float src) {
    double scaled = src * -(double)INT16_MIN;
    int16_t dst = (int16_t)fmin(fmax(scaled, INT16_MIN), INT16_MAX);
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteS16B(YM7128B_Float src) {
    double scaled = src * -(double)INT16_MIN;
    int16_t dst = (int16_t)fmin(fmax(scaled, INT16_MIN), INT16_MAX);
    return WRITE_BE(&dst, sizeof(dst));
}

int WriteU32L(YM7128B_Float src) {
    double scaled = src * -(double)INT32_MIN;
    int32_t dst = (int32_t)fmin(fmax(scaled, INT32_MIN), INT32_MAX);
    dst -= INT32_MIN;
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteU32B(YM7128B_Float src) {
    double scaled = src * -(double)INT32_MIN;
    int32_t dst = (int32_t)fmin(fmax(scaled, INT32_MIN), INT32_MAX);
    dst -= INT32_MIN;
    return WRITE_BE(&dst, sizeof(dst));
}

int WriteS32L(YM7128B_Float src) {
    double scaled = src * -(double)INT32_MIN;
    int32_t dst = (int32_t)fmin(fmax(scaled, INT32_MIN), INT32_MAX);
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteS32B(YM7128B_Float src) {
    double scaled = src * -(double)INT32_MIN;
    int32_t dst = (int32_t)fmin(fmax(scaled, INT32_MIN), INT32_MAX);
    return WRITE_BE(&dst, sizeof(dst));
}

int WriteU64L(YM7128B_Float src) {
    double scaled = src * -(double)INT64_MIN;
    int64_t dst = (int64_t)fmin(fmax(scaled, (double)INT64_MIN), (double)INT64_MAX);
    dst -= INT64_MIN;
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteU64B(YM7128B_Float src) {
    double scaled = src * -(double)INT64_MIN;
    int64_t dst = (int64_t)fmin(fmax(scaled, (double)INT64_MIN), (double)INT64_MAX);
    dst -= INT64_MIN;
    return WRITE_BE(&dst, sizeof(dst));
}

int WriteS64L(YM7128B_Float src) {
    double scaled = src * -(double)INT64_MIN;
    int64_t dst = (int64_t)fmin(fmax(scaled, (double)INT64_MIN), (double)INT64_MAX);
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteS64B(YM7128B_Float src) {
    double scaled = src * -(double)INT64_MIN;
    int64_t dst = (int64_t)fmin(fmax(scaled, (double)INT64_MIN), (double)INT64_MAX);
    return WRITE_BE(&dst, sizeof(dst));
}

int WriteF32L(YM7128B_Float src) {
    float dst = (float)src;
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteF32B(YM7128B_Float src) {
    float dst = (float)src;
    return WRITE_BE(&dst, sizeof(dst));
}

int WriteF64L(YM7128B_Float src) {
    double dst = (double)src;
    return WRITE_LE(&dst, sizeof(dst));
}

int WriteF64B(YM7128B_Float src) {
    double dst = (double)src;
    return WRITE_BE(&dst, sizeof(dst));
}


typedef int (*STREAM_READER)(YM7128B_Float* dst);
typedef int (*STREAM_WRITER)(YM7128B_Float src);

struct FormatTable {
    char const* label;
    STREAM_READER reader;
    STREAM_WRITER writer;
} const FORMAT_TABLE[] =
{
    { "dummy",      ReadDummy, WriteDummy },
    { "U8",         ReadU8,    WriteU8    },
    { "S8",         ReadS8,    WriteS8    },
    { "U16_LE",     ReadU16L,  WriteU16L  },
    { "U16_BE",     ReadU16B,  WriteU16B  },
    { "S16_LE",     ReadS16L,  WriteS16L  },
    { "S16_BE",     ReadS16B,  WriteS16B  },
    { "U32_LE",     ReadU32L,  WriteU32L  },
    { "U32_BE",     ReadU32B,  WriteU32B  },
    { "S32_LE",     ReadS32L,  WriteS32L  },
    { "S32_BE",     ReadS32B,  WriteS32B  },
    { "FLOAT_LE",   ReadF32L,  WriteF32L  },
    { "FLOAT_BE",   ReadF32B,  WriteF32B  },
    { "FLOAT64_LE", ReadF64L,  WriteF64L  },
    { "FLOAT64_BE", ReadF64B,  WriteF64B  },
    { NULL,         NULL,      NULL       }
};


struct ChipModeTable {
    char const* label;
    YM7128B_ChipEngine value;
} const MODE_TABLE[] =
{
    { "fixed", YM7128B_ChipEngine_Fixed },
    { "float", YM7128B_ChipEngine_Float },
    { "ideal", YM7128B_ChipEngine_Ideal },
    { "short", YM7128B_ChipEngine_Short },
    { NULL,    YM7128B_ChipEngine_Count }
};


struct RegisterTable {
    char const* label;
    YM7128B_Reg value;
} const REGISTER_TABLE[] =
{
    { "GL1", YM7128B_Reg_GL1 },
    { "GL2", YM7128B_Reg_GL2 },
    { "GL3", YM7128B_Reg_GL3 },
    { "GL4", YM7128B_Reg_GL4 },
    { "GL5", YM7128B_Reg_GL5 },
    { "GL6", YM7128B_Reg_GL6 },
    { "GL7", YM7128B_Reg_GL7 },
    { "GL8", YM7128B_Reg_GL8 },

    { "GR1", YM7128B_Reg_GR1 },
    { "GR2", YM7128B_Reg_GR2 },
    { "GR3", YM7128B_Reg_GR3 },
    { "GR4", YM7128B_Reg_GR4 },
    { "GR5", YM7128B_Reg_GR5 },
    { "GR6", YM7128B_Reg_GR6 },
    { "GR7", YM7128B_Reg_GR7 },
    { "GR8", YM7128B_Reg_GR8 },

    { "VM",  YM7128B_Reg_VM },
    { "VC",  YM7128B_Reg_VC },
    { "VL",  YM7128B_Reg_VL },
    { "VR",  YM7128B_Reg_VR },

    { "C0",  YM7128B_Reg_C0 },
    { "C1",  YM7128B_Reg_C1 },

    { "T0",  YM7128B_Reg_T0 },
    { "T1",  YM7128B_Reg_T1 },
    { "T2",  YM7128B_Reg_T2 },
    { "T3",  YM7128B_Reg_T3 },
    { "T4",  YM7128B_Reg_T4 },
    { "T5",  YM7128B_Reg_T5 },
    { "T6",  YM7128B_Reg_T6 },
    { "T7",  YM7128B_Reg_T7 },
    { "T8",  YM7128B_Reg_T8 },

    { NULL,  YM7128B_Reg_Count }
};


struct PresetTable {
    char const* label;
    YM7128B_Register regs[YM7128B_Reg_Count];
} const PRESET_TABLE[] =
{
    { "off", {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    }},
    { "direct", {
        0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x3F, 0x00, 0x3F, 0x3F,
        0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    }},
    { "gold/recital_hall", {
        0x1F, 0x3E, 0x1D, 0x3C, 0x1B, 0x3A, 0x19, 0x38,
        0x3F, 0x1E, 0x3D, 0x1C, 0x3B, 0x1A, 0x39, 0x18,
        0x18, 0x1C, 0x1C, 0x1C,
        0x15, 0x15,
        0x14, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10, 0x12,
    }},
    { "gold/concert_hall", {
        0x31, 0x00, 0x15, 0x00, 0x39, 0x00, 0x1D, 0x00,
        0x00, 0x33, 0x00, 0x17, 0x00, 0x3B, 0x00, 0x1F,
        0x1A, 0x1C, 0x1D, 0x1D,
        0x16, 0x16,
        0x1F, 0x1C, 0x19, 0x16, 0x13, 0x10, 0x0D, 0x0A, 0x07,
    }},
    { "gold/chapel", {
        0x1F, 0x1E, 0x1D, 0x1C, 0x1B, 0x1A, 0x19, 0x18,
        0x3F, 0x3E, 0x3D, 0x3C, 0x3B, 0x3A, 0x39, 0x38,
        0x38, 0x3D, 0x1B, 0x1B,
        0x10, 0x10,
        0x1F, 0x1F, 0x1D, 0x1B, 0x19, 0x17, 0x15, 0x13, 0x11,
    }},
    { "gold/jazz_club", {
        0x1F, 0x1B, 0x37, 0x13, 0x2F, 0x0B, 0x27, 0x03,
        0x1F, 0x3B, 0x17, 0x33, 0x0F, 0x2B, 0x07, 0x23,
        0x1C, 0x1F, 0x1B, 0x1B,
        0x0C, 0x0C,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    }},
    { "gold/movie_theater", {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1C, 0x1C,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    }},
    { "gold/stadium", {
        0x1F, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x33, 0x00,
        0x00, 0x1D, 0x00, 0x19, 0x00, 0x15, 0x00, 0x11,
        0x1D, 0x1D, 0x3D, 0x3D,
        0x13, 0x13,
        0x06, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
    }},
    { "gold/cavern", {
        0x1F, 0x00, 0x1D, 0x00, 0x1B, 0x00, 0x19, 0x00,
        0x20, 0x3E, 0x20, 0x3C, 0x20, 0x3A, 0x20, 0x38,
        0x3C, 0x3E, 0x1C, 0x1C,
        0x11, 0x0A,
        0x12, 0x10, 0x0E, 0x0C, 0x0A, 0x08, 0x06, 0x04, 0x02,
    }},
    { "gold/deep_space", {
        0x18, 0x00, 0x1A, 0x00, 0x1C, 0x00, 0x1E, 0x00,
        0x00, 0x19, 0x00, 0x1B, 0x00, 0x1D, 0x00, 0x1F,
        0x1B, 0x1F, 0x1C, 0x1C,
        0x12, 0x08,
        0x1F, 0x07, 0x0A, 0x0D, 0x10, 0x13, 0x16, 0x19, 0x1C,
    }},
    { "dune/arrakis", {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1A, 0x1A,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    }},
    { "dune/baghdad", {
        0x1F, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x33, 0x00,
        0x00, 0x1D, 0x00, 0x19, 0x00, 0x15, 0x00, 0x11,
        0x1D, 0x1D, 0x1D, 0x1D,
        0x13, 0x13,
        0x06, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
    }},
    { "dune/morning", {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1B, 0x1B,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    }},
    { "dune/sequence", {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1C, 0x1C,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    }},
    { "dune/sietch", {
        0x1F, 0x00, 0x1B, 0x00, 0x17, 0x00, 0x33, 0x00,
        0x00, 0x1D, 0x00, 0x19, 0x00, 0x15, 0x00, 0x11,
        0x1D, 0x1D, 0x1D, 0x1D,
        0x13, 0x13,
        0x06, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x0C, 0x0E, 0x10,
    }},
    { "dune/warsong", {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1C, 0x1C,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    }},
    { "dune/water", {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x1A, 0x1A,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    }},
    { "dune/wormintro", {
        0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07, 0x00,
        0x00, 0x1F, 0x00, 0x17, 0x00, 0x0F, 0x00, 0x07,
        0x1A, 0x1D, 0x18, 0x18,
        0x16, 0x16,
        0x1F, 0x03, 0x07, 0x0B, 0x0F, 0x13, 0x17, 0x1B, 0x1F,
    }},
    { "dune/wormsuit", {
        0x18, 0x00, 0x1A, 0x00, 0x1C, 0x00, 0x1E, 0x00,
        0x00, 0x19, 0x00, 0x1B, 0x00, 0x1D, 0x00, 0x1F,
        0x1B, 0x1F, 0x17, 0x17,
        0x12, 0x08,
        0x1F, 0x07, 0x0A, 0x0D, 0x10, 0x13, 0x16, 0x19, 0x1C,
    }},
    { NULL, {0} }
};


typedef struct Args {
    STREAM_READER stream_reader;
    STREAM_WRITER stream_writer;
    YM7128B_Float dry;
    YM7128B_Float wet;
    YM7128B_TapIdeal rate;
    YM7128B_ChipEngine chip_engine;
    YM7128B_Reg regs[YM7128B_Reg_Count];
} Args;


static uint8_t HexToByte(char const* str);
static int RunFixed(Args const* args);
static int RunFloat(Args const* args);
static int RunIdeal(Args const* args);
static int RunShort(Args const* args);


static uint8_t HexToByte(char const* str)
{
    uint8_t value = 0;
    for (int i = 0; i < 2; ++i) {
        char c = str[i];
        if (c >= '0' && c <= '9') {
            value |= (uint8_t)(c - '0');
        }
        else if (c >= 'a' && c <= 'f') {
            value |= (uint8_t)((c - 'a') + 10);
        }
        else if (c >= 'A' && c <= 'F') {
            value |= (uint8_t)((c - 'A') + 10);
        }
        else {
            errno = ERANGE;
            return 0;
        }
        value <<= 4;
    }
    return value;
}


int main(int argc, char const* argv[])
{
    Args args;
    args.stream_reader = ReadU8;
    args.stream_writer = WriteU8;
    args.dry = 1;
    args.wet = 1;
    args.rate = (YM7128B_TapIdeal)YM7128B_Input_Rate;
    args.chip_engine = YM7128B_ChipEngine_Fixed;
    for (YM7128B_Address r = 0; r < (YM7128B_Address)YM7128B_Reg_Count; ++r) {
        args.regs[r] = 0;
    }

    for (int i = 1; i < argc; ++i) {
        // Unary arguments
        if (!strcmp(argv[i], "-h") ||
            !strcmp(argv[i], "--help")) {
            puts(USAGE);
            return 0;
        }

        // Binary arguments
        if (i >= argc - 1) {
            fprintf(stderr, "Expecting binary argument: %s\n", argv[i]);
            return 1;
        }
        else if (!strcmp(argv[i], "--dry")) {
            long db = strtol(argv[++i], NULL, 10);
            if (errno) {
                fprintf(stderr, "Invalid decibels: %s\n", argv[i]);
                return 1;
            }
            if (db <= -128 || db >= 128) {
                args.dry = 0;
            }
            else {
                args.dry = (YM7128B_Float)pow(10, (double)db / 20);
            }
        }
        else if (!strcmp(argv[i], "-f") || !strcmp(argv[i], "--format")) {
            char const* label = argv[++i];
            int j;
            for (j = 0; FORMAT_TABLE[j].label; ++j) {
                if (!strcmp(label, FORMAT_TABLE[j].label)) {
                    args.stream_reader = FORMAT_TABLE[j].reader;
                    args.stream_writer = FORMAT_TABLE[j].writer;
                    break;
                }
            }
            if (!FORMAT_TABLE[j].label) {
                fprintf(stderr, "Unknown format: %s\n", label);
                return 1;
            }
        }
        else if (!strcmp(argv[i], "-e") || !strcmp(argv[i], "--engine")) {
            char const* label = argv[++i];
            int j;
            for (j = 0; MODE_TABLE[j].label; ++j) {
                if (!strcmp(label, MODE_TABLE[j].label)) {
                    args.chip_engine = MODE_TABLE[j].value;
                    break;
                }
            }
            if (!MODE_TABLE[j].label) {
                fprintf(stderr, "Unknown engine: %s\n", label);
                return 1;
            }
        }
        else if (!strcmp(argv[i], "--preset")) {
            char const* label = argv[++i];
            int j;
            for (j = 0; PRESET_TABLE[j].label; ++j) {
                if (!strcmp(label, PRESET_TABLE[j].label)) {
                    break;
                }
            }
            if (!PRESET_TABLE[j].label) {
                fprintf(stderr, "Unknown preset: %s\n", label);
                return 1;
            }
            YM7128B_Register const* data = PRESET_TABLE[j].regs;
            for (YM7128B_Address r = 0; r < (YM7128B_Address)YM7128B_Reg_Count; ++r) {
                args.regs[r] = data[r];
            }
        }
        else if (!strcmp(argv[i], "-r") || !strcmp(argv[i], "--rate")) {
            long rate = strtol(argv[++i], NULL, 10);
            if (errno || rate < 1) {
                fprintf(stderr, "Invalid rate: %s\n", argv[i]);
                return 1;
            }
            args.rate = (YM7128B_TapIdeal)rate;
        }
        else if (!strncmp(argv[i], "--reg-", 6)) {
            char const* label = &argv[i][6];
            int r;
            for (r = 0; REGISTER_TABLE[r].label; ++r) {
                if (!strcmp(label, REGISTER_TABLE[r].label)) {
                    break;
                }
            }
            if (!REGISTER_TABLE[r].label) {
                fprintf(stderr, "Unknown register: %s\n", label);
                return 1;
            }
            long value = strtol(argv[++i], NULL, 16);
            if (errno || value < 0x00 || value > 0xFF) {
                fprintf(stderr, "Invalid register value: %s\n", argv[i]);
                return 1;
            }
            args.regs[REGISTER_TABLE[r].value] = (YM7128B_Register)value;
        }
        else if (!strcmp(argv[i], "--regdump")) {
            char const* strptr = argv[++i];
            size_t length = strlen(strptr) / 2;
            if (length > (size_t)YM7128B_Reg_Count) {
                length = (size_t)YM7128B_Reg_Count;
            }
            for (size_t r = 0; r < length; ++r) {
                uint8_t value = HexToByte(strptr);
                strptr += 2;
                if (errno) {
                    fprintf(stderr, "Invalid HEX string: %s\n", argv[i]);
                    return 1;
                }
                args.regs[r] = value;
            }
            for (size_t r = length; r < (size_t)YM7128B_Reg_Count; ++r) {
                args.regs[r] = 0;
            }
        }
        else if (!strcmp(argv[i], "--wet")) {
            long db = strtol(argv[++i], NULL, 10);
            if (errno) {
                fprintf(stderr, "Invalid decibels: %s\n", argv[i]);
                return 1;
            }
            if (db <= -128 || db >= 128) {
                args.wet = 0;
            }
            else {
                args.wet = (YM7128B_Float)pow(10, (double)db / 20);
            }
        }
        else {
            fprintf(stderr, "Unknown switch: %s\n", argv[i]);
            return 1;
        }

        if (errno) {
            fprintf(stderr, "arg %d", i);
            perror("");
            return 1;
        }
    }

#ifdef __WINDOWS__
    _setmode(_fileno(stdin), O_BINARY);
    if (errno) {
        perror("_setmode(stdin)");
        return 1;
    }

    _setmode(_fileno(stdout), O_BINARY);
    if (errno) {
        perror("_setmode(stdout)");
        return 1;
    }
#endif  // __WINDOWS__

    switch (args.chip_engine)
    {
    case YM7128B_ChipEngine_Fixed:
        return RunFixed(&args);

    case YM7128B_ChipEngine_Float:
        return RunFloat(&args);

    case YM7128B_ChipEngine_Ideal:
        return RunIdeal(&args);

    case YM7128B_ChipEngine_Short:
        return RunShort(&args);

    default:
        return 1;
    }
}


static int RunFixed(Args const* args)
{
    YM7128B_ChipFixed* chip;
    chip = (YM7128B_ChipFixed*)malloc(sizeof(YM7128B_ChipFixed));
    if (!chip) {
        return 1;
    }
    YM7128B_ChipFixed_Ctor(chip);
    YM7128B_ChipFixed_Reset(chip);
    for (YM7128B_Address r = 0; r < (YM7128B_Address)YM7128B_Reg_Count; ++r) {
        YM7128B_ChipFixed_Write(chip, r, args->regs[r]);
    }
    YM7128B_ChipFixed_Start(chip);
    int error = 0;

    while (!feof(stdin)) {
        YM7128B_ChipFixed_Process_Data data;

        for (int c = 0; c < YM7128B_InputChannel_Count; ++c) {
            YM7128B_Float value;
            if (!args->stream_reader(&value)) {
                if (ferror(stdin)) {
                    perror("stream_reader()");
                    error = 1;
                }
                goto end;
            }
            YM7128B_Float const k = (YM7128B_Float)YM7128B_Fixed_Max;
            value = YM7128B_ClampFloat(value);
            data.inputs[c] = (YM7128B_Fixed)(value * k);
        }

        YM7128B_ChipFixed_Process(chip, &data);

        for (int c = 0; c < YM7128B_OutputChannel_Count; ++c) {
            for (int ovs = 0; ovs < YM7128B_Oversampling; ++ovs) {
                YM7128B_Float const k = ((YM7128B_Float)1 / (YM7128B_Float)YM7128B_Fixed_Max);
                YM7128B_Float wet = ((YM7128B_Float)data.outputs[c][ovs] * k);
                YM7128B_Float dry = ((YM7128B_Float)data.inputs[YM7128B_InputChannel_Mono] * k);
                YM7128B_Float value = (dry * args->dry) + (wet * args->wet);
                if (!args->stream_writer(value)) {
                    perror("stream_writer()");
                    error = 1;
                    goto end;
                }
            }
        }
    }

end:
    YM7128B_ChipFixed_Stop(chip);
    YM7128B_ChipFixed_Dtor(chip);
    free(chip);
    return error;
}


static int RunFloat(Args const* args)
{
    YM7128B_ChipFloat* chip;
    chip = (YM7128B_ChipFloat*)malloc(sizeof(YM7128B_ChipFloat));
    if (!chip) {
        return 1;
    }
    YM7128B_ChipFloat_Ctor(chip);
    YM7128B_ChipFloat_Reset(chip);
    for (YM7128B_Address r = 0; r < (YM7128B_Address)YM7128B_Reg_Count; ++r) {
        YM7128B_ChipFloat_Write(chip, r, args->regs[r]);
    }
    YM7128B_ChipFloat_Start(chip);
    int error = 0;

    while (!feof(stdin)) {
        YM7128B_ChipFloat_Process_Data data;

        for (int c = 0; c < YM7128B_InputChannel_Count; ++c) {
            if (!args->stream_reader(&data.inputs[c])) {
                if (ferror(stdin)) {
                    perror("stream_reader()");
                    error = 1;
                }
                goto end;
            }
        }

        YM7128B_ChipFloat_Process(chip, &data);

        for (int c = 0; c < YM7128B_OutputChannel_Count; ++c) {
            for (int ovs = 0; ovs < YM7128B_Oversampling; ++ovs) {
                YM7128B_Float wet = data.outputs[c][ovs];
                YM7128B_Float dry = data.inputs[YM7128B_InputChannel_Mono];
                YM7128B_Float value = (dry * args->dry) + (wet * args->wet);
                if (!args->stream_writer(value)) {
                    perror("stream_writer()");
                    error = 1;
                    goto end;
                }
            }
        }
    }

end:
    YM7128B_ChipFloat_Stop(chip);
    YM7128B_ChipFloat_Dtor(chip);
    free(chip);
    return error;
}


static int RunIdeal(Args const* args)
{
    YM7128B_ChipIdeal* chip;
    chip = (YM7128B_ChipIdeal*)malloc(sizeof(YM7128B_ChipIdeal));
    if (!chip) {
        return 1;
    }
    YM7128B_ChipIdeal_Ctor(chip);
    YM7128B_ChipIdeal_Setup(chip, args->rate);
    YM7128B_ChipIdeal_Reset(chip);
    for (YM7128B_Address r = 0; r < (YM7128B_Address)YM7128B_Reg_Count; ++r) {
        YM7128B_ChipIdeal_Write(chip, r, args->regs[r]);
    }
    YM7128B_ChipIdeal_Start(chip);
    int error = 0;

    while (!feof(stdin)) {
        YM7128B_ChipIdeal_Process_Data data;

        for (int c = 0; c < YM7128B_InputChannel_Count; ++c) {
            if (!args->stream_reader(&data.inputs[c])) {
                if (ferror(stdin)) {
                    perror("stream_reader()");
                    error = 1;
                }
                goto end;
            }
        }

        YM7128B_ChipIdeal_Process(chip, &data);

        for (int c = 0; c < YM7128B_OutputChannel_Count; ++c) {
            YM7128B_Float wet = data.outputs[c];
            YM7128B_Float dry = data.inputs[YM7128B_InputChannel_Mono];
            YM7128B_Float value = (dry * args->dry) + (wet * args->wet);
            if (!args->stream_writer(value)) {
                perror("stream_writer()");
                error = 1;
                goto end;
            }
        }
    }

end:
    YM7128B_ChipIdeal_Stop(chip);
    YM7128B_ChipIdeal_Dtor(chip);
    free(chip);
    return error;
}


static int RunShort(Args const* args)
{
    YM7128B_ChipShort* chip;
    chip = (YM7128B_ChipShort*)malloc(sizeof(YM7128B_ChipShort));
    if (!chip) {
        return 1;
    }
    YM7128B_ChipShort_Ctor(chip);
    YM7128B_ChipShort_Setup(chip, args->rate);
    YM7128B_ChipShort_Reset(chip);
    for (YM7128B_Address r = 0; r < (YM7128B_Address)YM7128B_Reg_Count; ++r) {
        YM7128B_ChipShort_Write(chip, r, args->regs[r]);
    }
    YM7128B_ChipShort_Start(chip);
    int error = 0;

    while (!feof(stdin)) {
        YM7128B_ChipShort_Process_Data data;

        for (int c = 0; c < YM7128B_InputChannel_Count; ++c) {
            YM7128B_Float value;
            if (!args->stream_reader(&value)) {
                if (ferror(stdin)) {
                    perror("stream_reader()");
                    error = 1;
                }
                goto end;
            }
            YM7128B_Float const k = (YM7128B_Float)YM7128B_Fixed_Max;
            value = YM7128B_ClampFloat(value);
            data.inputs[c] = (YM7128B_Fixed)(value * k);
        }

        YM7128B_ChipShort_Process(chip, &data);

        for (int c = 0; c < YM7128B_OutputChannel_Count; ++c) {
            YM7128B_Float const k = ((YM7128B_Float)1 / (YM7128B_Float)YM7128B_Fixed_Max);
            YM7128B_Float wet = ((YM7128B_Float)data.outputs[c] * k);
            YM7128B_Float dry = ((YM7128B_Float)data.inputs[YM7128B_InputChannel_Mono] * k);
            YM7128B_Float value = (dry * args->dry) + (wet * args->wet);
            if (!args->stream_writer(value)) {
                perror("stream_writer()");
                error = 1;
                goto end;
            }
        }
    }

end:
    YM7128B_ChipShort_Stop(chip);
    YM7128B_ChipShort_Dtor(chip);
    free(chip);
    return error;
}
