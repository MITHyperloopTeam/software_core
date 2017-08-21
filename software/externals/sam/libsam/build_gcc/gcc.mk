#
#  Copyright (c) 2011 Arduino.  All right reserved.
#
#  This library is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 2.1 of the License, or (at your option) any later version.
#
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this library; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
#

# Tool suffix when cross-compiling
CROSS_COMPILE = arm-none-eabi-

# Compilation tools
AR = $(CROSS_COMPILE)ar
CC = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++
AS = $(CROSS_COMPILE)as
NM = $(CROSS_COMPILE)nm
ifeq ($(OS),Windows_NT)
RM=cs-rm -Rf
else
RM=rm -Rf
endif

SEP=/

# ---------------------------------------------------------------------------------------
# C Flags

CFLAGS_SAM += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS_SAM += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS_SAM += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS_SAM += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CFLAGS_SAM += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS_SAM += -Wsign-compare -Waggregate-return -Wstrict-prototypes
CFLAGS_SAM += -Wmissing-prototypes -Wmissing-declarations
CFLAGS_SAM += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS_SAM += -Wpacked -Wredundant-decls -Wnested-externs -Winline -Wlong-long
CFLAGS_SAM += -Wunreachable-code
CFLAGS_SAM += -Wcast-align
#CFLAGS_SAM += -Wmissing-noreturn
#CFLAGS_SAM += -Wconversion

CFLAGS_SAM += --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb -mlong-calls -ffunction-sections -fdata-sections -nostdlib -std=gnu99
# -pedantic
CFLAGS_SAM += $(OPTIMIZATION) $(INCLUDES) $(INCLUDES_SAM) -D$(CHIP)

# To reduce application size use only integer printf function.
CFLAGS_SAM += -Dprintf=iprintf

# ---------------------------------------------------------------------------------------
# CPP Flags

CPPFLAGS_SAM += -Wall -Wchar-subscripts -Wcomment -Wformat=2
CPPFLAGS_SAM += -Wmain -Wparentheses -Wcast-align -Wunreachable-code
CPPFLAGS_SAM += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CPPFLAGS_SAM += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CPPFLAGS_SAM += -Wshadow -Wpointer-arith -Wwrite-strings
CPPFLAGS_SAM += -Wsign-compare -Waggregate-return -Wmissing-declarations
CPPFLAGS_SAM += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CPPFLAGS_SAM += -Wpacked -Wredundant-decls -Winline -Wlong-long
#CPPFLAGS_SAM += -Wmissing-noreturn
#CPPFLAGS_SAM += -Wconversion

CPPFLAGS_SAM += --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb -mlong-calls -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -std=c++98
CPPFLAGS_SAM += $(OPTIMIZATION) $(INCLUDES) $(INCLUDES_SAM) -D$(CHIP)

# To reduce application size use only integer printf function.
CPPFLAGS_SAM += -Dprintf=iprintf

# ---------------------------------------------------------------------------------------
# ASM Flags

ASFLAGS_SAM = -mcpu=cortex-m3 -mthumb -Wall -g $(OPTIMIZATION) $(INCLUDES) $(INCLUDES_SAM)
