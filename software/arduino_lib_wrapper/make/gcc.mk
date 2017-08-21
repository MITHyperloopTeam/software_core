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
# Compilation tools
AR_SIM = ar
CC_SIM = gcc
CXX_SIM = g++
AS_SIM = as
NM_SIM = nm
ifeq ($(OS),Windows_NT)
RM=cs-rm -Rf
else
RM=rm -Rf
endif

SEP=/

# ---------------------------------------------------------------------------------------
# C Flags

CFLAGS_SIM += -Wall -Wchar-subscripts -Wcomment -Wformat=2 -Wimplicit-int
CFLAGS_SIM += -Werror-implicit-function-declaration -Wmain -Wparentheses
CFLAGS_SIM += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CFLAGS_SIM += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CFLAGS_SIM += -Wshadow -Wpointer-arith -Wbad-function-cast -Wwrite-strings
CFLAGS_SIM += -Wsign-compare -Waggregate-return -Wstrict-prototypes
CFLAGS_SIM += -Wmissing-prototypes -Wmissing-declarations
CFLAGS_SIM += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS_SIM += -Wpacked -Wredundant-decls -Wnested-externs -Winline -Wlong-long
CFLAGS_SIM += -Wunreachable-code
CFLAGS_SIM += -Wcast-align -g -Woverloaded-virtual
#CFLAGS_SIM += -Wmissing-noreturn
#CFLAGS_SIM += -Wconversion

CFLAGS_SIM += --param max-inline-insns-single=500 -ffunction-sections -fdata-sections -nostdlib -std=c99 
# -pedantic
CFLAGS_SIM += $(OPTIMIZATION) $(INCLUDES) $(INCLUDES_SIM) -D$(CHIP)

# ---------------------------------------------------------------------------------------
# CPP Flags

CPPFLAGS_SIM += -Wall -Wchar-subscripts -Wcomment -Wformat=2
CPPFLAGS_SIM += -Wmain -Wparentheses -Wcast-align -Wunreachable-code
CPPFLAGS_SIM += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wunused
CPPFLAGS_SIM += -Wuninitialized -Wunknown-pragmas -Wfloat-equal -Wundef
CPPFLAGS_SIM += -Wshadow -Wpointer-arith -Wwrite-strings
CPPFLAGS_SIM += -Wsign-compare -Waggregate-return -Wmissing-declarations
CPPFLAGS_SIM += -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CPPFLAGS_SIM += -Wpacked -Wredundant-decls -Winline -Wlong-long -g
#CPPFLAGS_SIM += -Wmissing-noreturn
#CPPFLAGS_SIM += -Wconversion

CPPFLAGS_SIM += --param max-inline-insns-single=500 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -std=c++98 
CPPFLAGS_SIM += $(OPTIMIZATION) $(INCLUDES) $(INCLUDES_SIM) -D$(CHIP)

# ---------------------------------------------------------------------------------------
# ASM Flags

ASFLAGS_SIM = -Wall -g $(OPTIMIZATION) $(INCLUDES) $(INCLUDES_SIM)
