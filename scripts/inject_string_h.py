# Workaround: framework-arduinopico's deprecated-avr-comp/avr/pgmspace.h
# expands memcpy_P -> memcpy without including <string.h>. Force-include it
# for C/C++ TUs only (not .S, which would break the assembler).
Import("env")
env.Append(CFLAGS=["-include", "string.h"], CXXFLAGS=["-include", "string.h"])
