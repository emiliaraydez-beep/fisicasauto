#pragma once
// Stub minimo SOLO para compilar plugin.cpp en Linux como comprobacion de sintaxis.
#include <cstdint>
typedef int BOOL; typedef unsigned long DWORD; typedef void* LPVOID; typedef void* HINSTANCE; typedef const char* LPCSTR; typedef char* LPSTR;
#define APIENTRY
#define MAX_PATH 260
#define TRUE 1
#define DLL_PROCESS_ATTACH 1
#define __stdcall
#define __cdecl
inline DWORD GetPrivateProfileStringA(LPCSTR, LPCSTR, LPCSTR, LPSTR b, DWORD, LPCSTR) { b[0] = 0; return 0; }
inline DWORD GetModuleFileNameA(HINSTANCE, LPSTR b, DWORD) { b[0] = 0; return 0; }
