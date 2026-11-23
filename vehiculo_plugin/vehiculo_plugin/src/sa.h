#pragma once
// Enlaces con GTA San Andreas 1.0 US (HOODLUM / sin parches). TODO lo que toca memoria del juego esta en este archivo.
// Los offsets de CEntity/CPhysical y el dt de CTimer son los mismos que ya usa tu hover.cs. Las direcciones de funciones
// (ProcessLineOfSight) y el layout de CColPoint salen de la tabla conocida de la 1.0 US: VERIFICAR con el log antes de
// confiar (ver README, "Primera prueba").
#include <cstdint>

namespace sa {

constexpr uintptr_t kTimeStep = 0xB7CB5C;       // float CTimer::ms_fTimeStep (1.0 = 1/50 s). hover.cs lo lee como 12045148.
constexpr uintptr_t kProcessLineOfSight = 0x56BA00;

// Offsets en CVehicle (CEntity -> CPhysical)
constexpr int kModelIndex = 0x22;  // uint16 CEntity::m_nModelIndex
constexpr int kMatrixPtr = 0x14;   // CMatrix* CPlaceable::m_matrix  (hover.cs: "car struct + 20")
constexpr int kMoveSpeed = 0x44;   // CVector, unidades de juego por fotograma de 1/50 s (1.0 = 50 m/s = 180 km/h)
constexpr int kTurnSpeed = 0x50;   // CVector, rad por fotograma de 1/50 s, en ejes del mundo
// Offsets en CMatrix
constexpr int kRight = 0x00, kFwd = 0x10, kUp = 0x20, kPos = 0x30;

constexpr float kSegPorUnidad = 0.02f;  // 1 unidad de ms_fTimeStep = 20 ms

struct CColPoint {  // 0x2C bytes
    float px, py, pz, pad0;
    float nx, ny, nz, pad1;
    uint8_t surfA, pieceA, lightA, surfB, pieceB, lightB, pad2[2];
    float depth;
};
static_assert(sizeof(CColPoint) == 0x2C, "CColPoint");

// bool CWorld::ProcessLineOfSight(CVector& origin, CVector& target, CColPoint&, CEntity*&, bool buildings, bool vehicles,
//                                 bool peds, bool objects, bool dummies, bool seeThrough, bool ignoreSomeObjectsForCamera, bool shootThrough)
using FnLOS = bool(__cdecl*)(const float* origen, const float* destino, CColPoint& cp, void*& entidad, bool edificios,
                             bool vehiculos, bool peatones, bool objetos, bool dummies, bool atravesar, bool ignorarCamara,
                             bool disparar);
inline FnLOS LOS() { return reinterpret_cast<FnLOS>(kProcessLineOfSight); }

inline float leerF(uintptr_t a) { return *reinterpret_cast<const float*>(a); }
inline float* matriz(void* auto_) {
    return *reinterpret_cast<float**>(reinterpret_cast<uintptr_t>(auto_) + kMatrixPtr);
}
inline float* campo(void* auto_, int off) { return reinterpret_cast<float*>(reinterpret_cast<uintptr_t>(auto_) + off); }

}  // namespace sa
