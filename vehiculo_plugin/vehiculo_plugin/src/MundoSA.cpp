// colision::Mundo sobre el mundo estatico de GTA SA (edificios y objetos), con CWorld::ProcessLineOfSight.
// Los vehiculos y peatones NO se consultan: de los choques con ellos se encarga el juego (ver SimTick en plugin.cpp).
#include <cmath>

#include "Colision.h"
#include "sa.h"

namespace colision {

static bool rayo(const float* o, const float* d, sa::CColPoint& cp) {
    void* ent = nullptr;
    return sa::LOS()(o, d, cp, ent, /*edificios*/ true, /*vehiculos*/ false, /*peatones*/ false, /*objetos*/ true,
                     /*dummies*/ false, /*atravesar*/ false, /*camara*/ false, /*disparar*/ false);
}

bool Mundo::suelo(float x, float y, float zDesde, float maxBajada, float& zSuelo, V3* normal, uint8_t* material) const {
    const float o[3] = {x, y, zDesde}, d[3] = {x, y, zDesde - maxBajada};
    sa::CColPoint cp;
    if (!rayo(o, d, cp)) return false;
    zSuelo = cp.pz;
    if (normal) { normal->x = cp.nx; normal->y = cp.ny; normal->z = cp.nz; }
    if (material) *material = cp.surfB;
    return true;
}

// Aproximacion: 8 rayos horizontales desde el centro, de longitud = radio. Vale para paredes (el simulador ignora
// normales casi verticales); la profundidad es radio - distancia al impacto.
bool Mundo::contactoEsfera(V3 c, float r, Contacto& out) const {
    static const float kDir[8][2] = {{1, 0}, {0.7071f, 0.7071f}, {0, 1}, {-0.7071f, 0.7071f},
                                     {-1, 0}, {-0.7071f, -0.7071f}, {0, -1}, {0.7071f, -0.7071f}};
    bool hay = false;
    float mejor = 0;
    for (const auto& k : kDir) {
        const float o[3] = {c.x, c.y, c.z}, d[3] = {c.x + k[0] * r, c.y + k[1] * r, c.z};
        sa::CColPoint cp;
        if (!rayo(o, d, cp)) continue;
        const float t = std::sqrt((cp.px - c.x) * (cp.px - c.x) + (cp.py - c.y) * (cp.py - c.y));
        const float prof = r - t;
        if (prof > mejor) {
            mejor = prof; hay = true;
            out.normal.x = cp.nx; out.normal.y = cp.ny; out.normal.z = cp.nz;
            out.profundidad = prof;
        }
    }
    return hay;
}

}  // namespace colision
