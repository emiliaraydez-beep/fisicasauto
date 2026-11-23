// Mundo de prueba: suelo plano en z = 0, asfalto, sin paredes.
#include "Colision.h"
namespace colision {
bool Mundo::suelo(float, float, float zDesde, float maxBajada, float& zSuelo, V3* n, uint8_t* m) const {
    if (zDesde < 0.0f || zDesde - maxBajada > 0.0f) return false;
    zSuelo = 0.0f;
    if (n) { n->x = 0; n->y = 0; n->z = 1; }
    if (m) *m = 0;
    return true;
}
bool Mundo::contactoEsfera(V3, float, Contacto&) const { return false; }
}  // namespace colision
