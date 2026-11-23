// Prueba del nucleo sin el juego: acelera, frena y gira sobre suelo plano.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include "Vehiculo.h"
int main() {
    colision::Mundo mundo;
    Vehiculo v;
    v.colocar(0, 0, 0.6f, 0);
    v.alternarMotor();  // llave: sin esto el motor esta apagado
    EntradaVehiculo in;
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 120; ++i) v.actualizar(dt, in, mundo);  // 2 s de arranque
    float t = 0, t100 = -1;
    bool ok = true;
    in.gas = 1;
    for (int i = 0; i < 60 * 25; ++i, t += dt) {
        v.actualizar(dt, in, mundo);
        if (t100 < 0 && v.velocidad() * 3.6f >= 100.0f) t100 = t;
        if (i % 300 == 0) std::printf("t=%5.1fs  v=%6.1f km/h  marcha=%d  rpm=%5.0f  z=%.2f\n", t, v.velocidad() * 3.6f, v.marcha(), v.rpm(), v.pos().z);
        if (!std::isfinite(v.pos().x) || !std::isfinite(v.pos().z)) { ok = false; break; }
    }
    std::printf("0-100 km/h: %.1f s | final %.1f km/h\n", t100, v.velocidad() * 3.6f);
    in.gas = -1;
    float d0 = v.pos().x, tf = 0;
    while (v.velocidad() > 0.3f && tf < 20) { v.actualizar(dt, in, mundo); tf += dt; }
    std::printf("frenada: %.1f s, %.0f m\n", tf, std::fabs(v.pos().x - d0));
    v.colocar(0, 0, 0.6f, 0);
    if (!v.motorEncendido()) v.alternarMotor();
    float vel[3] = {16.7f, 0, 0}, w[3] = {0, 0, 0};
    v.ponerVelocidadMundo(vel, w);
    in.gas = 0.35f; in.direccion = 0.5f;
    float amax = 0;
    for (int i = 0; i < 60 * 4; ++i) { v.actualizar(dt, in, mundo); amax = std::max(amax, std::fabs(v.alabeo())); }
    std::printf("curva: yaw=%.0f grados, alabeo max=%.1f grados, derrape=%.2f, z=%.2f\n", v.yaw() * 57.2958f, amax * 57.2958f, v.derrape(), v.pos().z);
    std::printf(ok ? "OK (sin NaN)\n" : "FALLO: NaN\n");
    return ok ? 0 : 1;
}
