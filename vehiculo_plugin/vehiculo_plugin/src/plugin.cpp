// VehiculoSim.dll: simulador de auto (Vehiculo.cpp) para GTA SA 1.0 US. No usa el SDK de CLEO: el script carga la DLL
// con 0AA2 / 0AA4 y llama a estas funciones con 0AA5 / 0AA7 (todas __stdcall, ver exports.def).
//
// Modo "marioneta": el simulador es la autoridad. En cada SimTick se corre la fisica y se escribe en el auto del juego
// la matriz y las velocidades. Si el juego cambio la velocidad de golpe (choque con otro auto, explosion) o el script
// movio el auto (teletransporte, viaje en el tiempo), el simulador adopta el estado del juego en lugar de pisarlo.
#include <windows.h>

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unordered_map>

#include "Colision.h"
#include "Vehiculo.h"
#include "sa.h"

namespace {

constexpr float kImpacto = 2.0f;       // m/s de cambio de velocidad que el simulador no explica: choque del juego
constexpr float kTeletransporte = 4.0f;  // m de salto de posicion: el script movio el auto

struct Ranura {
    Vehiculo sim;
    bool tienePose = false;
    float ultPos[3] = {0, 0, 0};
    float ultVel[3] = {0, 0, 0};
    unsigned ticks = 0;
};

std::unordered_map<uintptr_t, Ranura*> g_autos;
colision::Mundo g_mundo;
float g_humedad = 0;
char g_dir[MAX_PATH] = "";

void ruta(const char* archivo, char* out, size_t n) { std::snprintf(out, n, "%s%s", g_dir, archivo); }

void log(const char* fmt, ...) {
    char p[MAX_PATH]; ruta("VehiculoSim.log", p, sizeof p);
    FILE* f = std::fopen(p, "a");
    if (!f) return;
    va_list ap; va_start(ap, fmt); std::vfprintf(f, fmt, ap); va_end(ap);
    std::fputc('\n', f); std::fclose(f);
}

float leerIni(const char* sec, const char* clave, float def, const char* ini) {
    char buf[64];
    GetPrivateProfileStringA(sec, clave, "", buf, sizeof buf, ini);
    return buf[0] ? static_cast<float>(std::atof(buf)) : def;
}

// Perfil: [Auto_<modelo>] sobre [Auto] sobre los valores por defecto de DatosVehiculo. Ruedas simetricas.
DatosVehiculo cargarPerfil(int modelo) {
    DatosVehiculo d;
    char ini[MAX_PATH]; ruta("vehiculo.ini", ini, sizeof ini);
    char sec[2][32] = {"Auto", ""};
    std::snprintf(sec[1], sizeof sec[1], "Auto_%d", modelo);
    float ruedaX = 0.85f, yDel = 1.4f, yTras = -1.4f, ruedaZ = -0.3f;
    for (const auto& s : sec) {
        d.medioLargo = leerIni(s, "medioLargo", d.medioLargo, ini);
        d.medioAncho = leerIni(s, "medioAncho", d.medioAncho, ini);
        d.radioRueda = leerIni(s, "radioRueda", d.radioRueda, ini);
        d.radioRuedaTras = leerIni(s, "radioRuedaTras", d.radioRueda, ini);
        d.velMax = leerIni(s, "velMax", d.velMax, ini);
        d.aceleracion = leerIni(s, "aceleracion", d.aceleracion, ini);
        d.masa = leerIni(s, "masa", d.masa, ini);
        d.cgY = leerIni(s, "cgY", d.cgY, ini);
        d.cgZ = leerIni(s, "cgZ", d.cgZ, ini);
        d.agarre = leerIni(s, "agarre", d.agarre, ini);
        d.traccion = static_cast<int>(leerIni(s, "traccion", static_cast<float>(d.traccion), ini));
        d.frenos = leerIni(s, "frenos", d.frenos, ini);
        d.repartoFreno = leerIni(s, "repartoFreno", d.repartoFreno, ini);
        d.bloqueo = leerIni(s, "bloqueo", d.bloqueo, ini);
        d.frecSusp = leerIni(s, "frecSusp", d.frecSusp, ini);
        d.deposito = leerIni(s, "deposito", d.deposito, ini);
        ruedaX = leerIni(s, "ruedaX", ruedaX, ini);
        yDel = leerIni(s, "ruedaYDelantera", yDel, ini);
        yTras = leerIni(s, "ruedaYTrasera", yTras, ini);
        ruedaZ = leerIni(s, "ruedaZ", ruedaZ, ini);
    }
    const float r[4][3] = {{-ruedaX, yDel, ruedaZ}, {ruedaX, yDel, ruedaZ}, {-ruedaX, yTras, ruedaZ}, {ruedaX, yTras, ruedaZ}};
    std::memcpy(d.rueda, r, sizeof r);
    return d;
}

// Estado del juego -> simulador (posicion, rumbo, velocidades).
void importar(Ranura& r, void* car) {
    const float* m = sa::matriz(car);
    const float yaw = std::atan2(m[sa::kFwd / 4 + 1], m[sa::kFwd / 4]);
    r.sim.colocar(m[sa::kPos / 4], m[sa::kPos / 4 + 1], m[sa::kPos / 4 + 2], yaw);
    const float* ms = sa::campo(car, sa::kMoveSpeed);
    const float* ts = sa::campo(car, sa::kTurnSpeed);
    float v[3], w[3];
    for (int k = 0; k < 3; ++k) { v[k] = ms[k] * 50.0f; w[k] = ts[k] * 50.0f; }
    r.sim.ponerVelocidadMundo(v, w);
}

float dist3(const float* a, const float* b) {
    return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + (a[1] - b[1]) * (a[1] - b[1]) + (a[2] - b[2]) * (a[2] - b[2]));
}

Ranura* buscar(void* car) {
    auto it = g_autos.find(reinterpret_cast<uintptr_t>(car));
    return it == g_autos.end() ? nullptr : it->second;
}

}  // namespace

extern "C" {

int __stdcall SimVersion() { return 1; }

// Crea el simulador para un auto del juego (puntero de 0A97). Devuelve 1 si quedo enganchado.
int __stdcall SimAttach(void* car) {
    if (!car || !sa::matriz(car)) return 0;
    if (buscar(car)) return 1;
    auto* r = new Ranura();
    const int modelo = *reinterpret_cast<const uint16_t*>(reinterpret_cast<uintptr_t>(car) + sa::kModelIndex);
    r->sim.configurar(cargarPerfil(modelo));
    importar(*r, car);
    r->sim.alternarMotor();
    g_autos[reinterpret_cast<uintptr_t>(car)] = r;
    log("attach car=%p modelo=%d", car, modelo);
    return 1;
}

void __stdcall SimDetach(void* car) {
    auto it = g_autos.find(reinterpret_cast<uintptr_t>(car));
    if (it == g_autos.end()) return;
    delete it->second;
    g_autos.erase(it);
    log("detach car=%p", car);
}

// gas -1..1 (adelante / atras), dir -1 izquierda .. 1 derecha, mano 0/1 (freno de mano). Devuelve 1 si simulo.
int __stdcall SimTick(void* car, float gas, float dir, float mano) {
    Ranura* r = buscar(car);
    float* M = car ? sa::matriz(car) : nullptr;
    if (!r || !M) return 0;
    float dt = sa::leerF(sa::kTimeStep) * sa::kSegPorUnidad;
    if (!(dt > 0.0f)) return 0;
    if (dt > 0.05f) dt = 0.05f;

    // 1) cambios del juego que el simulador no explica -> adoptar el estado del juego
    const float* P = M + sa::kPos / 4;
    const float* ms = sa::campo(car, sa::kMoveSpeed);
    if (r->tienePose) {
        const float* ts = sa::campo(car, sa::kTurnSpeed);
        const float vj[3] = {ms[0] * 50.0f, ms[1] * 50.0f, ms[2] * 50.0f};
        const float wj[3] = {ts[0] * 50.0f, ts[1] * 50.0f, ts[2] * 50.0f};
        const float salto = dist3(P, r->ultPos);
        const float dv = dist3(vj, r->ultVel);
        if (salto > kTeletransporte) {
            importar(*r, car);
            log("resync: teletransporte %.1f m", salto);
        } else if (dv > kImpacto) {
            r->sim.ponerVelocidadMundo(vj, wj);
            log("resync: impacto dv=%.1f m/s", dv);
        }
    }

    // 2) fisica
    EntradaVehiculo in;
    in.gas = gas; in.direccion = dir; in.freno = mano > 0.5f;
    g_mundo.ignorar = car;
    r->sim.ponerHumedad(g_humedad);
    r->sim.actualizar(dt, in, g_mundo);

    // 3) escribir en el juego
    float m16[16];
    r->sim.matriz(m16);
    for (int k = 0; k < 3; ++k) {
        M[sa::kRight / 4 + k] = m16[0 + k];
        M[sa::kFwd / 4 + k] = m16[4 + k];
        M[sa::kUp / 4 + k] = m16[8 + k];
        M[sa::kPos / 4 + k] = m16[12 + k];
    }
    float v[3], w[3];
    r->sim.velocidadMundo(v);
    r->sim.velAngularMundo(w);
    float* mv = sa::campo(car, sa::kMoveSpeed);
    float* tv = sa::campo(car, sa::kTurnSpeed);
    for (int k = 0; k < 3; ++k) { mv[k] = v[k] / 50.0f; tv[k] = w[k] / 50.0f; r->ultPos[k] = m16[12 + k]; r->ultVel[k] = v[k]; }
    r->tienePose = true;
    if (++r->ticks == 1 || r->ticks % 600 == 0) log("tick %u car=%p v=%.1f km/h marcha=%d rpm=%.0f z=%.2f", r->ticks, car, r->sim.velocidad() * 3.6f, r->sim.marcha(), r->sim.rpm(), m16[14]);
    return 1;
}

void __stdcall SimMotor(void* car) { if (Ranura* r = buscar(car)) r->sim.alternarMotor(); }
void __stdcall SimServicio(void* car, int completo) { if (Ranura* r = buscar(car)) r->sim.servicio(completo != 0); }
void __stdcall SimPonerHumedad(float h) { g_humedad = h < 0 ? 0 : (h > 1 ? 1 : h); }

// out[8] = km/h, marcha, rpm, temp motor (C), combustible (L), derrape, motor encendido (0/1), salud motor (0..1)
void __stdcall SimTelemetria(void* car, float* out) {
    Ranura* r = buscar(car);
    if (!r || !out) return;
    out[0] = r->sim.velocidad() * 3.6f; out[1] = static_cast<float>(r->sim.marcha()); out[2] = r->sim.rpm();
    out[3] = r->sim.tempMotor(); out[4] = r->sim.combustible(); out[5] = r->sim.derrape();
    out[6] = r->sim.motorEncendido() ? 1.0f : 0.0f; out[7] = r->sim.saludMotor();
}

}  // extern "C"

BOOL APIENTRY DllMain(HINSTANCE h, DWORD razon, LPVOID) {
    if (razon == DLL_PROCESS_ATTACH) {
        char p[MAX_PATH];
        GetModuleFileNameA(h, p, MAX_PATH);
        char* barra = std::strrchr(p, '\\');
        if (barra) barra[1] = 0;
        std::strncpy(g_dir, p, MAX_PATH - 1);
        log("VehiculoSim cargado");
    }
    return TRUE;
}
