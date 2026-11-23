#include "Vehiculo.h"

#include <algorithm>
#include <cmath>

using colision::Contacto;
using colision::V3;

namespace {
constexpr float kG = 9.81f;
constexpr float kPi = 3.14159265358979f;

// --- entorno ---
constexpr float kRho = 1.2f;    // kg/m^3
constexpr float kCdA = 0.68f;   // Cd * area frontal (m^2)
constexpr float kClA = 0.12f;   // carga aerodinamica (Cl * area)
constexpr float kCrr0 = 0.010f;   // resistencia a la rodadura en asfalto a baja velocidad
constexpr float kCrrV = 0.0095f;  // lo que suma a 100 km/h (crece con el cuadrado de la velocidad)

// --- motor / transmision ---
constexpr float kRpmRalenti = 900.0f, kRpmMax = 6800.0f, kRpmPico = 4200.0f, kRpmEmb = 2400.0f;
constexpr float kMarchas[5] = {3.5f, 2.2f, 1.5f, 1.1f, 0.85f};
constexpr float kMarchaAtras = 3.2f;
constexpr float kEta = 0.90f;   // rendimiento de la transmision
constexpr float kIw = 1.3f;     // inercia de una rueda (kg m^2)
constexpr float kIe = 0.08f;    // inercia del motor + volante (kg m^2)

// --- neumatico transitorio y termica ---
constexpr float kRelajLong = 0.25f, kRelajLat = 0.55f;  // longitud de relajacion (m): el deslizamiento tarda ese recorrido en "llegar"
constexpr float kTempAmb = 25.0f, kTempNeuIni = 70.0f, kTempNeuIdeal = 85.0f;  // grados C
constexpr float kCalorNeu = 6000.0f;   // J/K de la banda de rodadura
constexpr float kCalorFric = 0.35f;    // fraccion de la potencia de deslizamiento que calienta el neumatico
// --- frenos ---
constexpr float kCalorDiscoDel = 1800.0f, kCalorDiscoTras = 1200.0f;  // J/K de cada disco (algo menos que uno real: el fade se alcanza bajando una montana)
constexpr float kFadeIni = 380.0f, kFadeRango = 320.0f;               // grados C: desde ahi las pastillas pierden eficacia

// --- neumatico (curva tipo Magic Formula normalizada: pico en s = 1) ---
constexpr float kKappa0 = 0.12f;  // deslizamiento longitudinal del pico
constexpr float kAlfa0 = 0.14f;   // tangente del angulo de deriva del pico (~8 grados)
constexpr float kB = 2.08f, kC = 1.4f;

// --- suspension ---
constexpr float kSRest = 0.30f;      // anclaje -> centro de rueda en reposo
constexpr float kRecorrido = 0.30f;  // recorrido total hasta el tope
constexpr float kXTope = 0.25f;      // compresion a la que entra el tope
constexpr float kRigTope = 250000.0f;
constexpr float kMaxVolante = 0.62f;  // rad a baja velocidad
constexpr float kPasoMax = 1.0f / 200.0f;

// --- mecanica ---
constexpr float kTiempoJuego = 6.0f;      // el consumo de combustible y aceite corre x6: un deposito dura minutos, no horas
constexpr float kAmbiente = 25.0f;        // grados C
constexpr float kRalentiL = 0.0003f, kPlenaL = 0.0075f;  // litros/s al ralenti y a plena carga (antes de x6)
constexpr float kAceiteGasto = 0.00002f;  // nivel/s que quema el motor (antes de x6)
constexpr float kDesgaste = 1.0f / 2400.0f;  // salud/s a plena carga con aceite y temperatura normales
constexpr float kArranque = 0.7f, kArranqueBat = 0.08f;  // s de motor de arranque y carga/s que cuesta
constexpr float kAlternador = 0.006f;     // carga/s que devuelve con el motor en marcha

// --- contactos de carroceria (se activan al volcar, al tocar fondo o al perder ruedas) ---
constexpr float kRigCarr = 80000.0f, kAmortCarr = 3500.0f;

inline float dot(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline void cross(const float* a, const float* b, float* o) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
inline void norm3(float* a) {
    const float l = std::sqrt(dot(a, a));
    if (l > 1e-9f) { a[0] /= l; a[1] /= l; a[2] /= l; }
}
// Rodrigues: gira a alrededor del eje unitario k (cos c, sen s)
inline void rot(float* a, const float* k, float c, float s) {
    float kxa[3];
    cross(k, a, kxa);
    const float kd = dot(k, a) * (1.0f - c);
    for (int i = 0; i < 3; ++i) a[i] = a[i] * c + kxa[i] * s + k[i] * kd;
}

// Par del motor normalizado (1 en el pico), interpolado entre puntos de una curva de motor de gasolina atmosferico:
// par decente desde ralenti, meseta ancha cerca de 4000 rpm y caida hacia el corte.
float curvaPar(float rpm) {
    static const float R[] = {0.0f, 900.0f, 1500.0f, 2400.0f, 3200.0f, 4200.0f, 5000.0f, 5800.0f, 6500.0f, 6800.0f, 7500.0f};
    static const float P[] = {0.30f, 0.55f, 0.68f, 0.84f, 0.95f, 1.00f, 0.98f, 0.90f, 0.80f, 0.74f, 0.50f};
    constexpr int n = sizeof(R) / sizeof(R[0]);
    if (rpm <= R[0]) return P[0];
    for (int i = 1; i < n; ++i) {
        if (rpm <= R[i]) {
            const float t = (rpm - R[i - 1]) / (R[i] - R[i - 1]);
            return P[i - 1] + (P[i] - P[i - 1]) * t;
        }
    }
    return P[n - 1];
}

// Fuerza del neumatico sobre el auto: fx (adelante) y fy (a lo largo del eje lateral "derecha"), con deslizamiento
// longitudinal kappa y tangente de la deriva tanA combinados (circulo de friccion). 'rig' (1 = asfalto) estira la curva:
// en suelo suelto el pico llega con mas deslizamiento.
void neumatico(float mu, float fz, float kappa, float tanA, float rig, float& fx, float& fy) {
    const float kn = kappa * rig / kKappa0, an = tanA * rig / kAlfa0;
    const float s = std::sqrt(kn * kn + an * an);
    if (s < 1e-5f) { fx = fy = 0; return; }
    const float f = std::sin(kC * std::atan(kB * s));
    const float mag = mu * fz * f;
    fx = mag * kn / s;
    fy = -mag * an / s;
}

// Lo que muerden las pastillas segun la temperatura del disco: frias un poco menos, muy calientes pierden eficacia (fade).
float eficaciaDisco(float t) {
    const float frio = 0.90f + 0.10f * std::clamp((t - kTempAmb) / 100.0f, 0.0f, 1.0f);
    const float fade = 1.0f - 0.55f * std::clamp((t - kFadeIni) / kFadeRango, 0.0f, 1.0f);
    return frio * fade;
}

// Temperatura del neumatico: el agarre es maximo en la temperatura ideal y cae si esta frio o pasado.
float agarreTemp(float t) {
    const float d = (t - kTempNeuIdeal) / 55.0f;
    return std::max(0.70f, 1.0f - 0.07f * d * d);
}
}  // namespace

// ------------------------------------------------------------------------------------------------
// Superficies. Ids de material de GTA SA (surface.dat): 0 defecto, 1-5 asfalto y acera, 6 grava, 9-20 hierba, 21-23 terreno,
// 24-25 barro, 26-27 tierra, 28-33 arena, 34 concreto de playa, 35-37 roca, 38-39 agua baja, 40-41 cultivo / seto, 42-44 madera,
// 45-47 vidrio, 51-56 metal, 63-65 auto, 67-68 goma / plastico, 71-73 alfombra / tarima. Es una tabla: afinala a gusto.
const SuperficieInfo& superficieDeMaterial(unsigned char m) {
    static const SuperficieInfo kAsfalto = {"asfalto", 1.00f, 1.0f, 1.00f, true};
    static const SuperficieInfo kAsfaltoRoto = {"asfalto roto", 0.93f, 1.3f, 0.95f, true};
    static const SuperficieInfo kAcera = {"concreto", 0.95f, 1.1f, 0.98f, true};
    static const SuperficieInfo kGrava = {"grava", 0.62f, 2.4f, 0.70f, false};
    static const SuperficieInfo kHierba = {"hierba", 0.58f, 3.0f, 0.62f, false};
    static const SuperficieInfo kHierbaLarga = {"hierba alta", 0.50f, 4.0f, 0.58f, false};
    static const SuperficieInfo kTerreno = {"terreno", 0.66f, 2.4f, 0.70f, false};
    static const SuperficieInfo kBarro = {"barro", 0.38f, 4.5f, 0.50f, false};
    static const SuperficieInfo kTierra = {"tierra", 0.68f, 2.2f, 0.72f, false};
    static const SuperficieInfo kArena = {"arena", 0.52f, 5.5f, 0.55f, false};
    static const SuperficieInfo kArenaFina = {"arena suelta", 0.46f, 7.0f, 0.50f, false};
    static const SuperficieInfo kRoca = {"roca", 0.82f, 1.4f, 0.90f, true};
    static const SuperficieInfo kAgua = {"agua", 0.30f, 8.0f, 0.45f, false};
    static const SuperficieInfo kMadera = {"madera", 0.72f, 1.2f, 0.90f, true};
    static const SuperficieInfo kMetal = {"metal", 0.62f, 1.1f, 0.90f, true};
    static const SuperficieInfo kVidrio = {"vidrio", 0.40f, 1.1f, 0.85f, true};
    static const SuperficieInfo kGoma = {"goma", 0.90f, 1.8f, 0.90f, true};
    if (m == 0 || m == 1) return kAsfalto;
    if (m <= 3) return kAsfaltoRoto;
    if (m == 4 || m == 5 || m == 7 || m == 8 || m == 34) return kAcera;
    if (m == 6) return kGrava;
    if (m >= 9 && m <= 16) return kHierba;
    if (m >= 17 && m <= 20) return kHierbaLarga;
    if (m >= 21 && m <= 23) return kTerreno;
    if (m == 24 || m == 25) return kBarro;
    if (m == 26 || m == 27) return kTierra;
    if (m == 28 || m == 32) return kArenaFina;
    if (m >= 29 && m <= 33) return kArena;
    if (m >= 35 && m <= 37) return kRoca;
    if (m == 38 || m == 39) return kAgua;
    if (m == 40 || m == 41) return kHierbaLarga;
    if (m >= 42 && m <= 44) return kMadera;
    if (m >= 45 && m <= 47) return kVidrio;
    if (m >= 50 && m <= 66) return kMetal;
    if (m == 67 || m == 68) return kGoma;
    if (m >= 70 && m <= 73) return kMadera;
    return kAsfalto;
}

const SuperficieInfo& Vehiculo::superficie(int i) const { return superficieDeMaterial(mat_[(i >= 0 && i < 4) ? i : 0]); }
float Vehiculo::eficaciaFreno(int i) const { return (i >= 0 && i < 4) ? eficaciaDisco(tempFreno_[i]) : 1.0f; }
float Vehiculo::agarreNeumatico(int i) const { return (i >= 0 && i < 4) ? agarreTemp(tempNeu_[i]) : 1.0f; }

// ------------------------------------------------------------------------------------------------
float Vehiculo::yaw() const { return std::atan2(av_[1], av_[0]); }
float Vehiculo::cabeceo() const { return std::asin(std::clamp(av_[2], -1.0f, 1.0f)); }
float Vehiculo::alabeo() const { return std::atan2(-der_[2], arr_[2]); }
float Vehiculo::velocidadTotal() const { return std::sqrt(v_[0] * v_[0] + v_[1] * v_[1]); }
float Vehiculo::rpmMaximas() const { return kRpmMax; }

void Vehiculo::desdeOrigen() {
    cg_.x = pos_.x + av_[0] * d_.cgY + arr_[0] * d_.cgZ;
    cg_.y = pos_.y + av_[1] * d_.cgY + arr_[1] * d_.cgZ;
    cg_.z = pos_.z + av_[2] * d_.cgY + arr_[2] * d_.cgZ;
}

void Vehiculo::hacerOrigen() {
    pos_.x = cg_.x - av_[0] * d_.cgY - arr_[0] * d_.cgZ;
    pos_.y = cg_.y - av_[1] * d_.cgY - arr_[1] * d_.cgZ;
    pos_.z = cg_.z - av_[2] * d_.cgY - arr_[2] * d_.cgZ;
}

void Vehiculo::configurar(const DatosVehiculo& d) {
    d_ = d;
    for (bool& b : sinRueda_) b = false;
    motor_ = false;  // mecanica nueva: deposito lleno, motor apagado (al subir se arranca solo)
    arrancando_ = 0;
    combustible_ = std::max(1.0f, d_.deposito);
    aceite_ = refri_ = bateria_ = 1.0f;
    temp_ = kAmbiente;
    saludMotor_ = saludTrans_ = saludBat_ = 1.0f;
    fugaComb_ = fugaAceite_ = fugaRefri_ = 0.0f;
    pot_ = trans_ = 1.0f;
    aviso_ = "";
    avisoT_ = 0;
    nCarr_ = 0;
    float h = 0;
    for (int i = 0; i < 4; ++i) h += radio(i) - d_.rueda[i][2];
    altura_ = std::max(0.2f, h / 4.0f);

    masa_ = std::max(300.0f, d_.masa);
    batalla_ = std::max(1.5f, d_.rueda[0][1] - d_.rueda[2][1]);
    via_ = std::max(0.8f, d_.rueda[1][0] - d_.rueda[0][0]);
    const float lf = std::max(0.3f, d_.rueda[0][1] - d_.cgY);  // CG -> eje delantero
    const float lr = std::max(0.3f, d_.cgY - d_.rueda[2][1]);  // CG -> eje trasero
    const float L = lf + lr;

    // inercias de una caja con el largo y ancho del auto
    const float l = 2.0f * d_.medioLargo, w = 2.0f * d_.medioAncho, hh = 1.4f;
    Ip_ = masa_ * (l * l + hh * hh) / 12.0f * 1.1f;
    Ir_ = masa_ * (w * w + hh * hh) / 12.0f;
    Iy_ = masa_ * (l * l + w * w) / 12.0f * 1.1f;

    // suspension: misma frecuencia en las cuatro esquinas -> misma compresion estatica
    const float om = 2.0f * kPi * std::max(0.5f, d_.frecSusp);
    sRest_ = kSRest;
    sMax_ = sRest_ + kG / (om * om);
    sMin_ = sMax_ - kRecorrido;
    for (int i = 0; i < 4; ++i) {
        const float mc = masa_ * (i < 2 ? lr / L : lf / L) * 0.5f;  // masa suspendida que carga esa rueda
        kS_[i] = mc * om * om;
        cS_[i] = 2.0f * 0.30f * std::sqrt(kS_[i] * mc);
    }
    kArb_[0] = 18000.0f;
    kArb_[1] = 12000.0f;
    fz0_ = masa_ * kG / 4.0f;

    // reparto del par segun la traccion
    switch (d_.traccion) {
        case 1: frac_[0] = frac_[1] = 0.5f; frac_[2] = frac_[3] = 0.0f; break;
        case 2: frac_[0] = frac_[1] = 0.2f; frac_[2] = frac_[3] = 0.3f; break;
        default: frac_[0] = frac_[1] = 0.0f; frac_[2] = frac_[3] = 0.5f; break;
    }
    float rDrive = 0;
    for (int i = 0; i < 4; ++i) rDrive += frac_[i] * radio(i);

    // relacion final: la ultima marcha llega al corte de motor justo a velMax
    const float wRed = kRpmMax * 2.0f * kPi / 60.0f;
    fd_ = wRed * rDrive / (std::max(5.0f, d_.velMax) * 1.03f * kMarchas[4]);
    // par maximo: en 1a, con el par pico, la fuerza en la rueda da ~75% de 'aceleracion' (el resto lo come la inercia)
    tMax_ = 1.2f * 0.75f * d_.aceleracion * masa_ * rDrive / (kMarchas[0] * fd_ * kEta);
    tbMax_ = std::max(0.3f, d_.frenos) * masa_ * kG * 0.5f * (d_.radioRueda + d_.radioRuedaTras);
    d_.repartoFreno = std::clamp(d_.repartoFreno, 0.45f, 0.85f);

    for (int i = 0; i < 4; ++i) {
        sLen_[i] = sRest_;
        kF_[i] = aF_[i] = 0.0f;
        tempNeu_[i] = kTempNeuIni;
        tempFreno_[i] = kTempAmb;
        mat_[i] = 0;
    }
    giroLat_ = 0;

    // hover: perfil del DeLorean ajustado a este vehiculo (masa, aceleracion, velocidad, agarre)
    ph_ = PerfilHover{};
    ph_.ajustarA(d_, DatosVehiculo{});
    hEstado_ = HoverEstado::Suelo;
    hModo_ = HoverModo::SinSustentacion;
    hModoPrev_ = 3;
    hPose_ = hRampa_ = hCalor_ = hBoost_ = hThr_ = hVzCmd_ = hVzObj_ = 0;
    hSobre_ = hAterrizando_ = hLibre_ = hAuto_ = false;
    for (int i = 0; i < 4; ++i) { padSalud_[i] = 1.0f; libre_[i] = false; }
    for (int k = 0; k < 3; ++k) hCmd_[k] = hTurb_[k] = hTurbT_[k] = 0;
    hInt_[0] = hInt_[1] = 0;
    desdeOrigen();
}

void Vehiculo::colocar(float x, float y, float z, float yawRad) {
    pos_ = {x, y, z};
    seguro_ = pos_;
    yawSeguro_ = yawRad;
    const float ch = std::cos(yawRad), sh = std::sin(yawRad);
    av_[0] = ch;  av_[1] = sh;   av_[2] = 0;
    der_[0] = sh; der_[1] = -ch; der_[2] = 0;
    arr_[0] = 0;  arr_[1] = 0;   arr_[2] = 1;
    for (int k = 0; k < 3; ++k) v_[k] = wb_[k] = 0;
    for (int i = 0; i < 4; ++i) {
        omega_[i] = giroR_[i] = steer_[i] = xPrev_[i] = fz_[i] = 0;
        kF_[i] = aF_[i] = 0;
        sLen_[i] = sRest_;
        hitPrev_[i] = false;
        nrm_[i][0] = nrm_[i][1] = 0; nrm_[i][2] = 1;
    }
    thr_ = brk_ = dirNorm_ = volante_ = giroLat_ = 0;
    rpm_ = kRpmRalenti;
    marcha_ = 1;
    tCambio_ = 0;
    tUltCambio_ = 1.0f;
    slipMotriz_ = 0;
    tcsF_ = 1;
    vf_ = derrape_ = volcado_ = tSeguro_ = 0;
    nContactos_ = 0;
    bump_ = false;
    hEstado_ = HoverEstado::Suelo;  // colocar = auto nuevo en el suelo
    hPose_ = hRampa_ = hBoost_ = hThr_ = hVzCmd_ = hVzObj_ = 0;
    hSobre_ = hAterrizando_ = hLibre_ = hAuto_ = false;
    for (int k = 0; k < 3; ++k) hCmd_[k] = 0;
    desdeOrigen();
}

void Vehiculo::enderezar() {
    const float y = yaw();
    const float ch = std::cos(y), sh = std::sin(y);
    av_[0] = ch;  av_[1] = sh;   av_[2] = 0;
    der_[0] = sh; der_[1] = -ch; der_[2] = 0;
    arr_[0] = 0;  arr_[1] = 0;   arr_[2] = 1;
    for (int k = 0; k < 3; ++k) v_[k] = wb_[k] = 0;
    for (int i = 0; i < 4; ++i) omega_[i] = 0;
    pos_.z += 0.8f;
    desdeOrigen();
}

// Ejes del chasis en el mundo: derecha, avance, arriba.
void Vehiculo::ejes(float* der, float* av, float* arr) const {
    for (int k = 0; k < 3; ++k) { der[k] = der_[k]; av[k] = av_[k]; arr[k] = arr_[k]; }
}

void Vehiculo::matriz(float* m) const {
    for (int k = 0; k < 3; ++k) { m[k] = der_[k]; m[4 + k] = av_[k]; m[8 + k] = arr_[k]; }
    m[3] = m[7] = m[11] = 0;
    m[12] = pos_.x; m[13] = pos_.y; m[14] = pos_.z; m[15] = 1;
}

namespace {
void mul3(const float* a, const float* b, float* o) {  // matrices 3x3 por filas
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) o[r * 3 + c] = a[r * 3] * b[c] + a[r * 3 + 1] * b[3 + c] + a[r * 3 + 2] * b[6 + c];
}
}  // namespace

void Vehiculo::matrizRueda(int i, float* m) const {
    float d[3], a[3], u[3];
    ejes(d, a, u);
    const bool izq = (i % 2) == 0, delante = i < 2;
    const float s = delante ? -steer_[i] : 0.0f;  // steer > 0 = derecha = giro horario
    const float cs = std::cos(s), ss = std::sin(s), cg = std::cos(giroR_[i]), sg = std::sin(giroR_[i]);
    const float F = izq ? -1.0f : 1.0f;  // las ruedas izquierdas miran con la cara hacia -X: se giran 180 grados
    const float rz[9] = {cs, -ss, 0, ss, cs, 0, 0, 0, 1};
    const float rx[9] = {1, 0, 0, 0, cg, sg, 0, -sg, cg};  // Rx(-giro): la parte alta avanza
    const float fl[9] = {F, 0, 0, 0, F, 0, 0, 0, 1};
    float t[9], A[9];
    mul3(rz, rx, t);
    mul3(t, fl, A);
    if (hPose_ > 0.0f) {  // hover: las ruedas giran hasta quedar horizontales (propulsor hacia abajo)
        const float th = hPose_ * ph_.inclinacionMax * kPi / 180.0f, ct = std::cos(th), st = std::sin(th);
        const float ry[9] = {ct, 0, st, 0, 1, 0, -st, 0, ct};
        float B[9];
        mul3(ry, A, B);
        for (int q = 0; q < 9; ++q) A[q] = B[q];
    }
    const float esc = 2.0f * radio(i);
    for (int c = 0; c < 3; ++c) {  // columna c de A (en el chasis) -> mundo
        const float ax = A[0 * 3 + c], ay = A[1 * 3 + c], az = A[2 * 3 + c];
        for (int k = 0; k < 3; ++k) m[c * 4 + k] = (d[k] * ax + a[k] * ay + u[k] * az) * esc;
        m[c * 4 + 3] = 0;
    }
    const float* o = d_.rueda[i];
    const float oz = o[2] + (sRest_ - sLen_[i]);  // la suspension sube / baja la rueda
    m[12] = pos_.x + d[0] * o[0] + a[0] * o[1] + u[0] * oz;
    m[13] = pos_.y + d[1] * o[0] + a[1] * o[1] + u[1] * oz;
    m[14] = pos_.z + d[2] * o[0] + a[2] * o[1] + u[2] * oz;
    m[15] = 1;
}

// ------------------------------------------------------------------------------------------------
void Vehiculo::cambiar(int g) {
    const float t = (marcha_ >= 1 && g >= 1) ? 0.25f : 0.15f;
    marcha_ = g;
    tCambio_ = t;
    tUltCambio_ = 0.0f;
}

float Vehiculo::relacion(int g) const {
    if (g == 0) return 0.0f;
    if (g < 0) return -kMarchaAtras * fd_;
    return kMarchas[std::min(g, 5) - 1] * fd_;
}

float Vehiculo::rpmRuedas(float ratio) const {
    float wd = 0;
    for (int i = 0; i < 4; ++i) wd += frac_[i] * omega_[i];
    return std::fabs(wd * ratio) * 30.0f / kPi;
}

// ------------------------------------------------------------------------------------------------
// Tres esferas a lo largo del chasis empujan el auto fuera de paredes (con impulso real: tambien lo hacen girar).
// El suelo y las rampas suaves los maneja la suspension, asi que se ignoran los contactos casi horizontales.
void Vehiculo::resolverParedes(const colision::Mundo& mundo) {
    const float r = std::max(0.45f, d_.medioAncho * 0.8f);
    const float offs[3] = {-0.65f * d_.medioLargo, 0.0f, 0.65f * d_.medioLargo};
    float fx = av_[0], fy = av_[1];
    const float hl = std::sqrt(fx * fx + fy * fy);
    if (hl > 1e-3f) { fx /= hl; fy /= hl; } else { fx = -der_[1]; fy = der_[0]; }
    const float I3[3] = {Ip_, Ir_, Iy_};

    for (int iter = 0; iter < 3; ++iter) {
        bool movido = false;
        for (float off : offs) {
            const V3 c = {pos_.x + fx * off, pos_.y + fy * off, pos_.z + 0.30f};
            Contacto k;
            if (!mundo.contactoEsfera(c, r, k)) continue;
            if (std::fabs(k.normal.z) > 0.7f) continue;
            float hx = k.normal.x, hy = k.normal.y;
            const float l = std::sqrt(hx * hx + hy * hy);
            if (l < 1e-5f) continue;
            hx /= l; hy /= l;
            const float prof = k.profundidad * (l > 0.3f ? 1.0f / l : 3.3f);
            pos_.x += hx * prof; pos_.y += hy * prof;
            cg_.x += hx * prof;  cg_.y += hy * prof;

            // impulso en el punto de contacto (sobre la superficie de la esfera)
            const float rr[3] = {c.x - hx * r - cg_.x, c.y - hy * r - cg_.y, c.z - cg_.z};
            float w[3];
            for (int q = 0; q < 3; ++q) w[q] = der_[q] * wb_[0] + av_[q] * wb_[1] + arr_[q] * wb_[2];
            float wxr[3];
            cross(w, rr, wxr);
            const float vn = (v_[0] + wxr[0]) * hx + (v_[1] + wxr[1]) * hy;
            if (vn < 0.0f) {
                if (vn < -0.5f) choques_.push_back(Choque{V3{c.x - hx * r, c.y - hy * r, c.z}, V3{hx, hy, 0.0f}, -vn, k.instancia});
                const float n[3] = {hx, hy, 0.0f};
                float rn[3];
                cross(rr, n, rn);
                const float rb[3] = {dot(rn, der_), dot(rn, av_), dot(rn, arr_)};
                const float kinv = 1.0f / masa_ + rb[0] * rb[0] / I3[0] + rb[1] * rb[1] / I3[1] + rb[2] * rb[2] / I3[2];
                const float j = -(1.0f + 0.15f) * vn / kinv;
                v_[0] += j * hx / masa_;
                v_[1] += j * hy / masa_;
                for (int q = 0; q < 3; ++q) wb_[q] += j * rb[q] / I3[q];
                v_[0] *= 0.985f;  // roce con la pared
                v_[1] *= 0.985f;
            }
            movido = true;
        }
        if (!movido) break;
    }
}

// Ocho puntos de la carroceria (cuatro bajos, cuatro del techo) contra el suelo: al volcar o tocar fondo el auto no
// atraviesa el terreno y se desliza sobre el chasis.
void Vehiculo::carroceria(const colision::Mundo& mundo, const float* w, float* F, float* T) {
    for (int p = 0; p < 8; ++p) {
        const bool techo = p >= 4;
        const float sx = (p & 1) ? 1.0f : -1.0f, sy = (p & 2) ? 1.0f : -1.0f;
        const float lx = sx * d_.medioAncho * (techo ? 0.85f : 0.9f);
        const float ly = sy * d_.medioLargo * (techo ? 0.35f : 0.85f);
        const float lz = techo ? 0.60f : -0.45f;
        float q[3];
        for (int k = 0; k < 3; ++k) {
            const float o = (k == 0 ? pos_.x : (k == 1 ? pos_.y : pos_.z));
            q[k] = o + der_[k] * lx + av_[k] * ly + arr_[k] * lz;
        }
        float gz = 0;
        if (!mundo.suelo(q[0], q[1], q[2] + 1.0f, 4.0f, gz)) continue;
        const float pen = gz - q[2];
        if (pen <= 0.0f || pen > 1.5f) continue;
        const float r[3] = {q[0] - cg_.x, q[1] - cg_.y, q[2] - cg_.z};
        float wxr[3];
        cross(w, r, wxr);
        const float vx = v_[0] + wxr[0], vy = v_[1] + wxr[1], vz = v_[2] + wxr[2];
        const float N = std::max(0.0f, kRigCarr * pen - kAmortCarr * vz);
        if (N <= 0.0f) continue;
        ++nCarr_;
        const float vt = std::sqrt(vx * vx + vy * vy);
        const float f[3] = {-0.6f * N * vx / (vt + 0.5f), -0.6f * N * vy / (vt + 0.5f), N};
        float t[3];
        cross(r, f, t);
        for (int k = 0; k < 3; ++k) { F[k] += f[k]; T[k] += t[k]; }
    }
}

// ------------------------------------------------------------------------------------------------
// Un subpaso de integracion del cuerpo rigido.
void Vehiculo::paso(float h, const Mando& cmd, const colision::Mundo& mundo) {
    float F[3] = {0.0f, 0.0f, -masa_ * kG};
    float T[3] = {0.0f, 0.0f, 0.0f};
    const float cg[3] = {cg_.x, cg_.y, cg_.z};
    const float org[3] = {pos_.x, pos_.y, pos_.z};
    float w[3];
    for (int k = 0; k < 3; ++k) w[k] = der_[k] * wb_[0] + av_[k] * wb_[1] + arr_[k] * wb_[2];

    auto aplicar = [&](const float* p, const float* f) {  // fuerza f aplicada en el punto p (mundo)
        const float r[3] = {p[0] - cg[0], p[1] - cg[1], p[2] - cg[2]};
        float t[3];
        cross(r, f, t);
        for (int k = 0; k < 3; ++k) { F[k] += f[k]; T[k] += t[k]; }
    };

    // ruedas que no apoyan en este subpaso: arrancadas o plegadas en posicion de hover
    for (int i = 0; i < 4; ++i) libre_[i] = sinRueda_[i] || hLibre_;
    const bool volando = hLibre_ || (hEstado_ != HoverEstado::Suelo && hRampa_ > 0.3f);

    // --- aerodinamica: resistencia y carga (en hover la resistencia la pone el perfil de hover) ---
    if (!volando) {
        const float vh2 = v_[0] * v_[0] + v_[1] * v_[1];
        const float sp = std::sqrt(vh2 + v_[2] * v_[2]);
        for (int k = 0; k < 3; ++k) F[k] -= 0.5f * kRho * kCdA * sp * v_[k];
        F[2] -= 0.5f * kRho * kClA * vh2;
    }

    // --- 1) suspension: un rayo hacia abajo por rueda ---
    float mp[4][3], cp[4][3];
    float x[4] = {0, 0, 0, 0}, xd[4] = {0, 0, 0, 0};
    bool hit[4];
    nContactos_ = 0;
    for (int i = 0; i < 4; ++i) {
        const float* o = d_.rueda[i];
        const float oz = o[2] + sRest_;
        for (int k = 0; k < 3; ++k) mp[i][k] = org[k] + der_[k] * o[0] + av_[k] * o[1] + arr_[k] * oz;
        hit[i] = false;
        float s = sMax_, gz = 0;
        unsigned char mt = 0;
        if (!libre_[i] && mundo.suelo(mp[i][0], mp[i][1], mp[i][2] + 0.25f, 4.0f, gz, nullptr, &mt)) {
            const float hh = std::max(0.0f, mp[i][2] - gz);
            const float upn = std::max(dot(arr_, nrm_[i]), 0.3f);
            const float sc = hh * nrm_[i][2] / upn - radio(i);  // longitud de suspension hasta tocar el suelo
            if (sc < sMax_) { hit[i] = true; s = std::max(sc, sMin_ - 0.15f); mat_[i] = mt; }
        }
        if (hit[i]) {
            x[i] = sMax_ - s;
            xd[i] = hitPrev_[i] ? std::clamp((x[i] - xPrev_[i]) / h, -6.0f, 6.0f) : 0.0f;
            const float t = s + radio(i);
            for (int k = 0; k < 3; ++k) cp[i][k] = mp[i][k] - arr_[k] * t;
            ++nContactos_;
        }
        sLen_[i] = s;
        xPrev_[i] = x[i];
        hitPrev_[i] = hit[i];
    }

    float Fs[4] = {0, 0, 0, 0};
    bump_ = false;
    for (int i = 0; i < 4; ++i) {
        if (!hit[i]) continue;
        const float c = cS_[i] * (xd[i] > 0.0f ? 0.8f : 1.3f);  // bump mas blando que rebote
        Fs[i] = kS_[i] * x[i] + c * xd[i];
        if (x[i] > kXTope) { Fs[i] += kRigTope * (x[i] - kXTope); bump_ = true; }
    }
    for (int ax = 0; ax < 2; ++ax) {  // barra estabilizadora
        const int l = ax * 2, r = l + 1;
        if (hit[l] && hit[r]) {
            const float arb = kArb_[ax] * (x[l] - x[r]);
            Fs[l] += arb;
            Fs[r] -= arb;
        }
    }
    for (int i = 0; i < 4; ++i) {
        fz_[i] = hit[i] ? std::clamp(Fs[i], 0.0f, 1.2f * masa_ * kG) : 0.0f;
        if (hit[i]) {
            const float f[3] = {fz_[i] * arr_[0], fz_[i] * arr_[1], fz_[i] * arr_[2]};
            aplicar(mp[i], f);
        }
    }

    // --- 2) transmision: par en cada rueda ---
    float Td[4] = {0, 0, 0, 0}, Iref[4] = {0, 0, 0, 0};
    {
        // control de traccion: con el patinaje del subpaso anterior recorta el par (con un poco de suavizado)
        const float objTcs = d_.tcs ? std::clamp(1.0f - (slipMotriz_ - 0.18f) / 0.20f, 0.25f, 1.0f) : 1.0f;
        tcsF_ += (objTcs - tcsF_) * std::min(1.0f, (objTcs < tcsF_ ? 60.0f : 20.0f) * h);  // corta rapido, devuelve el par mas despacio
        const float ratio = relacion(marcha_);
        if (marcha_ != 0 && tCambio_ <= 0.0f) {
            float wd = 0;
            for (int i = 0; i < 4; ++i) wd += frac_[i] * omega_[i];
            const float rpmW = std::fabs(wd * ratio) * 30.0f / kPi;
            const bool on = motor_;  // apagado: no hay par, pero las ruedas arrastran el motor (freno motor)
            const bool desliz = on && thr_ > 0.02f && rpmW < kRpmEmb;  // embrague patinando al arrancar
            float Te = 0;
            if (desliz) {
                rpm_ = std::max(rpmW, 1500.0f + 1500.0f * thr_);
                Te = thr_ * tMax_ * curvaPar(rpm_) * pot_ * tcsF_;
            } else {
                // embrague cerrado: el motor sigue a las ruedas con la elasticidad de la transmision (no salta de golpe)
                const float rpmObj = std::max(rpmW, on ? kRpmRalenti : 0.0f);
                rpm_ += (rpmObj - rpm_) * std::min(1.0f, 40.0f * h);
                const float lim = std::clamp((kRpmMax - rpm_) / 150.0f, 0.0f, 1.0f);  // corte de motor
                Te = thr_ * tMax_ * curvaPar(rpm_) * lim * pot_ * tcsF_;
                if (wd * ratio > 0.0f)  // freno motor
                    Te -= (on ? 1.0f - thr_ : 0.8f) * tMax_ * (0.08f + 0.12f * rpm_ / kRpmMax) * std::clamp((rpmW - 800.0f) / 800.0f, 0.0f, 1.0f);
            }
            const float Twh = Te * ratio * kEta * trans_;
            for (int ax = 0; ax < 2; ++ax) {
                const int l = ax * 2, r = l + 1;
                const float fa = frac_[l] + frac_[r];
                if (fa <= 0.0f) continue;
                const float Tax = Twh * fa;
                const float lim = 0.4f * std::fabs(Tax);  // autoblocante: manda par a la rueda que mas agarra
                const float dl = std::clamp(d_.bloqueo * (omega_[l] - omega_[r]), -lim, lim);
                Td[l] = Tax * 0.5f - dl;
                Td[r] = Tax * 0.5f + dl;
                if (libre_[l] != libre_[r]) {  // una rueda arrancada: todo el par del eje va a la que queda
                    const int fuera = libre_[l] ? l : r, queda = libre_[l] ? r : l;
                    Td[queda] += Td[fuera];
                    Td[fuera] = 0.0f;
                } else if (libre_[l]) {
                    Td[l] = Td[r] = 0.0f;  // sin las dos ruedas del eje el motor gira en vacio
                }
                if (!desliz) Iref[l] = Iref[r] = kIe * ratio * ratio * fa * 0.25f;  // inercia del motor vista desde la rueda
            }
        } else {  // punto muerto / cambiando: el motor sube y baja de vueltas libre
            rpm_ += ((motor_ ? kRpmRalenti + thr_ * 2500.0f : 0.0f) - rpm_) * std::min(1.0f, 8.0f * h);
        }
    }

    // --- 3) neumaticos y ruedas ---
    derrape_ = 0;
    float slipMax = 0;  // patinaje (en traccion) de las ruedas motrices en este subpaso
    const float tbF = tbMax_ * d_.repartoFreno * 0.5f, tbR = tbMax_ * (1.0f - d_.repartoFreno) * 0.5f;  // reparto delantero / trasero
    for (int i = 0; i < 4; ++i) {
        if (libre_[i]) { omega_[i] = 0.0f; continue; }  // rueda arrancada: no existe
        const float R = radio(i);
        const float wI = omega_[i];
        float fx = 0, k = 0, absF = 1.0f;
        if (!hit[i]) { kF_[i] = aF_[i] = 0.0f; }  // en el aire el neumatico pierde su deslizamiento
        if (hit[i]) {
            const float r[3] = {cp[i][0] - cg[0], cp[i][1] - cg[1], cp[i][2] - cg[2]};
            float wxr[3];
            cross(w, r, wxr);
            const float vc[3] = {v_[0] + wxr[0], v_[1] + wxr[1], v_[2] + wxr[2]};  // velocidad del punto de contacto

            // direccion de la rueda proyectada en el plano del suelo
            const float cd = std::cos(steer_[i]), sd = std::sin(steer_[i]);
            float wf[3];
            for (int q = 0; q < 3; ++q) wf[q] = cd * av_[q] + sd * der_[q];
            const float* n = nrm_[i];
            const float dn = dot(wf, n);
            float wfg[3] = {wf[0] - n[0] * dn, wf[1] - n[1] * dn, wf[2] - n[2] * dn};
            norm3(wfg);
            float wsg[3];
            cross(wfg, n, wsg);  // "derecha" de la rueda

            const float vlong = dot(vc, wfg), vlat = dot(vc, wsg);
            const float den = std::sqrt(vlong * vlong + 4.0f);  // suaviza el deslizamiento a baja velocidad
            const float kappa = (wI * R - vlong) / den;
            const float tanA = vlat / den;
            derrape_ = std::max(derrape_, std::fabs(tanA));
            if (frac_[i] > 0.0f && Td[i] > 0.0f) slipMax = std::max(slipMax, (wI * R - vlong) / std::max(std::fabs(vlong), 2.0f));

            // neumatico transitorio: el deslizamiento "llega" tras recorrer la longitud de relajacion (a baja velocidad, piso de 1.5 m/s)
            const float vr = std::max(std::fabs(vlong), 1.5f);
            const float xl = h * vr / kRelajLong, xt = h * vr / kRelajLat;
            const float al = xl / (1.0f + xl), at = xt / (1.0f + xt);
            kF_[i] += (kappa - kF_[i]) * al;
            aF_[i] += (tanA - aF_[i]) * at;

            // agarre: coeficiente del auto x carga x superficie x temperatura x lluvia
            const SuperficieInfo& su = superficieDeMaterial(mat_[i]);
            float mojado = 1.0f;
            if (humedad_ > 0.0f) {
                mojado -= humedad_ * (su.dura ? 0.28f : 0.10f);  // en suelo suelto la lluvia importa menos (ya agarra poco)
                if (su.dura) mojado *= 1.0f - 0.45f * humedad_ * std::clamp((std::fabs(vlong) - 17.0f) / 20.0f, 0.0f, 1.0f);  // aquaplaning
            }
            // sensibilidad a la carga: a mas carga, menos agarre por newton (la transferencia de peso cuesta agarre)
            const float mu = d_.agarre * std::clamp(1.0f - 0.12f * (fz_[i] / fz0_ - 1.0f), 0.6f, 1.3f) * su.agarre * agarreTemp(tempNeu_[i]) * mojado;
            float fxT, fyT, fx2, fy2;
            neumatico(mu, fz_[i], kF_[i], aF_[i], su.rigidez, fxT, fyT);
            neumatico(mu, fz_[i], kF_[i] + 0.02f, aF_[i], su.rigidez, fx2, fy2);
            k = std::max(0.0f, (fx2 - fxT) / 0.02f) * al * R / den;  // dFx/d(omega): para resolver la rueda de forma implicita
            const float maxFy = masa_ * 0.25f * std::fabs(vlat) / h;  // la fuerza lateral no puede invertir la velocidad
            fyT = std::clamp(fyT, -maxFy, maxFy);
            // rodadura: crece con el cuadrado de la velocidad y con lo blando que es el suelo
            const float vn = std::fabs(vlong) / 27.8f;
            const float rr = (kCrr0 + kCrrV * vn * vn) * su.rodadura * fz_[i] * std::tanh(vlong / 0.4f);
            fx = fxT;

            float Ft[3];
            for (int q = 0; q < 3; ++q) Ft[q] = (fxT - rr) * wfg[q] + fyT * wsg[q];
            aplicar(cp[i], Ft);

            // calentamiento del neumatico por el deslizamiento (longitudinal y lateral)
            const float pot = kCalorFric * (std::fabs(fxT * (wI * R - vlong)) + std::fabs(fyT * vlat)) + std::fabs(rr * vlong);  // deslizamiento + histeresis de la rodadura
            tempNeu_[i] += pot * h / kCalorNeu;

            if (d_.antibloqueo && cmd.freno > 0.05f && std::fabs(vlong) > 3.0f)
                absF = std::clamp(1.0f - (-kappa - 0.12f) / 0.10f, 0.0f, 1.0f);
        }

        float Tb = cmd.freno * (i < 2 ? tbF : tbR) * absF * eficaciaDisco(tempFreno_[i]);
        if (cmd.retener) Tb += 1200.0f;
        if (cmd.mano && i >= 2) Tb += 4000.0f;

        // ecuacion de la rueda (implicita en el neumatico y en el freno: estable con cualquier agarre)
        const float cb = Tb / std::max(std::fabs(wI), 0.2f);
        const float dw = (Td[i] - R * fx - cb * wI) / ((kIw + Iref[i]) / h + R * k + cb);
        omega_[i] = wI + dw;
        giroR_[i] = std::fmod(giroR_[i] + omega_[i] * h, 2.0f * kPi);

        // temperaturas: el disco se calienta con la potencia que frena y se enfria con el aire; el neumatico se enfria con el aire
        {
            const float vAire = std::sqrt(v_[0] * v_[0] + v_[1] * v_[1]);
            const float Cd = i < 2 ? kCalorDiscoDel : kCalorDiscoTras;
            tempFreno_[i] += (Tb * std::fabs(omega_[i]) - (tempFreno_[i] - kTempAmb) * (6.0f + 3.0f * vAire)) * h / Cd;
            tempNeu_[i] -= (tempNeu_[i] - kTempAmb) * (0.0012f + 0.00025f * vAire) * h;
            tempNeu_[i] = std::clamp(tempNeu_[i], kTempAmb - 10.0f, 200.0f);
            tempFreno_[i] = std::clamp(tempFreno_[i], kTempAmb - 10.0f, 900.0f);
        }
    }

    slipMotriz_ = slipMax;

    // --- 3b) hover: sustentacion, empuje y control de actitud ---
    hoverFuerzas(h, mundo, F, T, w);

    // --- 4) carroceria (volcado / tocar fondo) ---
    nCarr_ = 0;
    if (arr_[2] < 0.85f || bump_ || sinRuedas() || hLibre_) carroceria(mundo, w, F, T);  // sin ruedas se apoya en el chasis

    // --- 5) integracion ---
    for (int k = 0; k < 3; ++k) v_[k] += F[k] / masa_ * h;
    cg_.x += v_[0] * h;
    cg_.y += v_[1] * h;
    cg_.z += v_[2] * h;

    const float Tb3[3] = {dot(T, der_), dot(T, av_), dot(T, arr_)};  // par en ejes del chasis
    const float I3[3] = {Ip_, Ir_, Iy_};
    const float Iw3[3] = {I3[0] * wb_[0], I3[1] * wb_[1], I3[2] * wb_[2]};
    float giro[3];
    cross(wb_, Iw3, giro);  // termino giroscopico de las ecuaciones de Euler
    for (int k = 0; k < 3; ++k) wb_[k] = std::clamp(wb_[k] + (Tb3[k] - giro[k]) / I3[k] * h, -25.0f, 25.0f);

    float wn[3];
    for (int k = 0; k < 3; ++k) wn[k] = der_[k] * wb_[0] + av_[k] * wb_[1] + arr_[k] * wb_[2];
    const float wm = std::sqrt(dot(wn, wn));
    if (wm > 1e-6f) {
        const float ax[3] = {wn[0] / wm, wn[1] / wm, wn[2] / wm};
        const float ang = wm * h, c = std::cos(ang), s = std::sin(ang);
        rot(der_, ax, c, s);
        rot(av_, ax, c, s);
        rot(arr_, ax, c, s);
    }
    norm3(av_);  // reortonormaliza para que no se acumule el error
    const float dd = dot(der_, av_);
    for (int k = 0; k < 3; ++k) der_[k] -= av_[k] * dd;
    norm3(der_);
    cross(der_, av_, arr_);

    hacerOrigen();
    resolverParedes(mundo);
    hacerOrigen();
}

// ------------------------------------------------------------------------------------------------
void Vehiculo::actualizar(float dt, const EntradaVehiculo& in, const colision::Mundo& mundo) {
    dt = std::min(dt, 0.05f);
    if (dt <= 0) return;
    choques_.clear();
    hoverActualizar(dt, in, mundo);
    const bool hov = hEstado_ != HoverEstado::Suelo;  // con el hover activo gas / direccion / freno son los mandos del hover

    const float vf0 = dot(v_, av_);
    const float vabs = std::fabs(vf0);

    // --- pedales y marcha segun lo que pide el conductor ---
    const float gas = hov ? 0.0f : std::clamp(in.gas, -1.0f, 1.0f);
    float acel = 0, freno = 0;
    if (gas > 0.01f) {
        if (vf0 < -1.0f) freno = gas;                       // va hacia atras y pide avanzar: frena
        else { if (marcha_ <= 0) cambiar(1); acel = gas; }
    } else if (gas < -0.01f) {
        if (vf0 > 1.0f) freno = -gas;                       // va hacia delante y pide atras: frena
        else { if (marcha_ != -1) cambiar(-1); acel = -gas; }
    }
    if (hov && hLibre_) acel = std::max(std::fabs(hThr_), hBoost_);  // el hover consume: sube el regimen y el consumo del motor
    const bool retener = !hov && std::fabs(gas) <= 0.01f && !in.freno && vabs < 0.7f;
    thr_ += std::clamp((motor_ ? acel : 0.0f) - thr_, -8.0f * dt, 4.0f * dt);    // el pedal no es un interruptor (motor apagado: no responde)
    brk_ += std::clamp(freno - brk_, -12.0f * dt, 8.0f * dt);

    // --- caja automatica ---
    // Las revoluciones para decidir el cambio salen de la velocidad real del auto, no de las ruedas: una rueda que patina
    // marcaria 6000 rpm con el auto casi parado y la caja subiria y bajaria sin parar (cada cambio corta el par).
    tUltCambio_ += dt;
    if (tCambio_ > 0.0f) tCambio_ -= dt;
    if (marcha_ >= 1 && tCambio_ <= 0.0f && tUltCambio_ > 0.5f) {
        float rm = 0;
        for (int i = 0; i < 4; ++i) rm += frac_[i] * radio(i);  // radio medio de las ruedas motrices
        const float rpmVel = vabs / std::max(0.1f, rm) * relacion(marcha_) * 30.0f / kPi;
        const float rpmW = std::min(rpmRuedas(relacion(marcha_)), 1.15f * std::fabs(rpmVel) + 300.0f);
        const float subir = 3500.0f + 2400.0f * thr_;
        // en plena curva con poco gas no se sube de marcha (el auto queda en la marcha que va a necesitar al salir)
        const bool encurvado = giroLat_ > 0.45f && thr_ < 0.7f;
        if (marcha_ < 5 && rpmW > subir && !encurvado) {
            cambiar(marcha_ + 1);
        } else if (marcha_ > 1) {
            // solo se reduce si en la marcha de abajo no tocaria subir otra vez (histeresis de 700 rpm)
            const float rpmAbajo = rpmW * relacion(marcha_ - 1) / relacion(marcha_);
            const bool pide = rpmW < 1700.0f || (thr_ > 0.85f && rpmW < 2800.0f)   // reduccion y "kickdown"
                              || (brk_ > 0.35f && rpmW < 2600.0f)                  // frenando fuerte: baja para tener freno motor y salir de la curva
                              || (encurvado && rpmW < 2300.0f);
            if (pide && rpmAbajo < subir - 700.0f) cambiar(marcha_ - 1);
        }
    }

    // --- direccion: el angulo baja con la velocidad (limite de aceleracion lateral) y el volante tiene velocidad finita ---
    // (el coeficiente sube con la velocidad: a 30-50 km/h un angulo grande solo hace que las ruedas delanteras arrastren)
    const float kLat = 6.5f + 4.5f * std::clamp((vabs - 8.0f) / 20.0f, 0.0f, 1.0f);
    const float tope = std::clamp(1.6f * batalla_ * kLat / std::max(vabs * vabs, 1.0f), 0.03f, kMaxVolante);
    const float obj = hov ? 0.0f : std::clamp(in.direccion, -1.0f, 1.0f);
    const bool centrando = obj * dirNorm_ < 0.0f || std::fabs(obj) < std::fabs(dirNorm_);
    const float velVol = (centrando ? 4.5f : 2.8f) / (1.0f + vabs / 25.0f);
    dirNorm_ += std::clamp(obj - dirNorm_, -velVol * dt, velVol * dt);
    volante_ = dirNorm_ * tope;
    {   // Ackermann: la rueda interior gira mas que la exterior
        const float t = std::tan(volante_);
        steer_[0] = std::atan2(batalla_ * t, batalla_ + 0.5f * via_ * t);
        steer_[1] = std::atan2(batalla_ * t, batalla_ - 0.5f * via_ * t);
        steer_[2] = steer_[3] = 0.0f;
    }

    // aceleracion lateral (g) filtrada: velocidad lateral del chasis x velocidad de giro
    {
        const float aLat = std::fabs(vf0) * std::fabs(dot(wb_, wb_) > 0 ? wb_[2] : 0.0f) / kG;
        giroLat_ += (std::min(aLat, 2.0f) - giroLat_) * std::min(1.0f, 4.0f * dt);
    }

    // --- normal del suelo bajo cada rueda (una vez por fotograma) ---
    for (int i = 0; i < 4; ++i) {
        nrm_[i][0] = nrm_[i][1] = 0.0f;
        nrm_[i][2] = 1.0f;
        const float* o = d_.rueda[i];
        const float oz = o[2] + sRest_;
        const float mx = pos_.x + der_[0] * o[0] + av_[0] * o[1] + arr_[0] * oz;
        const float my = pos_.y + der_[1] * o[0] + av_[1] * o[1] + arr_[1] * oz;
        const float mz = pos_.z + der_[2] * o[0] + av_[2] * o[1] + arr_[2] * oz;
        float z0 = 0, z1 = 0, z2 = 0;
        if (!mundo.suelo(mx, my, mz + 0.25f, 4.0f, z0)) continue;
        if (!mundo.suelo(mx + 0.5f, my, z0 + 0.6f, 4.0f, z1)) continue;
        if (!mundo.suelo(mx, my + 0.5f, z0 + 0.6f, 4.0f, z2)) continue;
        const float gx = std::clamp((z1 - z0) / 0.5f, -1.2f, 1.2f), gy = std::clamp((z2 - z0) / 0.5f, -1.2f, 1.2f);
        const float l = std::sqrt(gx * gx + gy * gy + 1.0f);
        nrm_[i][0] = -gx / l;
        nrm_[i][1] = -gy / l;
        nrm_[i][2] = 1.0f / l;
    }

    // --- subpasos fijos ---
    Mando m;
    m.freno = brk_;
    m.mano = in.freno && !hov;
    m.retener = retener;
    const int n = std::max(1, static_cast<int>(std::ceil(dt / kPasoMax)));
    const float hs = dt / static_cast<float>(n);
    for (int s = 0; s < n; ++s) paso(hs, m, mundo);

    vf_ = dot(v_, av_);
    mecanica(dt, velocidadTotal());

    // --- auto-enderezado si queda de techo y parado ---
    if (!hov && arr_[2] < 0.25f && velocidadTotal() < 2.0f) volcado_ += dt; else volcado_ = 0.0f;
    if (volcado_ > 3.0f) { enderezar(); volcado_ = 0.0f; }

    // --- red de seguridad: si cae al vacio (o algo sale NaN) vuelve al ultimo sitio firme ---
    const bool nan = !(std::isfinite(cg_.x) && std::isfinite(cg_.y) && std::isfinite(cg_.z));
    if (!nan && enSuelo() && arr_[2] > 0.6f) {
        tSeguro_ += dt;
        if (tSeguro_ > 1.0f) { seguro_ = pos_; yawSeguro_ = yaw(); tSeguro_ = 0; }
    }
    if (nan || pos_.z < seguro_.z - 150.0f) colocar(seguro_.x, seguro_.y, seguro_.z, yawSeguro_);
}

// ------------------------------------------------------------------------------------------------
// Mecanica
const char* Vehiculo::motivoNoArranca() const {
    if (saludMotor_ <= 0.0f) return "motor fundido";
    if (combustible_ <= 0.0f) return "sin combustible";
    if (bateria_ * saludBat_ < 0.04f) return "bateria descargada";
    if (temp_ > 130.0f) return "motor recalentado";
    return nullptr;
}

void Vehiculo::alternarMotor() {
    if (motor_) { motor_ = false; arrancando_ = 0.0f; return; }
    if (arrancando_ > 0.0f) return;
    if (const char* m = motivoNoArranca()) { aviso_ = m; avisoT_ = 3.0f; return; }
    arrancando_ = kArranque;
}

void Vehiculo::golpear(float lx, float ly, float vn) {
    const float e = std::clamp((vn - 3.0f) / 20.0f, 0.0f, 1.0f);
    if (e <= 0.0f) return;
    if (hEstado_ != HoverEstado::Suelo && vn > 4.0f) {  // un golpe fuerte dana el propulsor mas cercano (y un poco los vecinos)
        int cerca = 0;
        float dmin = 1e9f;
        for (int i = 0; i < 4; ++i) {
            const float dx = d_.rueda[i][0] - lx, dy = d_.rueda[i][1] - ly;
            const float dd = dx * dx + dy * dy;
            if (dd < dmin) { dmin = dd; cerca = i; }
        }
        for (int i = 0; i < 4; ++i) padSalud_[i] = std::max(0.0f, padSalud_[i] - (i == cerca ? 1.0f : 0.15f) * e * ph_.umbralFallas * 1.2f);
    }
    const float dl = ly / std::max(0.5f, d_.medioLargo);
    auto dano = [e](float& s, float k) { s = std::max(0.0f, s - k * e); };
    if (dl > 0.45f) {  // morro: motor, radiador, bateria, carter
        dano(saludMotor_, 0.55f);
        dano(saludBat_, 0.50f);
        bateria_ = std::max(0.0f, bateria_ - 0.30f * e);
        if (vn > 8.0f) fugaRefri_ = std::min(0.5f, fugaRefri_ + 0.004f + 0.020f * e);
        if (vn > 11.0f) fugaAceite_ = std::min(0.5f, fugaAceite_ + 0.002f + 0.012f * e);
    } else if (dl < -0.45f) {  // cola: tanque y diferencial
        dano(saludTrans_, 0.35f);
        if (vn > 7.0f) fugaComb_ = std::min(2.0f, fugaComb_ + 0.03f + 0.25f * e);
    } else {  // costado
        dano(saludTrans_, 0.15f);
        if (vn > 10.0f && std::fabs(lx) > 0.5f * d_.medioAncho) fugaComb_ = std::min(2.0f, fugaComb_ + 0.02f + 0.12f * e);
    }
}

void Vehiculo::servicio(bool completo) {
    combustible_ = std::max(1.0f, d_.deposito);
    aceite_ = refri_ = bateria_ = 1.0f;
    if (completo) {
        for (float& s : padSalud_) s = 1.0f;
        saludMotor_ = saludTrans_ = saludBat_ = 1.0f;
        fugaComb_ = fugaAceite_ = fugaRefri_ = 0.0f;
        temp_ = std::min(temp_, 90.0f);
    }
}

void Vehiculo::reposo(float dt) {
    dt = std::min(dt, 0.25f);
    if (dt <= 0.0f) return;
    thr_ = 0.0f;
    rpm_ = motor_ ? kRpmRalenti : 0.0f;
    mecanica(dt, 0.0f);
}

const char* Vehiculo::aviso() const {
    if (arrancando_ > 0.0f) return "arrancando...";
    if (avisoT_ > 0.0f && aviso_[0]) return aviso_;
    if (!motor_) return "motor apagado (I: arrancar)";
    if (temp_ > 115.0f) return "SOBRECALENTAMIENTO";
    if (aceite_ < 0.15f) return "ACEITE BAJO";
    if (combustible_ < 0.1f * d_.deposito) return "reserva de combustible";
    if (refri_ < 0.2f) return "refrigerante bajo";
    if (bateria_ * saludBat_ < 0.15f) return "bateria baja";
    if (saludMotor_ < 0.3f) return "motor muy danado";
    if (saludTrans_ < 0.3f) return "transmision danada";
    if (hayFuga()) return "fuga";
    return "";
}

void Vehiculo::mecanica(float dt, float vel) {
    if (avisoT_ > 0.0f) avisoT_ -= dt;
    const float rpmN = std::clamp(rpm_ / kRpmMax, 0.0f, 1.0f);
    const float cargaMotor = motor_ ? std::clamp(thr_, 0.0f, 1.0f) * rpmN : 0.0f;

    // --- motor de arranque ---
    if (arrancando_ > 0.0f) {
        arrancando_ -= dt;
        bateria_ = std::max(0.0f, bateria_ - kArranqueBat * dt);
        if (bateria_ * saludBat_ < 0.04f) {  // se quedo sin carga arrancando
            arrancando_ = 0.0f;
            aviso_ = "bateria descargada";
            avisoT_ = 3.0f;
        } else if (arrancando_ <= 0.0f) {
            if (const char* m = motivoNoArranca()) { aviso_ = m; avisoT_ = 3.0f; }
            else { motor_ = true; rpm_ = kRpmRalenti; }
        }
    }

    // --- consumos y fugas ---
    if (motor_) {
        combustible_ -= kTiempoJuego * (kRalentiL + kPlenaL * cargaMotor + 0.0008f * rpmN) * dt;
        aceite_ -= kTiempoJuego * kAceiteGasto * (0.3f + rpmN) * dt;
    }
    combustible_ = std::max(0.0f, combustible_ - fugaComb_ * dt);
    aceite_ = std::max(0.0f, aceite_ - fugaAceite_ * dt);
    refri_ = std::max(0.0f, refri_ - fugaRefri_ * dt);
    if (temp_ > 118.0f) refri_ = std::max(0.0f, refri_ - 0.002f * dt);  // hierve y se pierde

    // --- temperatura: el calor sale por el radiador (necesita refrigerante y aire) ---
    const float carga = motor_ ? 0.2f + 0.8f * cargaMotor : 0.0f;
    const float enfr = (0.35f + 0.65f * std::min(1.0f, vel / 15.0f)) * (0.08f + 0.92f * std::clamp(refri_ / 0.5f, 0.0f, 1.0f));
    const float obj = std::min(300.0f, kAmbiente + 120.0f * carga / (1.2f * enfr));
    temp_ += (obj - temp_) * std::min(1.0f, dt / (motor_ ? 45.0f : 240.0f));
    const float sobre = std::clamp((temp_ - 110.0f) / 25.0f, 0.0f, 1.0f);

    // --- desgaste: sin aceite o recalentado el motor se funde ---
    if (motor_) {
        const float falta = std::clamp((0.30f - aceite_) / 0.30f, 0.0f, 1.0f);
        const float rate = kDesgaste * (0.2f + 1.5f * cargaMotor) * (1.0f + 40.0f * falta * falta) * (1.0f + 8.0f * sobre);
        saludMotor_ = std::max(0.0f, saludMotor_ - rate * dt);
    }

    // --- bateria: el alternador la carga con el motor en marcha ---
    if (motor_ && rpm_ > 900.0f) bateria_ = std::min(1.0f, bateria_ + kAlternador * saludBat_ * dt);

    // --- el motor se para solo ---
    if (motor_) {
        const char* m = nullptr;
        if (saludMotor_ <= 0.0f) m = aceite_ <= 0.0f ? "motor gripado" : "motor fundido";
        else if (combustible_ <= 0.0f) m = "sin combustible";
        else if (temp_ > 135.0f) m = "motor recalentado";
        if (m) { motor_ = false; aviso_ = m; avisoT_ = 4.0f; }
    }

    // --- efecto en la potencia ---
    const float aceiteF = 0.6f + 0.4f * std::clamp(aceite_ / 0.3f, 0.0f, 1.0f);
    pot_ = (0.35f + 0.65f * saludMotor_) * (1.0f - 0.5f * sobre) * aceiteF;
    trans_ = std::clamp(saludTrans_ / 0.5f, 0.0f, 1.0f);
}
