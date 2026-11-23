// Hover de Vehiculo: sustentacion por propulsores en las ruedas, efecto suelo, control de altura, actitud por empuje
// diferencial, empuje con potencia limitada, vectorizado lateral, calor, fallas y turbulencia.
//
// Puerto del modelo de hover.cs (CLEO) a fisica real en SI. Diferencias con el script, a proposito:
//  - El script suma velocidades a mano cada fotograma. Aqui todo son FUERZAS sobre el cuerpo rigido de 6 grados de
//    libertad: cada pad empuja en su rueda (con su propio efecto suelo), asi que el cabeceo / alabeo salen del reparto
//    de empuje, y un pad fallado inclina el vehiculo de verdad (no hay un "desbalance" inventado).
//  - La sustentacion va por el eje "arriba" del chasis: inclinado, el hover acelera de lado (viraje coordinado real).
//  - El control de altura es un regulador de velocidad vertical que reparte el empuje sin pasar el maximo de los pads.
// Unidades: m, s, kg, N. El script usa m/fotograma; 0.0004 por fotograma^2 = 1 m/s^2 (a 50 fps).
#include <algorithm>
#include <cmath>

#include "Vehiculo.h"

namespace {
constexpr float kG = 9.81f;
inline float dot(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline void cross(const float* a, const float* b, float* o) {
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}
inline float suave(float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}
constexpr float kPadSano = 0.35f;  // por debajo de esta salud un pad no cuenta como operativo
}  // namespace

// ------------------------------------------------------------------------------------------------
void PerfilHover::ajustarA(const DatosVehiculo& d, const DatosVehiculo& ref) {
    const float rMasa = std::max(0.2f, d.masa / std::max(1.0f, ref.masa));
    const float rAcel = std::max(0.2f, d.aceleracion / std::max(0.1f, ref.aceleracion));
    const float rVel = std::max(0.2f, d.velMax / std::max(1.0f, ref.velMax));
    const float rTrac = std::max(0.2f, d.agarre / std::max(0.1f, ref.agarre));
    const float m4 = std::pow(rMasa, 0.25f);
    const float k = rAcel / m4;                 // empuje = Thrust * rAccel / rMasa^0.25
    empujeTope *= k;
    atrasTope *= k;
    boostTope *= k;
    potencia *= k * rVel;                       // potencia a velocidad = empuje * rVelMax
    atrasPotencia *= k * rVel;
    boostPotencia *= k * rVel;
    eficiencia = std::clamp(eficiencia / m4, 0.6f, 1.5f);  // un camion pesado asciende lento o solo levita
    topeLevitacion *= rVel;                     // tope de levitacion = 15 m/s * rVelMax
    turbulencia /= std::sqrt(rMasa);            // turbulencia = 1 / sqrt(rMasa)
    agarreLateral *= rTrac;                     // agarre lateral = rTraccion * TractionMax
}

// ------------------------------------------------------------------------------------------------
int Vehiculo::padsSanos() const {
    int n = 0;
    for (int i = 0; i < 4; ++i)
        if (!sinRueda_[i] && padSalud_[i] > kPadSano) ++n;
    return n;
}

void Vehiculo::hoverReiniciarAltura() {
    hAuto_ = false;
    hVzCmd_ = 0;
    hZObj_ = cg_.z;
}

bool Vehiculo::alternarHover(const char** motivo) {
    auto no = [&](const char* m) { if (motivo) *motivo = m; return false; };
    if (hEstado_ == HoverEstado::Suelo) {
        if (padsSanos() < 2) return no("menos de 2 ruedas hover operativas");
        if (arr_[2] < 0.5f) return no("el vehiculo esta volcado");
        hoverModoCalcular();
        hEstado_ = HoverEstado::Desplegando;
        hAterrizando_ = false;
        hThr_ = hBoost_ = hCalor_ = 0;
        hSobre_ = false;
        hVzCmd_ = 0;
        hInt_[0] = hInt_[1] = 0;
        hAuto_ = hModo_ == HoverModo::Vuelo && hAlt_ < 0.8f;  // desde el suelo: sube solo a una altura segura
        hZObj_ = cg_.z + 1.6f;
        return true;
    }
    if (hEstado_ == HoverEstado::Plegando) {  // se arrepintio: vuelve a desplegar
        hEstado_ = HoverEstado::Desplegando;
        hAterrizando_ = false;
        return true;
    }
    // desplegando / activo: en el aire aterriza controlado, cerca del suelo pliega
    if (hModo_ == HoverModo::Vuelo && hAlt_ > 1.5f) {
        hAterrizando_ = !hAterrizando_;
        hAuto_ = false;
    } else {
        hEstado_ = HoverEstado::Plegando;
        hAterrizando_ = false;
    }
    return true;
}

// capacidad de sustentacion y modo (HzMode)
void Vehiculo::hoverModoCalcular() {
    float cap = 0;
    for (int i = 0; i < 4; ++i)
        if (!sinRueda_[i]) cap += padSalud_[i];
    cap *= ph_.padSustentacion * ph_.eficiencia;
    hCap_ = cap;
    const HoverModo antes = hModo_;
    if (cap >= 1.0f) hModo_ = HoverModo::Vuelo;
    else if (cap * ph_.efectoSueloMax >= 1.0f) hModo_ = HoverModo::Levitacion;  // solo se sostiene con efecto suelo
    else hModo_ = HoverModo::SinSustentacion;
    hReserva_ = std::clamp((cap - 1.0f) / 0.75f, 0.0f, 1.0f);  // (capacidad - 1) / 0.75
    if (hModoPrev_ != 3 && antes != hModo_ && hEstado_ != HoverEstado::Suelo) {
        aviso_ = hModo_ == HoverModo::Vuelo ? "ruedas hover operativas: vuelo habilitado"
               : hModo_ == HoverModo::Levitacion ? "pocas ruedas sanas: solo levita"
                                                 : "ruedas insuficientes: sin sustentacion";
        avisoT_ = 2.5f;
    }
    hModoPrev_ = static_cast<int>(hModo_);
}

// altura del pad i sobre el suelo y su posicion en el mundo
float Vehiculo::hoverCotaPad(int i, const colision::Mundo& mundo, float* p) const {
    const float* o = d_.rueda[i];
    p[0] = pos_.x + der_[0] * o[0] + av_[0] * o[1] + arr_[0] * o[2];
    p[1] = pos_.y + der_[1] * o[0] + av_[1] * o[1] + arr_[1] * o[2];
    p[2] = pos_.z + der_[2] * o[0] + av_[2] * o[1] + arr_[2] * o[2];
    float gz = 0;
    if (mundo.suelo(p[0], p[1], p[2] + 0.3f, 14.0f, gz)) return std::max(0.0f, p[2] - gz);
    return 14.0f;
}

namespace {
float gauss(unsigned& s) {  // aproximacion con 4 uniformes, sin <random>: determinista y barata
    float t = 0;
    for (int k = 0; k < 4; ++k) {
        s ^= s << 13; s ^= s >> 17; s ^= s << 5;
        t += static_cast<float>(s & 0xFFFFu) / 65535.0f;
    }
    return (t - 2.0f) * 1.732f;
}
}  // namespace

// ------------------------------------------------------------------------------------------------
// Una vez por fotograma: maquina de estados, modo, ordenes del piloto, calor y turbulencia.
void Vehiculo::hoverActualizar(float dt, const EntradaVehiculo& in, const colision::Mundo& mundo) {
    float p[3];
    hAlt_ = 99.0f;
    float hMed = 0;
    int n = 0;
    for (int i = 0; i < 4; ++i) {
        hCotas_[i] = sinRueda_[i] ? 14.0f : hoverCotaPad(i, mundo, p);
        if (!sinRueda_[i]) { hAlt_ = std::min(hAlt_, hCotas_[i]); hMed += hCotas_[i]; ++n; }
    }
    if (n == 0) hAlt_ = 0; else hMed /= static_cast<float>(n);
    hAltMed_ = hMed;

    if (hEstado_ == HoverEstado::Suelo) {
        hPose_ = hRampa_ = hBoost_ = hThr_ = 0;
        hLibre_ = false;
        hCalor_ = std::max(0.0f, hCalor_ - ph_.enfriamiento * dt);
        if (hCalor_ < 0.5f) hSobre_ = false;
        hoverModoCalcular();
        return;
    }
    hoverModoCalcular();

    // ---- animacion de las ruedas y rampa de sustentacion ----
    const float tr = std::max(0.1f, ph_.transicion);
    if (hEstado_ == HoverEstado::Desplegando) {
        hPose_ += dt / tr;
        if (hPose_ >= 1.0f) { hPose_ = 1.0f; hEstado_ = HoverEstado::Activo; }
    } else if (hEstado_ == HoverEstado::Plegando) {
        hPose_ -= dt / tr;
        if (hPose_ <= 0.0f) {
            hPose_ = hRampa_ = 0.0f;
            hEstado_ = HoverEstado::Suelo;
            hLibre_ = false;
            hBoost_ = hThr_ = 0;
            return;
        }
    }
    // al plegar, la sustentacion se mantiene hasta que las ruedas vuelven a apoyar
    hRampa_ = hEstado_ == HoverEstado::Plegando ? (hPose_ > 0.6f ? 1.0f : suave(hPose_ / 0.6f)) : suave(hPose_);
    hLibre_ = hPose_ > 0.6f;

    // sin sustentacion: cae amortiguado y apaga el hover al tocar el suelo
    if (hEstado_ == HoverEstado::Activo && hModo_ == HoverModo::SinSustentacion && hAlt_ < 1.2f) {
        aviso_ = "sin ruedas suficientes: hover apagado";
        avisoT_ = 2.5f;
        hEstado_ = HoverEstado::Plegando;
        hAterrizando_ = false;
    }

    // ---- altura: Alt / Ctrl, ascenso automatico, descenso rapido, aterrizaje ----
    float vzT = 0;
    if (hModo_ == HoverModo::Vuelo) {
        const bool alt = in.hAscenso != 0.0f;
        if (alt) { hAuto_ = false; hAterrizando_ = false; }
        if (in.hDescender) {
            vzT = -std::min(8.0f, 1.0f + 0.9f * hAlt_);  // frena la caida al llegar (flare)
            hAuto_ = false;
        } else if (hAterrizando_) {
            vzT = -std::min(4.0f, 0.8f + 0.7f * hAlt_);
            if (hAlt_ < 1.2f) { hEstado_ = HoverEstado::Plegando; hAterrizando_ = false; }
        } else if (alt) {
            vzT = std::clamp(in.hAscenso, -1.0f, 1.0f) * 3.0f * (in.hAscenso > 0 ? hReserva_ : 1.0f);  // con 3 pads sube mas lento
        } else if (hAuto_) {
            const float dz = hZObj_ - cg_.z;
            if (dz > 0.4f) vzT = std::clamp(dz * 1.0f, 0.4f, 2.0f);
            else hAuto_ = false;
        }
    } else {
        hAuto_ = false;
        hAterrizando_ = false;
    }
    hVzCmd_ += (vzT - hVzCmd_) * std::min(1.0f, 2.0f * dt);  // constante de tiempo ~0.5 s
    hVzObj_ = hVzCmd_;

    // ---- palanca de avance (W/S), propulsores (Espacio) y calor ----
    hThr_ += (std::clamp(in.gas, -1.0f, 1.0f) - hThr_) * std::min(1.0f, 2.0f * dt);
    const float sanos = static_cast<float>(padsSanos()) / 4.0f;
    const bool quiere = in.freno && hModo_ == HoverModo::Vuelo && !hSobre_ && hEstado_ == HoverEstado::Activo;
    hBoost_ += ((quiere ? 0.5f + 0.5f * sanos : 0.0f) - hBoost_) * std::min(1.0f, (quiere ? 1.5f : 3.0f) * dt);
    hCalor_ = std::clamp(hCalor_ + (hBoost_ > 0.05f ? ph_.calentamiento * hBoost_ : -ph_.enfriamiento) * dt, 0.0f, 1.0f);
    if (hCalor_ >= 1.0f && !hSobre_) { hSobre_ = true; aviso_ = "propulsores sobrecalentados"; avisoT_ = 2.5f; }
    if (hSobre_ && hCalor_ < 0.5f) hSobre_ = false;
    hEmpujeMot_ = 1.0f - 0.6f * ph_.danoMotor * (1.0f - saludMotor_);

    // ---- velocidades angulares pedidas ----
    float c[3] = {0, 0, 0};
    const bool vuelo = hModo_ == HoverModo::Vuelo;
    const float kNivel = 4.0f * ph_.autoNivel;
    // se nivela solo: en levitacion, con P, mientras despliega / pliega / esta pegado al suelo, y con algun pad fallado
    // (el piloto automatico compensa la falla; las flechas mandan mientras estan apretadas)
    const bool nivelar = hModo_ == HoverModo::Levitacion || in.hNivelar || hEstado_ != HoverEstado::Activo || hAlt_ < 0.6f ||
                         padsSanos() < 4;
    // cabeceo (+ = morro arriba): no pasa de ~58 grados
    if (vuelo && in.hCabeceo != 0.0f) {
        c[0] = in.hCabeceo * ph_.velCabeceo;
        if ((c[0] > 0 && av_[2] > 0.85f) || (c[0] < 0 && av_[2] < -0.85f)) c[0] = 0;
    } else if (nivelar) {
        c[0] = std::clamp(-kNivel * std::asin(std::clamp(av_[2], -1.0f, 1.0f)), -1.0f, 1.0f);
    }
    // alabeo (+ = lado derecho abajo)
    if (vuelo && in.hAlabeo != 0.0f) {
        c[1] = in.hAlabeo * ph_.velAlabeo;
        if ((c[1] > 0 && der_[2] < -0.85f) || (c[1] < 0 && der_[2] > 0.85f)) c[1] = 0;
    } else if (nivelar) {
        c[1] = std::clamp(-kNivel * alabeo(), -1.5f, 1.5f);  // boca abajo tambien gira de vuelta
    }
    // guinada tipo helicoptero: A izquierda (+), D derecha (-); mas el viraje coordinado de la inclinacion
    c[2] = -(std::clamp(in.direccion, -1.0f, 1.0f) + std::clamp(in.hGuinada, -1.0f, 1.0f)) * ph_.velGuinada;
    if (vuelo) c[2] += der_[2] * ph_.guinadaCoord;  // ala derecha abajo (der_.z < 0) -> gira a la derecha
    for (int k = 0; k < 3; ++k) hCmd_[k] = c[k];
    // integral del error de velocidad angular: sin ordenes ni nivelado el vehiculo MANTIENE su actitud (la integral de
    // (0 - w) es el angulo que se aleja de donde se solto la tecla). Con ordenes o nivelado se reinicia.
    for (int k = 0; k < 2; ++k) {
        if (c[k] != 0.0f || (nivelar && k <= 1)) hInt_[k] = 0.0f;
        else hInt_[k] = std::clamp(hInt_[k] + (c[k] - wb_[k]) * dt, -0.6f, 0.6f);
    }

    // ---- turbulencia: procesos de Ornstein-Uhlenbeck (rafagas suaves) ----
    const float sp = std::sqrt(v_[0] * v_[0] + v_[1] * v_[1] + v_[2] * v_[2]);
    const float sig = 0.45f * ph_.turbulencia * (0.3f + std::min(1.0f, sp / 30.0f));
    const float a = std::min(1.0f, dt / 1.2f);
    for (int k = 0; k < 3; ++k) {
        hTurb_[k] += -hTurb_[k] * a + sig * std::sqrt(2.0f * a) * gauss(hRng_);
        hTurbT_[k] += -hTurbT_[k] * a + 0.12f * ph_.turbulencia * (0.3f + std::min(1.0f, sp / 30.0f)) * std::sqrt(2.0f * a) * gauss(hRng_);
    }
}

// ------------------------------------------------------------------------------------------------
// En cada subpaso: fuerzas y pares del hover sobre el cuerpo rigido.
void Vehiculo::hoverFuerzas(float h, const colision::Mundo& mundo, float* F, float* T, const float* w) {
    if (hEstado_ == HoverEstado::Suelo || hRampa_ <= 0.0f) return;
    const float m = masa_;
    const float cg[3] = {cg_.x, cg_.y, cg_.z};
    const float vel = std::sqrt(dot(v_, v_));

    // ---- sustentacion maxima de cada pad (efecto suelo propio) ----
    float pp[4][3], lm[4], cota[4];
    float velF = 1.0f;
    if (ph_.velMinSust > 0.0f) velF = 0.4f + 0.6f * std::min(1.0f, vel / ph_.velMinSust);
    const float base = ph_.padSustentacion * ph_.eficiencia * m * kG * hRampa_ * velF;
    const float pendiente = (ph_.efectoSueloMax - 1.0f) / std::max(0.5f, ph_.efectoSueloAlt);
    float Fsum = 0;
    float hProm = 0;
    int nPad = 0;
    for (int i = 0; i < 4; ++i) {
        cota[i] = sinRueda_[i] ? 14.0f : hoverCotaPad(i, mundo, pp[i]);
        if (sinRueda_[i]) { lm[i] = 0; pp[i][0] = pp[i][1] = pp[i][2] = 0; continue; }
        const float ge = std::clamp(ph_.efectoSueloMax - pendiente * cota[i], 1.0f, ph_.efectoSueloMax);
        lm[i] = base * padSalud_[i] * ge;
        Fsum += lm[i];
        hProm += cota[i];
        ++nPad;
    }
    if (Fsum < 1.0f || nPad == 0) return;
    hProm /= static_cast<float>(nPad);

    // ---- empuje colectivo ----
    const float cosT = std::clamp(arr_[2], 0.3f, 1.0f);
    float Ftot = Fsum;
    const bool vuelo = hModo_ == HoverModo::Vuelo;
    if (vuelo) {
        // regulador de velocidad vertical: mantiene la altura (nunca mas que el maximo de los pads) y sigue Alt / Ctrl
        const float acmd = kG + 2.5f * (hVzObj_ - v_[2]);
        Ftot = std::clamp(m * acmd / cosT, 0.0f, Fsum);
    }
    // reparto: pesos por capacidad, desplazados para que la resultante pase por el centro de gravedad
    // (con un pad fallado el controlador reparte el empuje para no inclinar el vehiculo, mientras pueda)
    float x[4], y[4], wgt[4];
    for (int i = 0; i < 4; ++i) { x[i] = d_.rueda[i][0]; y[i] = d_.rueda[i][1] - d_.cgY; wgt[i] = lm[i] / Fsum; }
    if (vuelo) {
        float A = 0, B = 0, C = 0, D = 0, E = 0;
        for (int i = 0; i < 4; ++i) {
            A += wgt[i] * x[i] * x[i]; B += wgt[i] * x[i] * y[i]; C += wgt[i] * y[i] * y[i];
            D += wgt[i] * x[i]; E += wgt[i] * y[i];
        }
        const float det = A * C - B * B;
        if (std::fabs(det) > 1e-4f) {
            const float lx = (-D * C + E * B) / det, ly = (-E * A + D * B) / det;
            float s = 0;
            for (int i = 0; i < 4; ++i) { wgt[i] = std::max(0.0f, wgt[i] * (1.0f + lx * x[i] + ly * y[i])); s += wgt[i]; }
            if (s > 1e-4f) for (int i = 0; i < 4; ++i) wgt[i] /= s;
        }
    }
    float Fi[4];
    for (int i = 0; i < 4; ++i) Fi[i] = Ftot * wgt[i];

    // ---- actitud: cabeceo y alabeo por empuje diferencial de los pads ----
    // alfa = Kp (w_pedida - w) + Ki * integral  -> respuesta de segundo orden que mantiene el angulo.
    float tauReqP = 0, tauReqR = 0;
    float Sx2 = 0, Sy2 = 0;
    if (hModo_ != HoverModo::SinSustentacion) {
        const float kp0 = ph_.respuestaAng[0], kp1 = ph_.respuestaAng[1];
        const float alfaP = std::clamp(kp0 * (hCmd_[0] - wb_[0]) + 0.25f * kp0 * kp0 * hInt_[0], -30.0f, 30.0f);
        const float alfaR = std::clamp(kp1 * (hCmd_[1] - wb_[1]) + 0.25f * kp1 * kp1 * hInt_[1], -30.0f, 30.0f);
        for (int i = 0; i < 4; ++i) if (lm[i] > 0) { Sx2 += x[i] * x[i]; Sy2 += y[i] * y[i]; }
        tauReqP = Ip_ * alfaP;  // torque = inercia * aceleracion angular pedida
        tauReqR = Ir_ * alfaR;
        float logP = 0, logR = 0;  // lo que los pads entregan de verdad (saturan: 0 <= F <= max, autoridad limitada)
        for (int i = 0; i < 4; ++i) {
            if (lm[i] <= 0) continue;
            float d = 0;
            if (Sy2 > 1e-3f) d += tauReqP * y[i] / Sy2;     // pad delantero arriba -> morro arriba
            if (Sx2 > 1e-3f) d -= tauReqR * x[i] / Sx2;     // pad izquierdo arriba -> lado derecho abajo
            const float aut = ph_.autoridadPads * ph_.padSustentacion * m * kG * padSalud_[i];
            d = std::clamp(d, -aut, aut);
            const float nuevo = std::clamp(Fi[i] + d, 0.0f, lm[i]);
            logP += (nuevo - Fi[i]) * y[i];
            logR -= (nuevo - Fi[i]) * x[i];
            Fi[i] = nuevo;
        }
        tauReqP -= logP;  // el resto lo cubre el empuje vectorizado
        tauReqR -= logR;
    }

    // ---- aplicar la sustentacion en cada pad, a lo largo del eje arriba del chasis ----
    for (int i = 0; i < 4; ++i) {
        if (Fi[i] <= 0.0f) continue;
        const float f[3] = {Fi[i] * arr_[0], Fi[i] * arr_[1], Fi[i] * arr_[2]};
        const float r[3] = {pp[i][0] - cg[0], pp[i][1] - cg[1], pp[i][2] - cg[2]};
        float t[3];
        cross(r, f, t);
        for (int k = 0; k < 3; ++k) { F[k] += f[k]; T[k] += t[k]; }
    }

    // ---- guinada: empuje vectorizado (par puro, no sale de los pads) ----
    if (hModo_ != HoverModo::SinSustentacion) {
        const float alfaY = std::clamp(ph_.respuestaAng[2] * (hCmd_[2] - wb_[2]), -1.2f * ph_.parGuinada, 1.2f * ph_.parGuinada);
        const float tz = Iy_ * alfaY * hRampa_;
        for (int k = 0; k < 3; ++k) T[k] += arr_[k] * tz;
        // el par de cabeceo / alabeo que los pads no pudieron dar (pad fallado, 2 pads en diagonal, saturacion)
        const float tp = std::clamp(tauReqP, -Ip_ * ph_.autoridadVect, Ip_ * ph_.autoridadVect) * hRampa_;
        const float tr = std::clamp(tauReqR, -Ir_ * ph_.autoridadVect, Ir_ * ph_.autoridadVect) * hRampa_;
        for (int k = 0; k < 3; ++k) T[k] += der_[k] * tp + av_[k] * tr;
    }

    // ---- levitacion y caida amortiguada: sin control, la altura la fija el efecto suelo ----
    if (!vuelo) {
        const float vz = v_[2];
        if (hModo_ == HoverModo::Levitacion) {
            // amortiguacion vertical casi critica (baja con 'amortBajar', sube con 'amortSubir')
            F[2] -= m * (vz < 0.0f ? ph_.amortBajar : ph_.amortSubir) * vz;
            // el vehiculo se mantiene entre 2 y 4 m
            if (hProm > 4.0f) {
                F[2] += m * std::clamp(-50.0f * (hProm - 4.0f), -10.0f, 0.0f);
                if (vz > 0.0f) F[2] -= m * 20.0f * vz;
            } else if (hProm < 2.0f) {
                F[2] += m * std::clamp(50.0f * (2.0f - hProm), 0.0f, 10.0f);
                if (vz < 0.0f) F[2] -= m * 20.0f * vz;
            }
        }
    }

    // ---- empuje horizontal a lo largo de la nariz: potencia limitada, a = min(P / V, tope) ----
    {
        const float V = std::max(1.0f, vel);
        const float sanos = 0.5f + 0.5f * static_cast<float>(padsSanos()) / 4.0f;  // ruedas sanas 0.5..1
        float a;
        if (hThr_ >= 0.0f) a = std::min(ph_.potencia / V, ph_.empujeTope) * hThr_;
        else a = std::min(ph_.atrasPotencia / V, ph_.atrasTope) * hThr_;
        a *= sanos;
        if (!vuelo) {  // levitacion: motor al 50 % y se apaga al acercarse al tope de velocidad
            a *= 0.5f * std::clamp(1.0f - V / std::max(1.0f, ph_.topeLevitacion), 0.0f, 1.0f);
        }
        if (hBoost_ > 0.0f) a += std::min(ph_.boostPotencia / V, ph_.boostTope) * hBoost_;
        a *= hEmpujeMot_ * hRampa_;
        for (int k = 0; k < 3; ++k) F[k] += av_[k] * m * a;
    }

    // ---- arrastre: a = -k |v| v, mas alto de costado que de frente ----
    {
        const float s = dot(v_, av_);
        const float f = vel > 1e-3f ? std::min(1.0f, s * s / (vel * vel)) : 1.0f;
        const float k = ph_.arrastreLateral + (ph_.arrastreFrontal - ph_.arrastreLateral) * f;
        const float kd = std::min(k * vel, 0.5f / h);  // estable aunque el subpaso sea grande
        for (int i = 0; i < 3; ++i) F[i] -= m * kd * v_[i];
        const float kv = std::min(ph_.arrastreVertical * std::fabs(v_[2]), 0.5f / h);
        F[2] -= m * kv * v_[2];
    }

    // ---- vectorizado lateral: agarre limitado, radio de giro minimo = V^2 / a ----
    {
        float fh[2] = {av_[0], av_[1]};
        const float l = std::sqrt(fh[0] * fh[0] + fh[1] * fh[1]);
        if (l > 0.2f) {
            fh[0] /= l; fh[1] /= l;
            const float vl = v_[0] * fh[0] + v_[1] * fh[1];
            const float lat[2] = {v_[0] - vl * fh[0], v_[1] - vl * fh[1]};
            const float mag = std::sqrt(lat[0] * lat[0] + lat[1] * lat[1]);
            if (mag > 1e-4f) {
                const float am = std::min(ph_.gananciaLateral * mag, ph_.agarreLateral) * hRampa_;
                F[0] -= m * am * lat[0] / mag;
                F[1] -= m * am * lat[1] / mag;
            }
        }
    }

    // ---- tope de velocidad en levitacion ----
    if (hModo_ == HoverModo::Levitacion) {
        const float vh = std::sqrt(v_[0] * v_[0] + v_[1] * v_[1]);
        if (vh > ph_.topeLevitacion) {
            const float k = 3.0f * (vh - ph_.topeLevitacion) / vh;
            F[0] -= m * k * v_[0];
            F[1] -= m * k * v_[1];
        }
    }

    // ---- turbulencia ----
    {
        const float e = hRampa_;
        for (int k = 0; k < 3; ++k) F[k] += m * e * (hTurb_[0] * der_[k] + hTurb_[1] * av_[k] + hTurb_[2] * arr_[k]);
        for (int k = 0; k < 3; ++k) T[k] += e * (Ip_ * hTurbT_[0] * der_[k] + Ir_ * hTurbT_[1] * av_[k] + Iy_ * hTurbT_[2] * arr_[k]);
    }
    (void)w;
}
