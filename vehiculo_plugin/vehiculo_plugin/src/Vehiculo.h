#pragma once
// Fisica de auto estilo simulador. Sin SDL ni OpenGL: recibe la entrada ya interpretada y consulta colision::Mundo.
//
//  - Cuerpo rigido de 6 grados de libertad (posicion, velocidad, orientacion y velocidad angular, con tensor de inercia).
//  - Suspension independiente por rueda (muelle + amortiguador bump/rebote + tope + barra estabilizadora).
//  - Neumaticos con deslizamiento longitudinal y angulo de deriva combinados (curva tipo "Magic Formula"),
//    sensibilidad a la carga vertical y rueda con inercia propia (puede patinar, bloquearse y derrapar).
//  - Transmision: curva de par del motor, embrague, caja automatica de 5 marchas + marcha atras, diferencial
//    autoblocante, traccion trasera / delantera / total, frenos con reparto, ABS y freno de mano.
//  - Aerodinamica (resistencia y carga), resistencia a la rodadura que crece con la velocidad, transferencia de peso real.
//  - Superficies: el material del .col bajo cada rueda (asfalto, grava, hierba, barro, arena, roca...) cambia el agarre,
//    la rigidez del neumatico (en suelo suelto hay que deslizar mas para tener fuerza) y la resistencia a la rodadura.
//    Con lluvia (ponerHumedad) el agarre baja, mas en asfalto, y a alta velocidad aparece el aquaplaning.
//  - Neumaticos transitorios: la fuerza no aparece de golpe, el deslizamiento se filtra con una longitud de relajacion
//    (el auto "tarda" un poco en responder al volante); temperatura del neumatico: frio agarra menos, sobrecalentado tambien.
//  - Frenos con temperatura por disco: frios muerden menos, muy calientes pierden eficacia (fade); reparto delantero / trasero.
//  - Subpasos fijos (<= 1/200 s) para que sea estable con cualquier framerate.
//  - Mecanica: motor que se enciende / apaga, deposito de combustible, aceite, refrigerante con temperatura, bateria con
//    alternador, salud del motor / transmision / bateria y fugas por golpes. Todo afecta a la potencia real.
//  - Sin ruedas el auto sigue siendo un auto: se apoya y se arrastra sobre el chasis (carroceria()), conserva su
//    mecanica y su fisica (aunque sin ruedas no pueda traccionar).
//
// Convenciones de GTA SA: el modelo mira hacia +Y, +X es la derecha y +Z arriba. El rumbo (yaw) es el angulo del
// avance en el mundo: yaw = 0 avanza hacia +X (igual que Jugador). Unidades: metros, segundos, kg, N.
#include <algorithm>
#include <vector>

#include "Colision.h"

struct EntradaVehiculo {
    float gas = 0;        // -1 (atras / freno) .. 1 (adelante). Si va hacia delante y se pide -1 frena; parado, mete marcha atras.
    float direccion = 0;  // -1 (izquierda) .. 1 (derecha)
    bool freno = false;   // freno de mano: bloquea las ruedas traseras (en hover: propulsores)

    // --- solo con el hover activo (el resto de campos se interpreta asi: gas = palanca de avance, direccion = guinada) ---
    float hCabeceo = 0;   // -1 morro abajo .. 1 morro arriba
    float hAlabeo = 0;    // -1 izquierda .. 1 derecha (inclina y vira)
    float hGuinada = 0;   // Q/E: -1 izquierda .. 1 derecha (se suma a 'direccion')
    float hAscenso = 0;   // Alt/Ctrl: -1 baja .. 1 sube
    bool hNivelar = false;     // P: autonivelado
    bool hDescender = false;   // descenso rapido con frenado de la altura
};

struct DatosVehiculo;

// Propiedades de una superficie (material de GTA .col). Todos los factores son relativos al asfalto seco (= 1).
struct SuperficieInfo {
    const char* nombre;
    float agarre;     // multiplica el coeficiente de friccion del neumatico
    float rodadura;   // multiplica la resistencia a la rodadura (suelo blando = el neumatico se hunde)
    float rigidez;    // 1 = asfalto; menos = hace falta mas deslizamiento para llegar al pico de fuerza (se siente "suelto")
    bool dura;        // asfalto, concreto, roca...: se moja y puede hacer aquaplaning
};
// Busca las propiedades de un material de GTA SA (0 = por defecto = asfalto). Siempre devuelve algo valido.
const SuperficieInfo& superficieDeMaterial(unsigned char material);

// Perfil de hover en unidades del SI. Los valores por defecto son los de handling.meta del DeLorean (DMC12_FLYING) del
// script hover.cs traducidos de "por fotograma" a m/s (ver comentario de cada campo). Se puede cambiar por vehiculo.
struct PerfilHover {
    float padSustentacion = 0.4375f;  // fraccion del peso que sostiene cada propulsor sano (PAD_LIFT)
    float eficiencia = 1.0f;          // Lift/50 del perfil: multiplica la sustentacion de cada pad
    float velMinSust = 0.0f;          // m/s para sustentar a pleno (0 = sustenta parado)
    float efectoSueloMax = 1.5f;      // sustentacion x1.5 pegado al suelo...
    float efectoSueloAlt = 4.0f;      // ...que baja a x1.0 a esta altura (m)
    // empuje horizontal con potencia limitada: a = min(potencia / V, tope)  (m/s^2, V en m/s con minimo 1)
    float empujeTope = 2.5f, potencia = 14.0f;
    float atrasTope = 1.5f, atrasPotencia = 10.0f;
    float boostTope = 6.0f, boostPotencia = 120.0f;
    float arrastreFrontal = 0.00056f, arrastreLateral = 0.0028f, arrastreVertical = 0.0021f;  // a = -k |v| v  (1/m)
    float agarreLateral = 3.0f;       // m/s^2 maximos de vectorizado lateral
    float gananciaLateral = 2.5f;     // 1/s
    float respuestaAng[3] = {3.0f, 2.0f, 1.2f};  // 1/s: cabeceo, alabeo, guinada (AngDamp)
    float velCabeceo = 0.45f, velAlabeo = 0.60f, velGuinada = 0.80f;  // rad/s con la tecla a fondo
    float guinadaCoord = 0.60f;       // rad/s de guinada por unidad de inclinacion lateral (viraje coordinado)
    float autoNivel = 0.5f;           // AntiRoll: ganancia del autonivelado
    float autoridadPads = 0.35f;      // fraccion del empuje de un pad que se puede modular para cabecear / alabear
    float autoridadVect = 1.5f;       // rad/s^2 de cabeceo / alabeo que puede dar el empuje vectorizado cuando los pads no alcanzan
    float parGuinada = 1.0f;          // 1 = 1.2 rad/s^2 de guinada maxima
    float transicion = 1.0f;          // s de la animacion de desplegar / plegar las ruedas
    float inclinacionMax = 90.0f;     // grados que giran las ruedas al desplegar
    float amortSubir = 3.0f, amortBajar = 3.0f;   // 1/s: amortiguacion vertical en levitacion (casi critica)
    float topeLevitacion = 15.0f;     // m/s de velocidad horizontal maxima en levitacion
    float turbulencia = 1.0f;         // 0 = aire en calma, 1 = normal
    float danoMotor = 0.0f;           // 0 = el dano del motor no resta empuje; 1 = pierde hasta 60 %
    float umbralFallas = 0.75f;       // los pads empiezan a fallar con golpes por debajo de esto (salud de la carroceria)
    float calentamiento = 0.12f, enfriamiento = 0.08f;  // 1/s de calor de los propulsores (Espacio)

    // Ajusta el perfil a un vehiculo midiendolo contra el patron (DatosVehiculo por defecto, o sea el DeLorean).
    // Mas masa = menos empuje y menos eficiencia de pads (masa^0.25), mas turbulencia sensible (1/sqrt(masa)).
    void ajustarA(const DatosVehiculo& d, const DatosVehiculo& ref);
};

struct DatosVehiculo {
    float medioLargo = 2.3f, medioAncho = 0.9f;
    // Centro de cada rueda en el modelo, en este orden: delantera izq., delantera der., trasera izq., trasera der.
    float rueda[4][3] = {{-0.85f, 1.4f, -0.3f}, {0.85f, 1.4f, -0.3f}, {-0.85f, -1.4f, -0.3f}, {0.85f, -1.4f, -0.3f}};
    float radioRueda = 0.35f, radioRuedaTras = 0.35f;  // las ruedas del modelo miden 1 m de diametro: se escalan
    float velMax = 38.0f;       // m/s (~137 km/h): velocidad a la que la ultima marcha llega al corte de motor
    float aceleracion = 9.0f;   // m/s^2 de referencia en 1a: de ahi sale el par del motor (el agarre manda despues)

    // --- parametros de simulador (todos opcionales) ---
    float masa = 1400.0f;       // kg
    float cgY = 0.08f;          // centro de gravedad respecto al origen del modelo (+Y = hacia delante)
    float cgZ = -0.10f;         // (+Z = arriba). Mas bajo = mas estable
    float agarre = 1.05f;       // coeficiente de friccion maximo de los neumaticos (asfalto seco ~1.0 .. 1.2)
    int traccion = 0;           // 0 = trasera, 1 = delantera, 2 = total
    float frenos = 1.2f;        // capacidad de frenada en g
    float repartoFreno = 0.65f; // fraccion de la frenada que hacen las ruedas delanteras (0.5 .. 0.8)
    float bloqueo = 40.0f;      // autoblocante: Nm por rad/s de diferencia entre ruedas del mismo eje (0 = diferencial abierto)
    bool antibloqueo = true;    // ABS en el freno de servicio
    bool tcs = true;            // control de traccion: recorta el par cuando las ruedas motrices patinan de mas
    float frecSusp = 1.4f;      // Hz de la suspension (1.0 blanda .. 2.0 dura)
    float deposito = 55.0f;     // litros de combustible
};

// Golpe de la carroceria contra una pared u objeto del mundo (se rellena en actualizar()).
struct Choque {
    colision::V3 punto;        // donde toco (mundo)
    colision::V3 normal;       // hacia fuera de la superficie golpeada (horizontal)
    float velNormal = 0;       // m/s con que se acercaba a la superficie
    int instancia = -1;        // instancia de colision::Mundo (-1 si no se sabe)
};

class Vehiculo {
public:
    Vehiculo() {
        configurar(DatosVehiculo{});
        colocar(0, 0, 0, 0);
    }

    void configurar(const DatosVehiculo& d);
    void colocar(float x, float y, float z, float yawRad);
    void actualizar(float dt, const EntradaVehiculo& in, const colision::Mundo& mundo);

    colision::V3 pos() const { return pos_; }  // origen del modelo (no los pies)
    float yaw() const;
    float cabeceo() const;
    float alabeo() const;                      // > 0: el lado derecho baja
    float velocidad() const { return vf_; }    // m/s con signo (negativa = marcha atras)
    float velocidadTotal() const;              // m/s en horizontal
    bool enSuelo() const { return nContactos_ >= 2 || nCarr_ >= 2; }
    const DatosVehiculo& datos() const { return d_; }
    float masa() const { return masa_; }
    void velocidadMundo(float* v3) const { v3[0] = v_[0]; v3[1] = v_[1]; v3[2] = v_[2]; }
    float velocidadMundoZ() const { return v_[2]; }
    void velAngularMundo(float* w3) const {
        for (int k = 0; k < 3; ++k) w3[k] = der_[k] * wb_[0] + av_[k] * wb_[1] + arr_[k] * wb_[2];
    }
    // Importa la velocidad del juego (mundo): v3 = centro de gravedad en m/s, w3 = velocidad angular en rad/s.
    // Tambien pone las ruedas a rodar a esa velocidad para que no haya tiron al engancharse.
    void ponerVelocidadMundo(const float* v3, const float* w3) {
        for (int k = 0; k < 3; ++k) v_[k] = v3[k];
        wb_[0] = der_[0] * w3[0] + der_[1] * w3[1] + der_[2] * w3[2];
        wb_[1] = av_[0] * w3[0] + av_[1] * w3[1] + av_[2] * w3[2];
        wb_[2] = arr_[0] * w3[0] + arr_[1] * w3[1] + arr_[2] * w3[2];
        vf_ = av_[0] * v_[0] + av_[1] * v_[1] + av_[2] * v_[2];
        for (int i = 0; i < 4; ++i) omega_[i] = vf_ / std::max(0.05f, radio(i));
    }
    // Choques del ultimo actualizar() contra el mundo (con la velocidad con que se acercaba a cada superficie).
    const std::vector<Choque>& choques() const { return choques_; }
    // Una rueda arrancada deja de apoyar, de empujar y de frenar: el auto se arrastra sobre la carroceria.
    void quitarRueda(int i) { if (i >= 0 && i < 4) { sinRueda_[i] = true; omega_[i] = 0; } }
    bool sinRueda(int i) const { return i >= 0 && i < 4 && sinRueda_[i]; }
    bool sinRuedas() const { return sinRueda_[0] || sinRueda_[1] || sinRueda_[2] || sinRueda_[3]; }

    // --- mecanica ---
    void alternarMotor();                      // llave: arranca (con bateria y combustible) o apaga
    bool motorEncendido() const { return motor_; }
    // Golpe en la carroceria (lx = derecha, ly = avance, respecto al origen del modelo; vn = m/s): dana el motor, el
    // radiador, la bateria, la transmision o el tanque segun la zona, y puede abrir fugas.
    void golpear(float lx, float ly, float vn);
    // Rellena combustible, aceite, refrigerante y bateria. 'completo' tambien repara la mecanica y tapa las fugas.
    void servicio(bool completo);
    // Mecanica de un auto que nadie conduce (motor al ralenti si sigue encendido, fugas, enfriamiento).
    void reposo(float dt);
    float combustible() const { return combustible_; }          // litros
    float aceite() const { return aceite_; }                    // nivel 0..1
    float refrigerante() const { return refri_; }               // nivel 0..1
    float bateria() const { return bateria_; }                  // carga 0..1
    float tempMotor() const { return temp_; }                   // grados C
    float saludMotor() const { return saludMotor_; }            // 0..1 (0 = fundido)
    float saludTransmision() const { return saludTrans_; }      // 0..1 (0 = no transmite)
    float saludBateria() const { return saludBat_; }            // 0..1
    bool hayFuga() const { return fugaComb_ > 0 || fugaAceite_ > 0 || fugaRefri_ > 0; }
    const char* aviso() const;                                  // el aviso mas importante del tablero ("" si no hay)

    // Telemetria (para un HUD o para depurar)
    int marcha() const { return marcha_; }     // -1 atras, 0 punto muerto, 1..5
    float rpm() const { return rpm_; }
    float rpmMaximas() const;
    float acelerador() const { return thr_; }  // 0..1 suavizado
    float derrape() const { return derrape_; } // tangente maxima del angulo de deriva de las ruedas (0.14 ~ limite de agarre)
    float cargaRueda(int i) const { return fz_[i]; }  // N sobre cada neumatico
    bool tcsActivo() const { return tcsF_ < 0.95f; }  // el control de traccion esta recortando el par ahora mismo

    // --- superficie, clima, neumaticos y frenos (telemetria) ---
    void ponerHumedad(float h) { humedad_ = std::clamp(h, 0.0f, 1.0f); }   // 0 = seco, 1 = lluvia fuerte
    float humedad() const { return humedad_; }
    const SuperficieInfo& superficie(int i) const;                          // superficie bajo la rueda i (la ultima que toco)
    float tempNeumatico(int i) const { return (i >= 0 && i < 4) ? tempNeu_[i] : 0.0f; }  // grados C
    float tempFreno(int i) const { return (i >= 0 && i < 4) ? tempFreno_[i] : 0.0f; }    // grados C del disco
    float eficaciaFreno(int i) const;                                       // 0..1 lo que muerden las pastillas a esa temperatura
    float agarreNeumatico(int i) const;                                     // factor de agarre por temperatura (1 = temperatura ideal)
    float deslizamientoRueda(int i) const { return (i >= 0 && i < 4) ? kF_[i] : 0.0f; }  // deslizamiento longitudinal (filtrado)
    void ponerTcs(bool on) { d_.tcs = on; }
    void alternarTcs() { d_.tcs = !d_.tcs; }

    // --- hover (ver VehiculoHover.cpp) ---
    // Cada rueda es un propulsor de sustentacion. capacidad = suma de la salud de los pads * padSustentacion * eficiencia:
    //   >= 1.0 -> VUELO | capacidad * efecto suelo maximo >= 1.0 -> LEVITACION (se estabiliza solo, motor al 50 %) | menos -> sin sustentacion
    enum class HoverEstado { Suelo = 0, Desplegando = 1, Activo = 2, Plegando = 3 };
    enum class HoverModo { SinSustentacion = 0, Levitacion = 1, Vuelo = 2 };
    // H / J: alterna. Devuelve false (y 'motivo' dice por que) si no se puede (menos de 2 pads, volcado...).
    bool alternarHover(const char** motivo = nullptr);
    PerfilHover& perfilHover() { return ph_; }
    const PerfilHover& perfilHover() const { return ph_; }
    bool hoverActivo() const { return hEstado_ != HoverEstado::Suelo; }
    HoverEstado hoverEstado() const { return hEstado_; }
    HoverModo hoverModo() const { return hModo_; }
    float hoverCapacidad() const { return hCap_; }        // fraccion del peso que pueden sostener los pads (sin efecto suelo)
    float hoverReserva() const { return hReserva_; }      // 0..1 velocidad de ascenso disponible
    float hoverAltura() const { return hAlt_; }           // m sobre el suelo del pad mas bajo
    float hoverCalor() const { return hCalor_; }          // 0..1 propulsores
    bool hoverSobrecalentado() const { return hSobre_; }
    float hoverInclinacionRuedas() const { return hPose_; }  // 0 ruedas abajo .. 1 ruedas en posicion de hover
    float hoverEmpuje() const { return hThr_; }           // -1..1 palanca de avance (ya suavizada)
    float hoverBoost() const { return hBoost_; }          // 0..1 rampa de los propulsores
    float padSalud(int i) const { return (i >= 0 && i < 4 && !sinRueda_[i]) ? padSalud_[i] : 0.0f; }
    void danarPad(int i, float cantidad) { if (i >= 0 && i < 4) padSalud_[i] = std::max(0.0f, padSalud_[i] - cantidad); }
    int padsSanos() const;                                // pads con mas de 35 % de salud
    void hoverReiniciarAltura();                          // tras mover el auto (script CMD 2)

    // Matriz 4x4 por columnas (derecha, avance, arriba, posicion) del chasis y de cada rueda (0..3).
    void matriz(float* m16) const;
    void matrizRueda(int i, float* m16) const;

private:
    struct Mando {
        float freno = 0;       // freno de servicio 0..1 (ya suavizado)
        bool mano = false;     // freno de mano
        bool retener = false;  // sujeta el auto parado
    };

    void ejes(float* der, float* av, float* arr) const;
    void paso(float h, const Mando& m, const colision::Mundo& mundo);
    void resolverParedes(const colision::Mundo& mundo);
    void carroceria(const colision::Mundo& mundo, const float* w, float* F, float* T);
    void cambiar(int g);
    float relacion(int g) const;               // relacion total (caja * final), con signo
    float rpmRuedas(float ratio) const;
    float radio(int i) const { return i < 2 ? d_.radioRueda : d_.radioRuedaTras; }
    void desdeOrigen();                        // recalcula el centro de gravedad a partir de pos_
    void hacerOrigen();                        // recalcula pos_ a partir del centro de gravedad
    void enderezar();
    void hoverActualizar(float dt, const EntradaVehiculo& in, const colision::Mundo& mundo);  // una vez por fotograma
    void hoverFuerzas(float h, const colision::Mundo& mundo, float* F, float* T, const float* w);  // en cada subpaso
    void hoverModoCalcular();
    float hoverCotaPad(int i, const colision::Mundo& mundo, float* pos) const;  // altura del pad i sobre el suelo
    void mecanica(float dt, float vel);        // consumos, temperatura, desgaste, bateria (una vez por fotograma)
    const char* motivoNoArranca() const;       // nullptr si se puede arrancar

    DatosVehiculo d_;
    colision::V3 pos_, cg_, seguro_;
    float yawSeguro_ = 0;

    // Estado del cuerpo rigido
    float der_[3] = {0, -1, 0}, av_[3] = {1, 0, 0}, arr_[3] = {0, 0, 1};  // ejes del chasis en el mundo
    float v_[3] = {0, 0, 0};    // velocidad del centro de gravedad (mundo)
    float wb_[3] = {0, 0, 0};   // velocidad angular en ejes del chasis: (derecha=cabeceo, avance=alabeo, arriba=guinada)

    // Estado de las ruedas
    float omega_[4] = {0, 0, 0, 0};   // rad/s, positivo = rueda hacia delante
    float giroR_[4] = {0, 0, 0, 0};   // angulo acumulado para dibujar
    float steer_[4] = {0, 0, 0, 0};   // angulo de direccion por rueda (Ackermann), > 0 = derecha
    float sLen_[4] = {0, 0, 0, 0};    // longitud de suspension (anclaje -> centro de rueda)
    float xPrev_[4] = {0, 0, 0, 0};
    bool hitPrev_[4] = {false, false, false, false};
    float nrm_[4][3] = {{0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}};
    float fz_[4] = {0, 0, 0, 0};
    float kF_[4] = {0, 0, 0, 0}, aF_[4] = {0, 0, 0, 0};          // deslizamiento longitudinal y deriva filtrados (relajacion del neumatico)
    float tempNeu_[4] = {70, 70, 70, 70};                         // grados C de cada neumatico
    float tempFreno_[4] = {25, 25, 25, 25};                       // grados C de cada disco
    unsigned char mat_[4] = {0, 0, 0, 0};                         // material bajo cada rueda
    float humedad_ = 0;

    // Mando y motor
    float thr_ = 0, brk_ = 0, dirNorm_ = 0, volante_ = 0;
    float rpm_ = 900, tCambio_ = 0;
    float giroLat_ = 0;                    // aceleracion lateral filtrada en g (la caja no sube de marcha en plena curva)
    float tUltCambio_ = 0;                 // s desde el ultimo cambio de marcha (evita que la caja oscile)
    float slipMotriz_ = 0, tcsF_ = 1;      // deslizamiento maximo de las ruedas motrices (subpaso anterior) y factor de par del TCS
    int marcha_ = 1;

    std::vector<Choque> choques_;
    bool sinRueda_[4] = {false, false, false, false};

    // Mecanica
    bool motor_ = false;
    float arrancando_ = 0;                            // s que faltan de motor de arranque
    float combustible_ = 55, aceite_ = 1, refri_ = 1, bateria_ = 1, temp_ = 25;
    float saludMotor_ = 1, saludTrans_ = 1, saludBat_ = 1;
    float fugaComb_ = 0, fugaAceite_ = 0, fugaRefri_ = 0;  // por segundo (litros / nivel)
    float pot_ = 1, trans_ = 1;                       // factores de potencia y de transmision (salen de la salud)
    const char* aviso_ = "";
    float avisoT_ = 0;
    int nCarr_ = 0;                                   // puntos de la carroceria apoyados en el suelo

    // Varios
    float vf_ = 0, derrape_ = 0, volcado_ = 0, tSeguro_ = 0;
    int nContactos_ = 0;
    bool bump_ = false;

    // Hover
    PerfilHover ph_;
    HoverEstado hEstado_ = HoverEstado::Suelo;
    HoverModo hModo_ = HoverModo::SinSustentacion;
    float hPose_ = 0, hRampa_ = 0;           // animacion de las ruedas y rampa de sustentacion (0..1)
    float padSalud_[4] = {1, 1, 1, 1};
    float hCap_ = 0, hReserva_ = 0, hAlt_ = 0, hCalor_ = 0, hBoost_ = 0, hThr_ = 0;
    bool hSobre_ = false, hAterrizando_ = false, hLibre_ = false;
    float hVzObj_ = 0, hVzCmd_ = 0, hZObj_ = 0;  // velocidad vertical pedida, suavizada, y altura de ascenso automatico
    bool hAuto_ = false;                      // ascenso automatico al desplegar
    float hCmd_[3] = {0, 0, 0};               // velocidades angulares pedidas (cabeceo, alabeo, guinada), rad/s
    float hInt_[2] = {0, 0};                  // integral del error de velocidad angular (mantiene la actitud sin ordenes)
    float hEmpujeMot_ = 1.0f, hTurb_[3] = {0, 0, 0}, hTurbT_[3] = {0, 0, 0};
    float hCotas_[4] = {10, 10, 10, 10};
    unsigned hRng_ = 2463534242u;
    float hAltMed_ = 0;                       // altura media de los pads
    int hModoPrev_ = 3;                       // modo del fotograma anterior (3 = sin dato)
    bool libre_[4] = {false, false, false, false};  // ruedas que no apoyan en este subpaso (arrancadas o plegadas)

    // Derivados de configurar()
    float altura_ = 0.55f, batalla_ = 2.8f, via_ = 1.7f, masa_ = 1400;
    float Ip_ = 2700, Ir_ = 600, Iy_ = 2900;  // inercias: cabeceo, alabeo, guinada
    float sRest_ = 0.3f, sMax_ = 0.43f, sMin_ = 0.13f;
    float kS_[4] = {0, 0, 0, 0}, cS_[4] = {0, 0, 0, 0}, kArb_[2] = {18000, 12000};
    float fz0_ = 3400, fd_ = 7.5f, tMax_ = 170, tbMax_ = 5000;
    float frac_[4] = {0, 0, 0.5f, 0.5f};  // reparto del par motor por rueda
};
