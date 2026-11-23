#pragma once
// Version del PLUGIN de colision::Mundo: misma interfaz que la de tu motor (V3, Contacto, suelo, contactoEsfera),
// pero sin la rejilla ni formatos/Col.h. La implementacion esta en MundoSA.cpp (consulta al mundo del juego) o en
// test/MundoPlano.cpp (suelo plano, para probar el nucleo en Linux).
#include <cstdint>

namespace colision {

struct V3 {
    float x = 0, y = 0, z = 0;
};

struct Contacto {
    V3 normal;           // hacia fuera de la superficie, en el mundo
    float profundidad = 0;
    int instancia = -1;  // no se usa en el plugin
};

class Mundo {
public:
    // Altura del suelo: primer impacto hacia abajo desde (x,y,zDesde) hasta 'maxBajada' metros.
    bool suelo(float x, float y, float zDesde, float maxBajada, float& zSuelo, V3* normal = nullptr, uint8_t* material = nullptr) const;
    // Contacto mas profundo entre una esfera y el mundo estatico. false si no toca nada.
    bool contactoEsfera(V3 centro, float radio, Contacto& out) const;

    // El auto que se simula: las consultas lo ignoran (solo lo usa MundoSA).
    mutable void* ignorar = nullptr;
};

}  // namespace colision
