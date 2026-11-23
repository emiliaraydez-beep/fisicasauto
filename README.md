# VehiculoSim — fisica de auto realista para GTA SA (1.0 US)

Reutiliza `Vehiculo.cpp` / `VehiculoHover.cpp` de tu motor sin tocar su logica. El hover queda para tu `hover.cs`
(aqui nunca se llama a `alternarHover`). Cambios respecto a tus archivos: solo `ponerVelocidadMundo()` en `Vehiculo.h`
y un `Colision.h` propio (misma interfaz `suelo` / `contactoEsfera`, sin `formatos/Col.h`).

## Contenido
- `src/Vehiculo.*`, `VehiculoHover.cpp`: el nucleo (probado en Linux: 0-100 km/h en 11.4 s, 138 km/h de punta, frena en 77 m).
- `src/MundoSA.cpp`: `colision::Mundo` sobre `CWorld::ProcessLineOfSight` (edificios y objetos). Vehiculos y peatones los maneja el juego.
- `src/sa.h`: TODA la memoria del juego (offsets y direcciones) en un solo archivo.
- `src/plugin.cpp` + `exports.def`: la DLL. Sin SDK de CLEO: el script la carga con 0AA2/0AA4 y llama con 0AA5.
- `vehiculo.ini`: perfil por modelo (se copia junto a la DLL).
- `.github/workflows/build.yml`: compila la DLL de 32 bits en GitHub (no hace falta PC) y corre la prueba del nucleo.

## Compilar sin PC
1. Sube esta carpeta a un repositorio de GitHub. 2. Pestaña **Actions** → el workflow `build` corre solo.
3. Descarga el artefacto `VehiculoSim` → `VehiculoSim.dll`. 4. Copia `VehiculoSim.dll` y `vehiculo.ini` a `GTA SA\CLEO\`.

## API (todas __stdcall; floats como parametro normal)
| Funcion | Parametros | Hace |
|---|---|---|
| `SimVersion` | — | devuelve 1; sirve para comprobar que cargo |
| `SimAttach` | car (0A97) | crea el simulador con la pose y velocidad actuales del auto; arranca el motor |
| `SimTick` | car, gas, dir, mano | una vez por fotograma; escribe matriz y velocidades en el auto |
| `SimDetach` | car | libera |
| `SimMotor` | car | llave (arranca / apaga) |
| `SimServicio` | car, completo | combustible, aceite, refrigerante, bateria (completo = repara) |
| `SimPonerHumedad` | h 0..1 | lluvia: baja el agarre, aquaplaning |
| `SimTelemetria` | car, float[8]* | km/h, marcha, rpm, temp motor, litros, derrape, motor, salud motor |

## Uso desde el .cs (comprobar la sintaxis exacta en Sanny Builder, "Opcode search")
```
0AA2: 20@ = load_library "CLEO\VehiculoSim.dll"
0AA4: 21@ = get_proc_address "SimAttach" library 20@
0AA4: 22@ = get_proc_address "SimTick"   library 20@
0AA4: 23@ = get_proc_address "SimDetach" library 20@
// una vez, con el auto registrado:
0A97: 3@ = car 0@ struct
0AA5: call 21@ num_params 1 pop 0 3@
// cada fotograma: 5@ = gas (-1..1), 6@ = direccion (-1..1), 7@ = freno de mano (0.0 / 1.0), todos FLOAT
0AA5: call 22@ num_params 4 pop 0 3@ 5@ 6@ 7@
```
Con el hover: llama `SimDetach` cuando hover.cs pase a STATE != 0 (shared var 100, +0) y `SimAttach` al volver a 0.
Teletransportes del viaje en el tiempo: el plugin detecta el salto (>4 m) y se resincroniza solo.

## Como convive con el juego ("marioneta")
El simulador manda: cada fotograma escribe la matriz y las velocidades del auto. El juego sigue corriendo su propia fisica
encima, pero como se pisa cada fotograma no se acumula. Si el juego cambia la velocidad de golpe (>2 m/s: choque con otro auto,
explosion) el simulador adopta esa velocidad. Pierdes: colisiones precisas con otros vehiculos y peatones (las resuelve el juego,
sin la fisica del simulador), y el sonido / ruedas del juego siguen su propia logica.

## Primera prueba (en este orden; el log es `CLEO\VehiculoSim.log`)
1. Solo `load_library` + `SimVersion`: debe aparecer "VehiculoSim cargado" en el log.
2. `SimAttach` y `SimTick` con gas 0 y el auto parado: debe seguir parado; el log escribe una linea `tick` por segundo (~10 s).
   Si el juego crashea aqui, lo mas probable es la direccion de `ProcessLineOfSight` o el layout de `CColPoint` en `sa.h`.
3. Gas 1.0: avanza, sube marchas. Revisa rpm y marcha en el log.

## Por verificar (no pude probar en el juego)
- `kProcessLineOfSight = 0x56BA00` y el layout de `CColPoint` (el material se lee de `surfB`) — datos de la 1.0 US recordados, no medidos.
- La sintaxis de 0AA2/0AA4/0AA5 en tu version de CLEO 4.
- Los parametros de `vehiculo.ini` salen de los valores por defecto del simulador (un auto generico), no del handling de cada auto del juego.
