# Resultados: robot_localization con odometría degradada + IMU

Domingo 4-oct-2026, TurtleBot 4 en Gazebo Harmonic (ROS 2 Jazzy), Nav2 + AMCL
sobre el mapa de `../NAV2`. Todo en simulación.

## Qué se ejecutó

- Recorrido: rectángulo de 6 × 4,1 m con giros siempre a la izquierda,
  3 vueltas (~62 m, ~277 s) por ejecución, objetivos orientados en la
  dirección de avance (`waypoints.py --orientar`).
- 6 condiciones × 2 repeticiones = 12 ejecuciones (`bateria.sh`). 144/144
  objetivos alcanzados.
- Por ejecución se graban a la vez, contra `/sim_ground_truth_pose`: la
  odometría del simulador, la degradada, la del EKF y la de AMCL
  (`map -> base_link`). Nav2 navega con la odometría del simulador; el EKF
  solo observa (`publish_tf: false`).
- IMU: la del simulador con ruido de giróscopo MEMS (0,0015 rad/s) puesto por
  `imu_realista.py`.

Las hipótesis se escribieron en `README.md` antes de la batería, después de
una prueba de humo de 1 vuelta que no cuenta como resultado.

## Tabla (media de 2 repeticiones; desviación entre repeticiones ≤ 0,03 m en el EKF)

| condición | estimador | ATE (m) | error final (% dist.) | escala dist. | escala giro | error de orientación final |
|---|---|---|---|---|---|---|
| `nada` | odometría del simulador | 2,20 | 6,8 | 1,000 | 1,060 | — |
| | **EKF** | **0,13** | **0,45** | 1,000 | 1,004 | — |
| `giro` | odometría degradada | 4,27 | 6,6 | 1,000 | 1,256 | |
| | **EKF** | **0,30** | **0,97** | 1,000 | 1,009 | |
| `lineal` | odometría degradada | 1,97 | 6,2 | **1,050** | 1,060 | |
| | **EKF** | **0,25** | **0,42** | **1,050** | 1,004 | |
| `ambos` | odometría degradada | 4,29 | 6,5 | 1,020 | 1,256 | |
| | **EKF** | **0,31** | **0,99** | 1,020 | 1,009 | 10,0° |
| `ruido` | odometría degradada | 4,27 | 6,7 | 1,020 | 1,257 | |
| | **EKF** | **0,11** | **0,27** | 1,020 | 1,002 | 2,6° |
| `imu_sesgo` | odometría degradada | 4,29 | 6,7 | 1,020 | 1,256 | |
| | **EKF** | **1,21** | **3,8** | 1,020 | 1,037 | 40,1° |
| todas | AMCL (`slam`) | 0,23–0,29 | 0,12–0,32 | 0,96 | 1,00 | |

ATE alineado en la pose inicial. Tabla completa, con ATE Umeyama y RPE por
ejecución: `resultados/resumen.txt` (`python3 evaluar.py resultados/2026100*_*/`).

Figuras: `figura_ambos.png` (condición `ambos`), `figura_imu_sesgo.png`.

## Hipótesis

1. **La IMU arregla el giro: se cumple.** En `giro` y `ambos` la escala de
   giro pasa de 1,256 a 1,009 y el ATE de ~4,3 m a ~0,3 m (÷14).
2. **La IMU no arregla la escala lineal: se cumple.** En `lineal` la escala
   de distancia del EKF es 1,050, idéntica a la de la odometría degradada. Su
   ATE baja de 1,97 a 0,25 m de todos modos, pero por arreglar el giro que
   trae el simulador, no la escala: por eso esta hipótesis se mide con la
   escala y no con el ATE.
3. **AMCL no deriva: se cumple.** Error final entre 0,12 y 0,32 % en todas
   las condiciones. Su escala de distancia (0,96) y su RPE local (~8 %) salen
   peores que los del EKF: AMCL corrige a saltos. No se ha investigado más.
4. **El ruido blanco no cambia el resultado del EKF: falla, y al revés de lo
   esperado.** Con ruido el EKF es *mejor* que sin él (ATE 0,11 frente a
   0,31 m; error de orientación 2,6° frente a 10°). Explicación probable, **sin
   verificar**: `degradar_odom.py` sube la covarianza declarada del giro
   cuando hay ruido (0,05² frente a 0,02² de mínimo), así que el EKF se apoya
   aún más en la IMU y le entra menos error de la odometría. Prueba que lo
   confirmaría: `ambos` sin ruido pero con la covarianza de `ruido`.
5. **El EKF no puede quitar un sesgo de la IMU: se cumple; la cifra se quedó
   corta.** Error de orientación de 40,1° en las dos repeticiones. Predicho:
   sesgo × tiempo = 0,002 rad/s × 277 s ≈ 31,7°. La diferencia (~8°) cuadra
   con el error que el EKF ya tiene en `ambos` sin sesgo (10°): los dos se
   suman. El ATE se multiplica por 4 (0,31 → 1,21 m) y aun así es menor que el
   de la odometría degradada (4,29 m).

## Hallazgos que no estaban en el plan

- **La odometría del simulador no es perfecta**: gira un 6 % de más (escala
  de giro 1,060 en todas las ejecuciones), ~70° de error de orientación en
  3 vueltas, con la distancia exacta. El controlador y el URDF coinciden en la
  distancia entre ruedas (0,233 m); lo más probable es deslizamiento en los
  giros dentro del motor físico (no comprobado). El EKF con IMU lo corrige
  incluso sin degradar nada (`nada`: ATE de 2,20 a 0,13 m).
- **Con objetivos orientados todos a +x los errores de giro se cancelaban**:
  el robot giraba y desgiraba en cada esquina. Las 2 ejecuciones hechas así
  están en `resultados/descartadas_orientacion_fija/` y no entran en la tabla.

## Limitaciones

- `degradar_odom.py` declara covarianzas que *conocen* el error inyectado:
  el EKF sabe cuánto desconfiar de la odometría. Fuera del simulador eso hay
  que ajustarlo a mano (y la hipótesis 4 muestra lo mucho que pesa).
- Modelo de IMU sencillo: ruido blanco y sesgo constante, sin paseo aleatorio
  del sesgo ni temperatura.
- 2 repeticiones de un único recorrido en un único mundo. La dispersión entre
  repeticiones es pequeña porque el simulador es casi determinista: no mide
  la variabilidad de un robot físico.
- La verdad del terreno la regala el simulador; fuera haría falta captura de
  movimiento o RTK.

## Qué hubo que arreglar en los scripts (nunca se habían ejecutado)

1. `experimento.sh` con `set -u` abortaba al hacer `source` del setup de ROS.
2. Finales de línea CRLF en `experimento.sh`, `registrar.py` y `evaluar.py`.
3. **No había IMU**: el mundo `warehouse` no carga `gz-sim-imu-system`.
   `experimento.sh` lo añade en caliente
   (`/world/warehouse/entity/system/add`) y puentea el topic a ROS.
4. La IMU de Gazebo llega con `frame_id` `turtlebot4/imu_link/imu`, que no
   está en el árbol TF, y con ruido y covarianza 0: nuevo `imu_realista.py`.
5. Verdad del simulador: en lugar del puente de `dynamic_pose/info`, se usa
   `/sim_ground_truth_pose`, que ya está en ROS. Llega *best effort*: hace
   falta suscribirse con QoS de sensor.
6. Los waypoints por defecto incluían `0,0`, que es la base de carga: un
   obstáculo que el LiDAR no ve (ver `../NAV2/RESULTADOS.md`).
7. La condición `giro` heredaba `--escala-v 1.02` por defecto: no era pura.
8. `evaluar.py`: añadidas escala de distancia, escala de giro, error de
   orientación final y un resumen por condición.
9. `waypoints.py --orientar` (ver «Hallazgos»).
10. `limpiar()` en `experimento.sh` no paraba `waypoints.py` si se
    interrumpía la ejecución.
