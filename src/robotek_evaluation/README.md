# robotek_evaluation

Todo lo que **solo existe en simulación** y que una estrategia no debe usar nunca (el coche real no tiene nada de esto):

| Módulo / nodo | Qué hace |
|---|---|
| `rule_judge.py` | El juez: puntos según las reglas (secciones, vueltas, parada) y la validación estándar (sin contacto, margen, dirección dentro de ±25°) |
| `contact_recorder` | Guarda los contactos del coche con los muros (`contacts.jsonl`, un renglón por episodio, ver `contact_log.py`) |
| `sim_actuator_model` | Servo y motor del coche real (retraso, límite de velocidad angular, tabla de velocidad) entre tu nodo y Gazebo |
| `sim_scan_model` | Los rayos que pierde el LiDAR LD19 (4,3 % al azar) |
| `open_lab.py` | El Open lab: el campo y el coche en Python puro, unos segundos por ronda; sirve de filtro rápido, Gazebo decide |
| `open_field_geometry.py`, `footprint.py` | Geometría de los muros y huella del coche |

El árbitro de una ronda (`round_recorder`) y `score` están en `robotek_course`.

```bash
python3 -m robotek_evaluation.open_lab --strategy my_open.strategy:Strategy   # las 12 rondas del set estándar
```

Depende de `robotek_common`. Ninguna estrategia depende de este paquete.
