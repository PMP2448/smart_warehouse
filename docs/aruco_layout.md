# Distribución de Códigos ArUco en el Almacén

## Vista superior del almacén

```
                    Pared Norte (y=13)
    ┌─────────────────────────────────────────────────────────┐
    │                                                         │
    │   ┌─────────────────────────────────────────────────────┐
    │   │  [3]     [2]     [1]     [0]  ← ArUcos (0-3: +X→-X) │
    │   │  P4      P3      P2      P1   ← Pallets             │
    │   │                                                     │
    │   │           SHELF 1 (y=5)                             │
    │   │                                                     │
    │   │  P5      P7      P6      P8   ← Pallets             │
    │   │  [4]     [5]     [6]     [7]  ← ArUcos (4-7: -X→+X) │
    │   └─────────────────────────────────────────────────────┘
    │                                                         │
    │                     PASILLO (y=0)                       │     ┌───────────┐
    │                                                         │     │  P4  [19] │
    │   ┌─────────────────────────────────────────────────────┐     │  P3  [18] │
    │   │  [11]    [10]    [9]     [8]  ← ArUcos (8-11: +X→-X)│     │           │
    │   │  P5      P7      P6      P8   ← Pallets             │     │  SHELF 3  │
    │   │                                                     │     │  (x=12)   │
    │   │           SHELF 2 (y=-5)                            │     │  16-19:   │
    │   │                                                     │     │  -Y → +Y  │
    │   │  P4      P3      P2      P1   ← Pallets             │     │  P2  [17] │
    │   │  [12]    [13]    [14]    [15] ← ArUcos (12-15:-X→+X)│     │  P1  [16] │
    │   └─────────────────────────────────────────────────────┘     └───────────┘
    │                                                         │
    └─────────────────────────────────────────────────────────┘
                    Pared Sur (y=-13)

    ←── x negativo                            x positivo ──→
```

## Criterio de numeración

| Zona | IDs | Sentido de numeración |
|------|-----|----------------------|
| Shelf 1, lado +Y (pasillo norte) | 0-3 | +X → -X (derecha a izquierda) |
| Shelf 1, lado -Y (pasillo central) | 4-7 | -X → +X (izquierda a derecha) |
| Shelf 2, lado +Y (pasillo central) | 8-11 | +X → -X (derecha a izquierda) |
| Shelf 2, lado -Y (pasillo sur) | 12-15 | -X → +X (izquierda a derecha) |
| Shelf 3 (pasillo este) | 16-19 | -Y → +Y (abajo a arriba) |

## Tabla de asignación ArUco → Posición

| ArUco ID | Estantería | Lado | Posición Pallet | Coordenadas (x, y) |
|----------|------------|------|-----------------|-------------------|
| 0 | Shelf 1 | +Y (Norte) | pallet_s1p1 | (6.65, 7.25) |
| 1 | Shelf 1 | +Y (Norte) | pallet_s1p2 | (2, 7.25) |
| 2 | Shelf 1 | +Y (Norte) | pallet_s1p3 | (-2, 7.25) |
| 3 | Shelf 1 | +Y (Norte) | pallet_s1p4 | (-6.65, 7.25) |
| 4 | Shelf 1 | -Y (Centro) | pallet_s1p5 | (-6.65, 2.9) |
| 5 | Shelf 1 | -Y (Centro) | pallet_s1p7 | (-2, 2.9) |
| 6 | Shelf 1 | -Y (Centro) | pallet_s1p6 | (2, 2.9) |
| 7 | Shelf 1 | -Y (Centro) | pallet_s1p8 | (6.65, 2.9) |
| 8 | Shelf 2 | +Y (Centro) | pallet_s2p8 | (6.65, -2.9) |
| 9 | Shelf 2 | +Y (Centro) | pallet_s2p6 | (2, -2.9) |
| 10 | Shelf 2 | +Y (Centro) | pallet_s2p7 | (-2, -2.9) |
| 11 | Shelf 2 | +Y (Centro) | pallet_s2p5 | (-6.65, -2.9) |
| 12 | Shelf 2 | -Y (Sur) | pallet_s2p4 | (-6.65, -7.25) |
| 13 | Shelf 2 | -Y (Sur) | pallet_s2p3 | (-2, -7.25) |
| 14 | Shelf 2 | -Y (Sur) | pallet_s2p2 | (2, -7.25) |
| 15 | Shelf 2 | -Y (Sur) | pallet_s2p1 | (6.65, -7.25) |
| 16 | Shelf 3 | +X (Este) | pallet_s3p1 | (14, -6.6) |
| 17 | Shelf 3 | +X (Este) | pallet_s3p2 | (14, -2.15) |
| 18 | Shelf 3 | +X (Este) | pallet_s3p3 | (14, 2.15) |
| 19 | Shelf 3 | +X (Este) | pallet_s3p4 | (14, 6.6) |

## Orientación de los ArUcos

- **Shelf 1 lado +Y**: ArUcos miran hacia +Y (hacia la pared norte)
- **Shelf 1 lado -Y**: ArUcos miran hacia -Y (hacia el pasillo central)
- **Shelf 2 lado +Y**: ArUcos miran hacia +Y (hacia el pasillo central)
- **Shelf 2 lado -Y**: ArUcos miran hacia -Y (hacia la pared sur)
- **Shelf 3**: ArUcos miran hacia +X (hacia la pared este)

## Archivos de textura

Los archivos de textura ArUco se encuentran en:
```
/home/pmp/ros2_ws/src/smart_warehouse/aruco_textures/
├── aruco_texture_id0.png
├── aruco_texture_id1.png
├── ...
└── aruco_texture_id19.png
```

Generados con:
```bash
python3 aruco_generator.py --range 0 19 --ancho 0.33m
```
