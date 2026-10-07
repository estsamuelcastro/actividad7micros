import math
import socket
import threading
import time
from collections import defaultdict

import pybullet as p
import pybullet_data

UDP_PORT = 6000
DEFAULT_ROUTE = [0, 1, 4, 5, 8]

EDGES = {
    tuple(sorted(edge))
    for edge in [
        (0, 1), (0, 3), (1, 4), (2, 5), (3, 6),
        (4, 5), (4, 7), (5, 8), (6, 7), (7, 8)
    ]
}

GRID = {
    0: (0, 0), 1: (1, 0), 2: (2, 0),
    3: (0, 1), 4: (1, 1), 5: (2, 1),
    6: (0, 2), 7: (1, 2), 8: (2, 2),
}
CELL = 2.4

latest_routes = defaultdict(lambda: DEFAULT_ROUTE[:])
latest_costs = {}
running = True


def world_pos(node, z=0.25, offset_x=0.0, offset_y=0.0):
    gx, gy = GRID[node]
    x = (gx - 1) * CELL + offset_x
    y = (gy - 1) * CELL + offset_y
    return (x, y, z)


def receive_routes():
    global running
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    sock.settimeout(0.3)

    while running:
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue
        except OSError:
            break

        msg = data.decode("utf-8", errors="replace").strip()
        parts = msg.split("|")
        if len(parts) < 4 or parts[0] != "R":
            continue
        try:
            node_id = int(parts[1])
            cost = float(parts[2])
            route = [int(x) for x in parts[3].split(",")]
        except ValueError:
            continue

        if node_id in (1, 2, 3) and route and route[0] == 0 and route[-1] == 8:
            latest_routes[node_id] = route
            latest_costs[node_id] = cost
            print(f"[PYBULLET] NODE {node_id}: ruta={route} costo={cost:.1f}")

    sock.close()


def create_box(half_extents, pos, mass=0, rgba=None):
    collision = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
    )
    visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=rgba if rgba else [0.7, 0.7, 0.7, 1],
    )
    return p.createMultiBody(
        baseMass=mass,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=pos,
    )


def create_maze():
    floor = create_box((4.6, 4.6, 0.08), (0, 0, -0.08), rgba=[0.25, 0.25, 0.25, 1])
    _ = floor

    # Bordes del escenario.
    wall_h = 0.7
    t = 0.10
    span = 4.7
    create_box((span, t, wall_h), (0, -span, wall_h), rgba=[0.85, 0.85, 0.85, 1])
    create_box((span, t, wall_h), (0, span, wall_h), rgba=[0.85, 0.85, 0.85, 1])
    create_box((t, span, wall_h), (-span, 0, wall_h), rgba=[0.85, 0.85, 0.85, 1])
    create_box((t, span, wall_h), (span, 0, wall_h), rgba=[0.85, 0.85, 0.85, 1])

    # Paredes internas para enlaces NO permitidos del grid.
    # Vertical separa columnas; horizontal separa filas.
    gap = CELL / 2
    for a, b in [
        (0, 1), (1, 2), (3, 4), (4, 5), (6, 7), (7, 8),
        (0, 3), (1, 4), (2, 5), (3, 6), (4, 7), (5, 8)
    ]:
        key = tuple(sorted((a, b)))
        if key in EDGES:
            continue
        ax, ay = GRID[a]
        bx, by = GRID[b]
        x1, y1, _ = world_pos(a)
        x2, y2, _ = world_pos(b)
        if ax != bx:
            x = (x1 + x2) / 2
            y = y1
            create_box((t, gap, wall_h), (x, y, wall_h), rgba=[0.95, 0.70, 0.20, 1])
        else:
            x = x1
            y = (y1 + y2) / 2
            create_box((gap, t, wall_h), (x, y, wall_h), rgba=[0.95, 0.70, 0.20, 1])

    # Marcadores de inicio y meta.
    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.34, length=0.04,
            rgbaColor=[0.2, 0.8, 0.2, 1]
        ),
        basePosition=world_pos(0, 0.02),
    )
    p.addUserDebugText("INICIO", world_pos(0, 0.15), textSize=1.2, textColorRGB=[0.2, 0.8, 0.2])

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.34, length=0.04,
            rgbaColor=[0.9, 0.2, 0.2, 1]
        ),
        basePosition=world_pos(8, 0.02),
    )
    p.addUserDebugText("META", world_pos(8, 0.15), textSize=1.2, textColorRGB=[0.9, 0.2, 0.2])


def create_robot(node_id, offset_x):
    pos = world_pos(0, 0.30, offset_x, 0.0)
    rgba = {
        1: [0.95, 0.25, 0.25, 1],
        2: [0.25, 0.75, 0.95, 1],
        3: [0.95, 0.75, 0.20, 1],
    }[node_id]
    return create_box((0.28, 0.28, 0.18), pos, mass=1, rgba=rgba)


def draw_robot_label(node_id, body_id):
    pos, _ = p.getBasePositionAndOrientation(body_id)
    p.addUserDebugText(
        f"NODE {node_id}",
        (pos[0], pos[1], pos[2] + 0.45),
        textSize=1.0,
        textColorRGB=[1, 1, 1],
        lifeTime=0.15,
    )


def main():
    global running

    threading.Thread(target=receive_routes, daemon=True).start()

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.resetDebugVisualizerCamera(
        cameraDistance=12.5,
        cameraYaw=45,
        cameraPitch=-55,
        cameraTargetPosition=(0, 0, 0),
    )

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    create_maze()

    robots = {
        1: create_robot(1, -0.55),
        2: create_robot(2, 0.0),
        3: create_robot(3, 0.55),
    }

    progress = {1: 1, 2: 1, 3: 1}
    last_move = time.time()
    step_interval = 0.55

    print("[PYBULLET] Simulacion iniciada.")
    print("[PYBULLET] Esperando rutas ACO de NODE 1, 2 y 3...")

    try:
        while p.isConnected():
            now = time.time()

            if now - last_move >= step_interval:
                last_move = now
                for node_id, body_id in robots.items():
                    route = latest_routes[node_id]
                    idx = progress[node_id]
                    if idx < len(route):
                        pos = world_pos(route[idx], 0.30, (-0.55, 0, 0.55)[node_id - 1], 0.0)
                        p.resetBasePositionAndOrientation(body_id, pos, [0, 0, 0, 1])
                        progress[node_id] += 1
                    else:
                        # Una vez en la meta, reinicia para mostrar continuamente la colonia.
                        progress[node_id] = 1
                        pos = world_pos(0, 0.30, (-0.55, 0, 0.55)[node_id - 1], 0.0)
                        p.resetBasePositionAndOrientation(body_id, pos, [0, 0, 0, 1])

            # Cabecera.
            p.addUserDebugText(
                "ACO HIBRIDO | 1 ESP32 FISICO + 2 ESP32 VIRTUALES",
                (-4.2, 5.1, 0.4),
                textSize=1.2,
                textColorRGB=[1, 1, 1],
                lifeTime=0.15,
            )

            p.addUserDebugText(
                "Feromonas compartidas por UDP | Rutas calculadas por ACO",
                (-3.3, 4.55, 0.25),
                textSize=0.9,
                textColorRGB=[0.8, 0.8, 0.8],
                lifeTime=0.15,
            )

            for node_id, body_id in robots.items():
                draw_robot_label(node_id, body_id)

            p.stepSimulation()
            time.sleep(1.0 / 60.0)

    except KeyboardInterrupt:
        pass
    finally:
        running = False
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
