import os
import random
import socket
import sys
import threading
import time

# ============================================================
# ESP32 VIRTUAL + ACO
# Uso:
#   python virtual_node.py 2
#   python virtual_node.py 3
# ============================================================

ESP32_IP = os.getenv("ESP32_IP", "192.168.4.1")
ESP32_PORT = int(os.getenv("ESP32_PORT", "5000"))
VISUALIZER_IP = os.getenv("VISUALIZER_IP", "127.0.0.1")
VISUALIZER_PORT = int(os.getenv("VISUALIZER_PORT", "6000"))

ALPHA = 1.0
BETA = 2.0
RHO = 0.15
Q = 1.0
NUM_ANTS = 18
START_NODE = 0
GOAL_NODE = 8
ITERATION_DELAY = 1.0

NODES = 9
EDGES = [
    (0, 1), (0, 3), (1, 4), (2, 5), (3, 6),
    (4, 5), (4, 7), (5, 8), (6, 7), (7, 8)
]

NEIGHBORS = {node: [] for node in range(NODES)}
for a, b in EDGES:
    NEIGHBORS[a].append(b)
    NEIGHBORS[b].append(a)


class VirtualACO:
    def __init__(self, node_id: int):
        if node_id not in (2, 3):
            raise ValueError("El nodo virtual debe ser 2 o 3")

        self.node_id = node_id
        self.local_port = 5000 + node_id
        self.random = random.Random(2026 + node_id)
        self.running = True
        self.connected = False
        self.iteration = 0
        self.best_path = None
        self.best_cost = float("inf")
        self.lock = threading.Lock()

        self.pheromone = {
            self.edge_key(a, b): 1.0
            for a, b in EDGES
        }

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.local_port))
        self.sock.settimeout(0.2)

    @staticmethod
    def edge_key(a: int, b: int):
        return tuple(sorted((a, b)))

    def distance(self, _a: int, _b: int) -> float:
        return 1.0

    def send(self, message: str):
        try:
            self.sock.sendto(
                message.encode("utf-8"),
                (ESP32_IP, ESP32_PORT),
            )
        except OSError as exc:
            print(f"[NODE {self.node_id}] UDP error: {exc}")

    def send_visualizer(self, message: str):
        try:
            self.sock.sendto(
                message.encode("utf-8"),
                (VISUALIZER_IP, VISUALIZER_PORT),
            )
        except OSError:
            # La visualización puede iniciar después del nodo.
            pass

    def register(self):
        self.send(f"H|{self.node_id}")
        print(f"[NODE {self.node_id}] Registro -> {ESP32_IP}:{ESP32_PORT}")

    def receive_loop(self):
        while self.running:
            try:
                data, _addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break

            message = data.decode("utf-8", errors="replace").strip()
            if message:
                self.process_message(message)

    def process_message(self, message: str):
        parts = message.split("|")
        if not parts:
            return

        if parts[0] == "S" and len(parts) >= 3:
            self.connected = parts[2] == "READY"
            print(f"[NODE {self.node_id}] ESP32 -> {message}")
            if self.connected:
                print(f"[NODE {self.node_id}] ✓ RED ACO CONECTADA")
            return

        if parts[0] == "P" and len(parts) >= 5:
            try:
                sender = int(parts[1])
                a = int(parts[2])
                b = int(parts[3])
                value = float(parts[4])
            except ValueError:
                return

            if sender == self.node_id:
                return

            key = self.edge_key(a, b)
            if key not in self.pheromone:
                return

            with self.lock:
                self.pheromone[key] = (
                    0.7 * self.pheromone[key] + 0.3 * value
                )

            if self.iteration % 3 == 0:
                print(
                    f"[NODE {self.node_id}] FEROMONA de NODE {sender}: "
                    f"{a}-{b}={value:.3f}"
                )
            return

        if parts[0] == "R" and len(parts) >= 4:
            try:
                sender = int(parts[1])
                cost = float(parts[2])
                path = [int(x) for x in parts[3].split(",")]
            except ValueError:
                return

            if sender == self.node_id:
                return

            # La ruta externa refuerza localmente las mismas aristas.
            reinforcement = 0.15 * Q / max(cost, 1.0)
            with self.lock:
                for a, b in zip(path, path[1:]):
                    key = self.edge_key(a, b)
                    if key in self.pheromone:
                        self.pheromone[key] += reinforcement

            print(
                f"[NODE {self.node_id}] RUTA de NODE {sender}: "
                f"{path} costo={cost:.1f}"
            )
            self.send_visualizer(message)

    def choose_next(self, current: int, visited: set[int]):
        candidates = [n for n in NEIGHBORS[current] if n not in visited]
        if not candidates:
            return None

        weights = []
        total = 0.0
        with self.lock:
            for candidate in candidates:
                tau = max(self.pheromone[self.edge_key(current, candidate)], 1e-4)
                eta = 1.0 / self.distance(current, candidate)
                weight = (tau ** ALPHA) * (eta ** BETA)
                weights.append((candidate, weight))
                total += weight

        if total <= 0:
            return self.random.choice(candidates)

        pick = self.random.random() * total
        acc = 0.0
        for candidate, weight in weights:
            acc += weight
            if pick <= acc:
                return candidate
        return candidates[-1]

    def build_path(self):
        current = START_NODE
        path = [current]
        visited = {current}

        while current != GOAL_NODE and len(path) < NODES:
            nxt = self.choose_next(current, visited)
            if nxt is None:
                return None
            path.append(nxt)
            visited.add(nxt)
            current = nxt

        return path if current == GOAL_NODE else None

    @staticmethod
    def cost(path):
        return float(len(path) - 1) if path else float("inf")

    def evaporate(self):
        with self.lock:
            for key in self.pheromone:
                self.pheromone[key] = max(
                    self.pheromone[key] * (1.0 - RHO),
                    0.01,
                )

    def deposit(self, path, cost):
        if not path or cost <= 0:
            return
        amount = Q / cost
        with self.lock:
            for a, b in zip(path, path[1:]):
                key = self.edge_key(a, b)
                if key in self.pheromone:
                    self.pheromone[key] += amount

    def publish_pheromones(self):
        with self.lock:
            values = [
                (a, b, self.pheromone[self.edge_key(a, b)])
                for a, b in EDGES
            ]

        for a, b, value in values:
            self.send(f"P|{self.node_id}|{a}|{b}|{value:.4f}")

    def publish_best_route(self):
        if not self.best_path:
            return
        route = ",".join(map(str, self.best_path))
        msg = f"R|{self.node_id}|{self.best_cost:.3f}|{route}"
        self.send(msg)
        self.send_visualizer(msg)

    def iteration_step(self):
        self.iteration += 1
        solutions = []

        for _ in range(NUM_ANTS):
            path = self.build_path()
            if path:
                solutions.append((path, self.cost(path)))

        self.evaporate()
        for path, cost in solutions:
            self.deposit(path, cost)

        if solutions:
            iteration_best = min(solutions, key=lambda x: x[1])
            path, cost = iteration_best
            if cost < self.best_cost:
                self.best_path = path[:]
                self.best_cost = cost
                print(
                    f"[NODE {self.node_id}] ★ NUEVA MEJOR: "
                    f"{self.best_path} costo={self.best_cost:.1f}"
                )

        print(
            f"[NODE {self.node_id}] iter={self.iteration:03d} "
            f"exitosas={len(solutions):02d} "
            f"mejor={self.best_path} "
            f"costo={self.best_cost if self.best_path else 'N/A'}"
        )

        self.publish_best_route()
        self.publish_pheromones()

    def run(self):
        print("=" * 60)
        print(f"ESP32 VIRTUAL - NODE {self.node_id}")
        print("=" * 60)
        print(f"ESP32 fisico : {ESP32_IP}:{ESP32_PORT}")
        print(f"UDP local    : 0.0.0.0:{self.local_port}")
        print(f"Visualizador  : {VISUALIZER_IP}:{VISUALIZER_PORT}")
        print()

        receiver = threading.Thread(target=self.receive_loop, daemon=True)
        receiver.start()

        self.register()
        time.sleep(0.8)

        print(f"[NODE {self.node_id}] Iniciando ACO...")
        print(f"[NODE {self.node_id}] Inicio={START_NODE}, Meta={GOAL_NODE}")

        try:
            while self.running:
                self.iteration_step()
                time.sleep(ITERATION_DELAY)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            try:
                self.sock.close()
            except OSError:
                pass


def main():
    if len(sys.argv) != 2:
        print("Uso: python virtual_node.py 2")
        print("   o: python virtual_node.py 3")
        raise SystemExit(1)

    try:
        node_id = int(sys.argv[1])
    except ValueError:
        print("ERROR: el ID debe ser 2 o 3")
        raise SystemExit(1)

    VirtualACO(node_id).run()


if __name__ == "__main__":
    main()
