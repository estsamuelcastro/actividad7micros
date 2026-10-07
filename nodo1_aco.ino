#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>

// ============================================================
// SISTEMA HIBRIDO ACO - NODE 1 FISICO
// ESP32 DevKit V1 como Access Point + nodo ACO + relay UDP
// ============================================================

const char* SSID = "ACO_ESP32";
const char* PASSWORD = "aco12345";

WiFiUDP udp;
const uint16_t UDP_PORT = 5000;

// -------------------- ACO --------------------
const int N = 9;
const int NUM_EDGES = 10;
const int NUM_ANTS = 18;
const int START_NODE = 0;
const int GOAL_NODE = 8;

const float ALPHA = 1.0f;
const float BETA = 2.0f;
const float RHO = 0.15f;
const float Q = 1.0f;

const int EDGES[NUM_EDGES][2] = {
  {0,1}, {0,3}, {1,4}, {2,5}, {3,6},
  {4,5}, {4,7}, {5,8}, {6,7}, {7,8}
};

float pheromone[N][N];
int bestPath[N];
int bestPathLen = 0;
float bestCost = 999999.0f;

// -------------------- Peers --------------------
struct Peer {
  bool connected;
  int id;
  IPAddress ip;
  uint16_t port;
};

Peer peers[2];
unsigned long lastAco = 0;
unsigned long iteration = 0;

// ============================================================
// UTILIDADES
// ============================================================

bool edgeExists(int a, int b) {
  return pheromone[a][b] > 0.0f;
}

void initPheromone() {
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) {
      pheromone[i][j] = 0.0f;
    }
  }

  for (int i = 0; i < NUM_EDGES; i++) {
    int a = EDGES[i][0];
    int b = EDGES[i][1];
    pheromone[a][b] = 1.0f;
    pheromone[b][a] = 1.0f;
  }
}

float random01() {
  return (float)esp_random() / 4294967295.0f;
}

int degreeOf(int node) {
  int d = 0;
  for (int i = 0; i < N; i++) {
    if (edgeExists(node, i)) d++;
  }
  return d;
}

int getNeighbor(int node, int index) {
  int count = 0;
  for (int i = 0; i < N; i++) {
    if (edgeExists(node, i)) {
      if (count == index) return i;
      count++;
    }
  }
  return -1;
}

// ============================================================
// ACO
// ============================================================

bool buildAntPath(int path[], int &pathLen, float &cost) {
  bool visited[N] = {false};
  int current = START_NODE;
  pathLen = 1;
  path[0] = current;
  visited[current] = true;
  cost = 0.0f;

  for (int step = 0; step < N - 1; step++) {
    if (current == GOAL_NODE) return true;

    int candidates[N];
    float weights[N];
    int count = 0;
    float total = 0.0f;

    for (int i = 0; i < N; i++) {
      if (!edgeExists(current, i) || visited[i]) continue;

      float tau = fmaxf(pheromone[current][i], 0.0001f);
      float eta = 1.0f; // todas las aristas tienen distancia 1
      float w = powf(tau, ALPHA) * powf(eta, BETA);

      candidates[count] = i;
      weights[count] = w;
      total += w;
      count++;
    }

    if (count == 0) return false;

    float pick = random01() * total;
    float acc = 0.0f;
    int next = candidates[count - 1];

    for (int i = 0; i < count; i++) {
      acc += weights[i];
      if (pick <= acc) {
        next = candidates[i];
        break;
      }
    }

    current = next;
    visited[current] = true;
    path[pathLen++] = current;
    cost += 1.0f;
  }

  return current == GOAL_NODE;
}

void evaporate() {
  for (int i = 0; i < NUM_EDGES; i++) {
    int a = EDGES[i][0];
    int b = EDGES[i][1];
    pheromone[a][b] *= (1.0f - RHO);
    pheromone[b][a] = pheromone[a][b];
    if (pheromone[a][b] < 0.01f) {
      pheromone[a][b] = 0.01f;
      pheromone[b][a] = 0.01f;
    }
  }
}

void deposit(const int path[], int pathLen, float cost) {
  if (cost <= 0.0f) return;
  float amount = Q / cost;
  for (int i = 0; i < pathLen - 1; i++) {
    int a = path[i];
    int b = path[i + 1];
    if (!edgeExists(a, b)) continue;
    pheromone[a][b] += amount;
    pheromone[b][a] = pheromone[a][b];
  }
}

void localAcoIteration() {
  iteration++;

  int iterationBest[N];
  int iterationBestLen = 0;
  float iterationBestCost = 999999.0f;
  int successful = 0;

  // generar hormigas y guardar sus soluciones
  int antPaths[NUM_ANTS][N];
  int antLens[NUM_ANTS];
  float antCosts[NUM_ANTS];

  for (int a = 0; a < NUM_ANTS; a++) {
    antLens[a] = 0;
    antCosts[a] = 999999.0f;
    if (buildAntPath(antPaths[a], antLens[a], antCosts[a])) {
      successful++;
      if (antCosts[a] < iterationBestCost) {
        iterationBestCost = antCosts[a];
        iterationBestLen = antLens[a];
        for (int k = 0; k < antLens[a]; k++) {
          iterationBest[k] = antPaths[a][k];
        }
      }
    }
  }

  evaporate();

  for (int a = 0; a < NUM_ANTS; a++) {
    if (antCosts[a] < 999999.0f) {
      deposit(antPaths[a], antLens[a], antCosts[a]);
    }
  }

  if (iterationBestCost < bestCost) {
    bestCost = iterationBestCost;
    bestPathLen = iterationBestLen;
    for (int k = 0; k < iterationBestLen; k++) {
      bestPath[k] = iterationBest[k];
    }

    Serial.print("[NODE 1] NUEVA MEJOR RUTA: ");
    printPath(bestPath, bestPathLen);
    Serial.print("[NODE 1] COSTO: ");
    Serial.println(bestCost, 1);
  }

  Serial.print("[NODE 1] Iteracion ");
  Serial.print(iteration);
  Serial.print(" | exitosas=");
  Serial.print(successful);
  Serial.print(" | mejor=");
  if (bestPathLen > 0) printPath(bestPath, bestPathLen);
  else Serial.println("N/A");
}

void printPath(const int path[], int len) {
  for (int i = 0; i < len; i++) {
    Serial.print(path[i]);
    if (i < len - 1) Serial.print("-");
  }
  Serial.println();
}

// ============================================================
// RED UDP / PROTOCOLO
// H|2 / H|3                  registro
// S|2|READY                  confirmacion
// P|sender|a|b|tau          feromona
// R|sender|cost|ruta         mejor ruta
// ============================================================

int peerIndexById(int id) {
  for (int i = 0; i < 2; i++) {
    if (peers[i].connected && peers[i].id == id) return i;
  }
  return -1;
}

void sendToPeer(int index, const String &msg) {
  if (index < 0 || index >= 2) return;
  if (!peers[index].connected) return;
  udp.beginPacket(peers[index].ip, peers[index].port);
  udp.print(msg);
  udp.endPacket();
}

void broadcastPeers(const String &msg, int exceptId) {
  for (int i = 0; i < 2; i++) {
    if (!peers[i].connected) continue;
    if (peers[i].id == exceptId) continue;
    sendToPeer(i, msg);
  }
}

void sendBestRoute() {
  if (bestPathLen <= 0) return;

  String route = "";
  for (int i = 0; i < bestPathLen; i++) {
    route += String(bestPath[i]);
    if (i < bestPathLen - 1) route += ",";
  }

  String msg = "R|1|" + String(bestCost, 3) + "|" + route;
  broadcastPeers(msg, 0);
}

void sendPheromones() {
  // Se transmite una vez por arista en cada iteracion.
  for (int i = 0; i < NUM_EDGES; i++) {
    int a = EDGES[i][0];
    int b = EDGES[i][1];
    String msg = "P|1|" + String(a) + "|" + String(b) + "|" + String(pheromone[a][b], 4);
    broadcastPeers(msg, 0);
  }
}

void reinforceExternalRoute(const String &routeText, float cost) {
  if (cost <= 0.0f) return;
  float amount = 0.15f * Q / cost;

  int route[N];
  int count = 0;
  int start = 0;

  while (start < routeText.length() && count < N) {
    int comma = routeText.indexOf(',', start);
    String token;
    if (comma == -1) token = routeText.substring(start);
    else token = routeText.substring(start, comma);
    route[count++] = token.toInt();
    if (comma == -1) break;
    start = comma + 1;
  }

  for (int i = 0; i < count - 1; i++) {
    int a = route[i];
    int b = route[i + 1];
    if (!edgeExists(a, b)) continue;
    pheromone[a][b] += amount;
    pheromone[b][a] = pheromone[a][b];
  }
}

void handleRegistration(int nodeId, IPAddress remoteIP, uint16_t remotePort) {
  int idx = peerIndexById(nodeId);

  if (idx < 0) {
    for (int i = 0; i < 2; i++) {
      if (!peers[i].connected) {
        idx = i;
        break;
      }
    }
  }

  if (idx < 0) return;

  peers[idx].connected = true;
  peers[idx].id = nodeId;
  peers[idx].ip = remoteIP;
  peers[idx].port = remotePort;

  Serial.print("[NODE 1] NODE ");
  Serial.print(nodeId);
  Serial.print(" registrado desde ");
  Serial.print(remoteIP);
  Serial.print(":");
  Serial.println(remotePort);

  udp.beginPacket(remoteIP, remotePort);
  udp.print("S|");
  udp.print(nodeId);
  udp.print("|READY");
  udp.endPacket();
}

void handlePacket() {
  int packetSize = udp.parsePacket();
  if (packetSize <= 0) return;

  char buffer[256];
  int len = udp.read(buffer, sizeof(buffer) - 1);
  if (len <= 0) return;
  buffer[len] = '\0';

  String msg = String(buffer);
  Serial.print("[NODE 1] RX: ");
  Serial.println(msg);

  int first = msg.indexOf('|');
  String type = (first == -1) ? msg : msg.substring(0, first);

  if (type == "H") {
    int second = msg.indexOf('|', first + 1);
    int nodeId = msg.substring(first + 1, second == -1 ? msg.length() : second).toInt();
    handleRegistration(nodeId, udp.remoteIP(), udp.remotePort());
    return;
  }

  if (type == "P" || type == "R") {
    // Identificar emisor para evitar devolverle su propio paquete.
    int p1 = msg.indexOf('|');
    int p2 = msg.indexOf('|', p1 + 1);
    int sender = (p1 >= 0 && p2 > p1) ? msg.substring(p1 + 1, p2).toInt() : -1;

    if (type == "P") {
      int p3 = msg.indexOf('|', p2 + 1);
      int p4 = msg.indexOf('|', p3 + 1);
      if (p2 > 0 && p3 > p2 && p4 > p3) {
        int a = msg.substring(p2 + 1, p3).toInt();
        int b = msg.substring(p3 + 1, p4).toInt();
        float value = msg.substring(p4 + 1).toFloat();
        if (edgeExists(a, b)) {
          pheromone[a][b] = 0.7f * pheromone[a][b] + 0.3f * value;
          pheromone[b][a] = pheromone[a][b];
        }
      }
    }

    if (type == "R") {
      int p3 = msg.indexOf('|', p2 + 1);
      int p4 = msg.indexOf('|', p3 + 1);
      if (p2 > 0 && p3 > p2 && p4 > p3) {
        float cost = msg.substring(p2 + 1, p3).toFloat();
        String route = msg.substring(p3 + 1);
        reinforceExternalRoute(route, cost);
      }
    }

    broadcastPeers(msg, sender);
    return;
  }

  if (type == "TEST") {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("OK|");
    udp.print(msg);
    udp.endPacket();
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed((unsigned long)esp_random());

  initPheromone();

  peers[0].connected = false;
  peers[1].connected = false;

  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(SSID, PASSWORD);

  Serial.println();
  Serial.println("==============================================");
  Serial.println("      ACO HIBRIDO - NODE 1 FISICO");
  Serial.println("==============================================");

  if (!ok) {
    Serial.println("ERROR: no se pudo crear el Access Point");
    return;
  }

  udp.begin(UDP_PORT);

  Serial.print("SSID: ");
  Serial.println(SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("UDP: ");
  Serial.println(UDP_PORT);
  Serial.println("Esperando NODE 2 y NODE 3...");
  Serial.println("ACO iniciado.");
}

void loop() {
  handlePacket();

  unsigned long now = millis();
  if (now - lastAco >= 1000) {
    lastAco = now;

    localAcoIteration();
    sendBestRoute();
    sendPheromones();
  }

  delay(5);
}
