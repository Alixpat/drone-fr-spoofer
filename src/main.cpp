/*
 * drone-fr-spoofer
 *
 * Simule des drones émettant le signalement électronique français
 * (arrêté du 27 décembre 2019) sur un M5Stack Stamp S3 (ESP32-S3).
 *
 * Principe :
 *   - L'ESP32 se cale sur le canal 6, sans créer de point d'accès
 *   - Toutes les secondes, il envoie une trame beacon 802.11 brute
 *     contenant un payload Vendor Specific (OUI 0x6A5C35) au format TLV
 *   - Les beacons partent en 802.11b DSSS à 1 Mbit/s, préambule long,
 *     comme beaucoup de balises du commerce ; avec BEACON_OFDM, en
 *     802.11g OFDM à 6 Mbit/s, comme certains drones
 *   - Le numéro de séquence 802.11 compte les beacons de chaque drone,
 *     ce qui permet à un récepteur de savoir combien il en a manqué
 *   - Chaque drone simulé a sa propre trajectoire circulaire
 *   - La LED RGB du Stamp S3 flashe dans la couleur du drone émis
 *   - Les drones émettent en alternance (un par seconde)
 *
 * Basé sur la librairie droneID_FR de Pierre Kancir (GPL v3)
 * https://github.com/khancyr/droneID_FR
 */

#include <Arduino.h>
#include <WiFi.h>
#include "droneID_FR.h"

#include "esp_wifi.h"

// ─────────────────────────────────────────────────────────────────────
//  Configuration générale
// ─────────────────────────────────────────────────────────────────────

#define WIFI_CHANNEL      6      // Canal imposé par la réglementation FR
#define BEACON_INTERVAL  1000    // Intervalle entre émissions (ms), max 3s
#define TX_POWER_DBM      2      // Puissance d'émission (2 à 20 dBm) ; faible pour un essai

// Débit des beacons : 802.11b DSSS à 1 Mbit/s, ou 802.11g OFDM à 6 Mbit/s si
// BEACON_OFDM est défini (environnement m5stack-stamps3-ofdm).
#ifdef BEACON_OFDM
#define BEACON_PROTOCOL   (WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G)
#define BEACON_RATE       WIFI_PHY_RATE_6M
#define BEACON_BASIC_RATE 0x8C   // 6 Mbit/s, débit de base
#define BEACON_RATE_NAME  "6 Mbit/s OFDM"
#else
#define BEACON_PROTOCOL   WIFI_PROTOCOL_11B
#define BEACON_RATE       WIFI_PHY_RATE_1M_L
#define BEACON_BASIC_RATE 0x82   // 1 Mbit/s, débit de base
#define BEACON_RATE_NAME  "1 Mbit/s DSSS"
#endif
#ifdef RGB_BUILTIN
#define RGB_LED_PIN      RGB_BUILTIN  // LED RGB de la carte
#else
#define RGB_LED_PIN      21      // LED RGB WS2812 du Stamp S3
#endif
#define LED_FLASH_MS     80      // Durée du flash LED par drone (ms)

// ─────────────────────────────────────────────────────────────────────
//  Définition d'un drone simulé
// ─────────────────────────────────────────────────────────────────────

struct Drone {
    // Identité
    const char* id;             // ID AlphaTango, 30 chars exactement
    const char* ssid;           // SSID visible dans la beacon
    uint8_t     mac[6];         // Adresse MAC fictive
    uint8_t     color[3];       // Couleur LED {R, G, B}

    // Position de décollage (fixe)
    double home_lat;
    double home_lon;

    // Trajectoire circulaire
    double center_lat;          // Centre du cercle
    double center_lon;
    double radius_deg;          // Rayon (~0.001° ≈ 111m)
    double altitude;            // Altitude MSL (m)
    double height;              // Hauteur par rapport au sol (m)
    double speed;               // Vitesse sol (m/s)
    double angular_speed;       // Vitesse de rotation (°/s, négatif = anti-horaire)

    // État interne
    double angle;               // Position actuelle sur le cercle (°)
    uint16_t sequence;          // Compteur de beacons, 12 bits
};

// ─────────────────────────────────────────────────────────────────────
//  Drones à simuler
//  ID_FR : 30 chars exactement (trigramme constructeur + numéro série)
//  Le trigramme TST et le mot ESSAI disent clairement que ce sont des essais.
//  MAC : adresses administrées localement (premier octet 0x02).
//  Zone : carrefour de l'Anémomètre, parc de l'île Saint-Germain,
//         Issy-les-Moulineaux (48.8002, 2.2487)
// ─────────────────────────────────────────────────────────────────────

static Drone drones[] = {
    {
        //                 123456789012345678901234567890
        .id            = "TST000ESSAI0000000000000000001",
        .ssid          = "DRONE_TEST_01",
        .mac           = {0x02, 0x52, 0x49, 0x44, 0x00, 0x01},
        .color         = {0, 0, 255},       // Bleu
        .home_lat      = 48.799800,
        .home_lon      = 2.248000,
        .center_lat    = 48.800200,
        .center_lon    = 2.248700,
        .radius_deg    = 0.001,             // ~111m
        .altitude      = 120.0,
        .height        = 50.0,
        .speed         = 12,
        .angular_speed = 5.0,               // 72s par tour
        .angle         = 0.0,
    },
    {
        //                 123456789012345678901234567890
        .id            = "TST000ESSAI0000000000000000002",
        .ssid          = "DRONE_TEST_02",
        .mac           = {0x02, 0x52, 0x49, 0x44, 0x00, 0x02},
        .color         = {255, 0, 0},       // Rouge
        .home_lat      = 48.800500,
        .home_lon      = 2.249200,
        .center_lat    = 48.800800,
        .center_lon    = 2.248400,
        .radius_deg    = 0.0015,            // ~167m
        .altitude      = 80.0,
        .height        = 30.0,
        .speed         = 8,
        .angular_speed = -3.0,              // Sens inverse, 120s par tour
        .angle         = 180.0,
    },
};

static const size_t NUM_DRONES = sizeof(drones) / sizeof(drones[0]);

// ─────────────────────────────────────────────────────────────────────
//  Trame beacon 802.11
// ─────────────────────────────────────────────────────────────────────

static const uint16_t MAX_BEACON = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;
static uint8_t beacon[40 + 32 + 251];

static const uint8_t BEACON_HEADER[] = {
    0x80, 0x00,                                     // Frame Control: beacon
    0x00, 0x00,                                     // Duration
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,             // Destination: broadcast
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             // Source: rempli par drone
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             // BSSID: rempli par drone
    0x00, 0x00,                                     // Sequence: rempli par drone
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Timestamp
    0xE8, 0x03,                                     // Beacon interval: 1000 TU
    0x01, 0x00,                                     // Capability info: ESS
};

static droneIDFR idfr;

// ─────────────────────────────────────────────────────────────────────
//  LED RGB
// ─────────────────────────────────────────────────────────────────────

void led_flash(const uint8_t color[3]) {
    rgbLedWrite(RGB_LED_PIN, color[0], color[1], color[2]);
    delay(LED_FLASH_MS);
    rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
}

// ─────────────────────────────────────────────────────────────────────
//  Trajectoire
// ─────────────────────────────────────────────────────────────────────

void update_position(Drone &d, double dt) {
    d.angle += d.angular_speed * dt;
    if (d.angle >= 360.0) d.angle -= 360.0;
    if (d.angle < 0.0)    d.angle += 360.0;
}

void get_position(const Drone &d, double &lat, double &lon) {
    double rad = d.angle * DEG_TO_RAD;
    lat = d.center_lat + d.radius_deg * cos(rad);
    lon = d.center_lon + d.radius_deg * sin(rad);
}

uint16_t get_heading(const Drone &d) {
    double h = d.angle + (d.angular_speed >= 0 ? 90.0 : -90.0);
    while (h < 0)    h += 360.0;
    while (h >= 360) h -= 360.0;
    return (uint16_t)h;
}

// ─────────────────────────────────────────────────────────────────────
//  Construction et envoi de la beacon
// ─────────────────────────────────────────────────────────────────────

void send_beacon(Drone &d) {
    double lat, lon;
    get_position(d, lat, lon);

    memcpy(beacon, BEACON_HEADER, sizeof(BEACON_HEADER));
    memcpy(&beacon[10], d.mac, 6);
    memcpy(&beacon[16], d.mac, 6);
    beacon[22] = (uint8_t)(d.sequence << 4);
    beacon[23] = (uint8_t)(d.sequence >> 4);
    d.sequence = (d.sequence + 1) & 0x0FFF;

    uint16_t pos = sizeof(BEACON_HEADER);
    uint8_t ssid_len = strlen(d.ssid);
    beacon[pos++] = 0x00;
    beacon[pos++] = ssid_len;
    memcpy(&beacon[pos], d.ssid, ssid_len);
    pos += ssid_len;

    beacon[pos++] = 0x01;       // Supported rates
    beacon[pos++] = 0x01;
    beacon[pos++] = BEACON_BASIC_RATE;

    beacon[pos++] = 0x03;
    beacon[pos++] = 0x01;
    beacon[pos++] = WIFI_CHANNEL;

    idfr.set_drone_id(d.id);
    idfr.set_lat_lon(lat, lon);
    idfr.set_altitude((int16_t)d.altitude);
    idfr.set_heigth((int16_t)d.height);
    idfr.set_home_lat_lon(d.home_lat, d.home_lon);
    idfr.set_ground_speed((uint8_t)d.speed);
    idfr.set_heading(get_heading(d));

    uint16_t end = idfr.generate_beacon_frame(beacon, pos);
    esp_err_t err = esp_wifi_80211_tx(WIFI_IF_STA, beacon, end, false);
    if (err != ESP_OK) {
        Serial.printf("[WARN] tx: %s\n", esp_err_to_name(err));
    }
}

// ─────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== DRONE-FR-SPOOFER ===");
    Serial.printf("Drones: %d | Canal: %d | Intervalle: %dms | %s | %d dBm\n\n",
                  NUM_DRONES, WIFI_CHANNEL, BEACON_INTERVAL, BEACON_RATE_NAME, TX_POWER_DBM);

    rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
    for (size_t i = 0; i < NUM_DRONES; i++) {
        rgbLedWrite(RGB_LED_PIN, drones[i].color[0], drones[i].color[1], drones[i].color[2]);
        delay(300);
    }
    rgbLedWrite(RGB_LED_PIN, 0, 0, 0);

    // Pas de point d'accès : il émettrait ses propres beacons en plus des nôtres.
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Le débit des trames brutes ne se règle qu'interface arrêtée.
    esp_wifi_stop();
    esp_wifi_set_protocol(WIFI_IF_STA, BEACON_PROTOCOL);
    esp_err_t err = esp_wifi_config_80211_tx_rate(WIFI_IF_STA, BEACON_RATE);
    if (err != ESP_OK) {
        Serial.printf("[WARN] tx_rate: %s\n", esp_err_to_name(err));
    }
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_set_promiscuous(true);     // nécessaire pour choisir le canal sans association
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_max_tx_power(TX_POWER_DBM * 4);

    Serial.println("Emission en cours...\n");
}

// ─────────────────────────────────────────────────────────────────────
//  Loop — émission alternée : un drone par seconde
// ─────────────────────────────────────────────────────────────────────

static uint32_t last_tx = 0;
static uint32_t tx_count = 0;
static size_t   current_drone = 0;

void loop() {
    uint32_t now = millis();
    if (now - last_tx < BEACON_INTERVAL) return;
    last_tx = now;

    // Compenser le round-robin : chaque drone n'émet que toutes
    // les NUM_DRONES secondes, donc sa trajectoire avance d'autant
    double dt = BEACON_INTERVAL / 1000.0 * NUM_DRONES;
    update_position(drones[current_drone], dt);

    send_beacon(drones[current_drone]);
    led_flash(drones[current_drone].color);

    tx_count++;
    double lat, lon;
    get_position(drones[current_drone], lat, lon);
    Serial.printf("[#%lu] %s seq=%u  %.6f, %.6f  alt=%dm hdg=%03d\n",
                  tx_count,
                  drones[current_drone].ssid,
                  (drones[current_drone].sequence - 1) & 0x0FFF, lat, lon,
                  (int)drones[current_drone].altitude,
                  get_heading(drones[current_drone]));

    current_drone = (current_drone + 1) % NUM_DRONES;
}
