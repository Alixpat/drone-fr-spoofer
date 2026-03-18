/*
 * drone-fr-spoofer
 *
 * Simule des drones émettant le signalement électronique français
 * (arrêté du 27 décembre 2019) sur un M5Stack Stamp S3 (ESP32-S3).
 *
 * Principe :
 *   - L'ESP32 crée un point d'accès Wi-Fi sur le canal 6
 *   - Toutes les secondes, il envoie une trame beacon 802.11 brute
 *     contenant un payload Vendor Specific (OUI 0x6A5C35) au format TLV
 *   - Chaque drone simulé a sa propre trajectoire circulaire
 *   - La LED RGB du Stamp S3 flashe dans la couleur du drone émis
 *
 * Basé sur la librairie droneID_FR de Pierre Kancir (GPL v3)
 * https://github.com/khancyr/droneID_FR
 */

#include <Arduino.h>
#include <WiFi.h>
#include "droneID_FR.h"

// Fonctions internes ESP32 pour envoyer des trames Wi-Fi brutes
extern "C" {
#include "esp_wifi.h"
    esp_err_t esp_wifi_80211_tx(wifi_interface_t ifx, const void *buffer, int len, bool en_sys_seq);
    esp_err_t esp_wifi_internal_set_fix_rate(wifi_interface_t ifx, bool en, wifi_phy_rate_t rate);
}

// ─────────────────────────────────────────────────────────────────────
//  Configuration générale
// ─────────────────────────────────────────────────────────────────────

#define WIFI_CHANNEL      6      // Canal imposé par la réglementation FR
#define BEACON_INTERVAL  1000    // Intervalle entre émissions (ms), max 3s
#define RGB_LED_PIN      21      // LED RGB WS2812 du Stamp S3
#define LED_FLASH_MS     80      // Durée du flash LED par drone (ms)

// ─────────────────────────────────────────────────────────────────────
//  Définition d'un drone simulé
// ─────────────────────────────────────────────────────────────────────

struct Drone {
    // Identité
    const char* id;             // ID AlphaTango, 30 chars max
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
};

// ─────────────────────────────────────────────────────────────────────
//  Drones à simuler — modifier ici
// ─────────────────────────────────────────────────────────────────────

static Drone drones[] = {
    {
        .id            = "000FAKE01_TEST_DRONE_0001A",
        .ssid          = "DRONE_TEST_01",
        .mac           = {0xAA, 0xBB, 0xCC, 0x01, 0x01, 0x01},
        .color         = {0, 0, 255},       // Bleu
        .home_lat      = 48.825000,         // Issy-les-Moulineaux
        .home_lon      = 2.260000,
        .center_lat    = 48.826000,
        .center_lon    = 2.261000,
        .radius_deg    = 0.001,             // ~111m
        .altitude      = 120.0,
        .height        = 50.0,
        .speed         = 12,
        .angular_speed = 5.0,               // 72s par tour
        .angle         = 0.0,
    },
    {
        .id            = "000FAKE02_TEST_DRONE_0002B",
        .ssid          = "DRONE_TEST_02",
        .mac           = {0xAA, 0xBB, 0xCC, 0x02, 0x02, 0x02},
        .color         = {255, 0, 0},       // Rouge
        .home_lat      = 48.830000,
        .home_lon      = 2.265000,
        .center_lat    = 48.831000,
        .center_lon    = 2.266000,
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

// Structure d'une beacon 802.11 :
//   [Frame Control][Duration][Dest MAC][Src MAC][BSSID][Seq]
//   [Timestamp][Beacon Interval][Capability]
//   [Tag SSID][Tag Supported Rates][Tag DS Channel]
//   [Tag Vendor Specific → payload droneID_FR]

static const uint16_t MAX_BEACON = 40 + 32 + droneIDFR::FRAME_PAYLOAD_LEN_MAX;
static uint8_t beacon[40 + 32 + 251];

// En-tête fixe de la beacon (jusqu'au tag SSID exclu)
static const uint8_t BEACON_HEADER[] = {
    0x80, 0x00,                                     // Frame Control: beacon
    0x00, 0x00,                                     // Duration
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,             // Destination: broadcast
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             // Source: rempli par drone
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             // BSSID: rempli par drone
    0x00, 0x00,                                     // Sequence (géré par ESP)
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Timestamp
    0xE8, 0x03,                                     // Beacon interval: 1000 TU
    0x21, 0x04,                                     // Capability info
};

static droneIDFR idfr;  // Instance de la librairie

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

// Fait avancer le drone sur son cercle
void update_position(Drone &d, double dt) {
    d.angle += d.angular_speed * dt;
    if (d.angle >= 360.0) d.angle -= 360.0;
    if (d.angle < 0.0)    d.angle += 360.0;
}

// Calcule lat/lon à partir du centre + angle + rayon
void get_position(const Drone &d, double &lat, double &lon) {
    double rad = d.angle * DEG_TO_RAD;
    lat = d.center_lat + d.radius_deg * cos(rad);
    lon = d.center_lon + d.radius_deg * sin(rad);
}

// Cap tangent au cercle (perpendiculaire au rayon)
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

    // 1) Copier l'en-tête fixe
    memcpy(beacon, BEACON_HEADER, sizeof(BEACON_HEADER));

    // 2) Injecter la MAC source et BSSID
    memcpy(&beacon[10], d.mac, 6);  // Source
    memcpy(&beacon[16], d.mac, 6);  // BSSID

    // 3) Tag SSID (id 0x00, longueur, données)
    uint16_t pos = sizeof(BEACON_HEADER);
    uint8_t ssid_len = strlen(d.ssid);
    beacon[pos++] = 0x00;           // Tag ID: SSID
    beacon[pos++] = ssid_len;       // Longueur
    memcpy(&beacon[pos], d.ssid, ssid_len);
    pos += ssid_len;

    // 4) Tag Supported Rates (obligatoire pour une beacon valide)
    beacon[pos++] = 0x01;           // Tag ID: Supported Rates
    beacon[pos++] = 0x01;           // Longueur: 1
    beacon[pos++] = 0x0C;           // 6 Mbps

    // 5) Tag DS Parameter Set (canal Wi-Fi)
    beacon[pos++] = 0x03;           // Tag ID: DS Parameter
    beacon[pos++] = 0x01;           // Longueur: 1
    beacon[pos++] = WIFI_CHANNEL;

    // 6) Remplir les données drone dans la librairie
    idfr.set_drone_id(d.id);
    idfr.set_lat_lon(lat, lon);
    idfr.set_altitude((int16_t)d.altitude);
    idfr.set_heigth((int16_t)d.height);
    idfr.set_home_lat_lon(d.home_lat, d.home_lon);
    idfr.set_ground_speed((uint8_t)d.speed);
    idfr.set_heading(get_heading(d));

    // 7) Générer le tag Vendor Specific (OUI 6A:5C:35 + TLV)
    uint16_t end = idfr.generate_beacon_frame(beacon, pos);

    // 8) Envoyer la trame brute sur l'interface Wi-Fi
    esp_wifi_80211_tx(WIFI_IF_AP, beacon, end, true);
}

// ─────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== DRONE-FR-SPOOFER ===");
    Serial.printf("Drones: %d | Canal: %d | Intervalle: %dms\n\n",
                  NUM_DRONES, WIFI_CHANNEL, BEACON_INTERVAL);

    // Test LED : flash chaque couleur de drone au démarrage
    rgbLedWrite(RGB_LED_PIN, 0, 0, 0);
    for (size_t i = 0; i < NUM_DRONES; i++) {
        rgbLedWrite(RGB_LED_PIN, drones[i].color[0], drones[i].color[1], drones[i].color[2]);
        delay(300);
    }
    rgbLedWrite(RGB_LED_PIN, 0, 0, 0);

    // Démarrer le Wi-Fi en mode AP sur le canal réglementaire
    WiFi.softAP(drones[0].ssid, nullptr, WIFI_CHANNEL);
    delay(500);

    // Beacon interval à 1s (au lieu de 100ms par défaut)
    wifi_config_t cfg;
    esp_wifi_get_config(WIFI_IF_AP, &cfg);
    cfg.ap.beacon_interval = 1000;
    esp_wifi_set_config(WIFI_IF_AP, &cfg);

    // Bande passante 20 MHz
    esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);

    // Tenter de fixer le débit à 6 Mbps (peut échouer sur ESP-IDF 5.x)
    esp_err_t err = esp_wifi_internal_set_fix_rate(WIFI_IF_AP, true, WIFI_PHY_RATE_6M);
    if (err != ESP_OK) {
        Serial.printf("[WARN] fix_rate: %s (debit auto)\n", esp_err_to_name(err));
    }

    Serial.println("Emission en cours...\n");
}

// ─────────────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────────────

static uint32_t last_tx = 0;
static uint32_t tx_count = 0;

void loop() {
    uint32_t now = millis();
    if (now - last_tx < BEACON_INTERVAL) return;
    last_tx = now;

    double dt = BEACON_INTERVAL / 1000.0;
    tx_count++;
    Serial.printf("[#%lu]\n", tx_count);

    for (size_t i = 0; i < NUM_DRONES; i++) {
        update_position(drones[i], dt);
        send_beacon(drones[i]);
        led_flash(drones[i].color);

        double lat, lon;
        get_position(drones[i], lat, lon);
        Serial.printf("  %s  %.6f, %.6f  alt=%dm hdg=%03d\n",
                      drones[i].ssid, lat, lon,
                      (int)drones[i].altitude, get_heading(drones[i]));
    }
    Serial.println();
}
