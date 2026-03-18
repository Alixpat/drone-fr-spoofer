# drone-fr-spoofer

Simulateur de **signalement électronique français** (arrêté du 27 décembre 2019) pour **M5Stack Stamp S3** (ESP32-S3).

Génère des trames beacon Wi-Fi conformes à la norme française (OUI `0x6A5C35`, canal 6, format TLV) avec des positions simulées. Aucun GPS nécessaire.

## Build & Flash

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install platformio

pio run -t upload
pio device monitor
```

## Configuration

Modifier le tableau `drones[]` dans `src/main.cpp` :

- `id` : identifiant AlphaTango (30 chars max)
- `ssid` : nom du réseau Wi-Fi beacon
- `color` : couleur LED RGB `{R, G, B}` (0-255)
- `home_lat/lon` : coordonnées de décollage
- `center_lat/lon` : centre de la trajectoire circulaire
- `radius_deg` : rayon en degrés (~0.001° ≈ 111m)
- `altitude` : altitude MSL en mètres
- `height` : hauteur sol en mètres
- `speed` : vitesse sol en m/s
- `angular_speed` : vitesse angulaire en °/s (négatif = sens anti-horaire)

## Vérification

- **ReceptionInfoDrone** (script Python) : https://github.com/GendarmerieNationale/ReceptionInfoDrone
- **Wireshark** avec un adaptateur en mode monitor, filtre : `wlan.tag.oui == 6a:5c:35`

## Crédits

- Librairie `droneID_FR` par **Pierre Kancir** / Airbot Systems (GPL v3) : https://github.com/khancyr/droneID_FR

## Licence

GPL v3 (conformément à la licence de droneID_FR)
