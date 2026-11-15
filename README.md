# drone-fr-spoofer

Simulateur de **signalement électronique français** (arrêté du 27 décembre 2019) pour **M5Stack Stamp S3** (ESP32-S3).

Génère des trames beacon Wi-Fi conformes à la norme française (OUI `0x6A5C35`, canal 6, format TLV) avec des positions simulées. Aucun GPS nécessaire.

Les beacons partent en 802.11b DSSS à 1 Mbit/s, préambule long, à faible puissance (2 dBm par défaut). Le numéro de séquence 802.11 compte les beacons de chaque drone : un récepteur sait ainsi combien il en a manqué. Les identifiants par défaut (`TST000ESSAI...`) et les adresses MAC locales (`02:52:49:44:...`) disent clairement qu'il s'agit d'essais.

Certains drones émettent leurs balises en 802.11g OFDM à 6 Mbit/s. L'environnement `m5stack-stamps3-ofdm` émet les mêmes beacons de cette façon. L'environnement `m5stack-stamps3-mixed` alterne : le premier drone en DSSS, le second en OFDM, pour essayer un récepteur dans les deux cas avec une seule capture. La radio redémarre alors avant chaque beacon, le débit ne se réglant qu'interface arrêtée.

Sert d'émetteur d'essai à [gr-remoteid](https://github.com/Alixpat/gr-remoteid), qui reçoit ces beacons avec une radio logicielle.

## Build & Flash

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install platformio

pio run -t upload                           # 802.11b DSSS, 1 Mbit/s
pio run -e m5stack-stamps3-ofdm -t upload   # 802.11g OFDM, 6 Mbit/s
pio run -e m5stack-stamps3-mixed -t upload  # un drone sur deux dans chaque débit
pio device monitor
```

## Configuration

Modifier le tableau `drones[]` dans `src/main.cpp` :

- `id` : identifiant AlphaTango (30 chars exactement)
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

- **gr-remoteid** : `hackrf_transfer -r essai.cs8 -f 2437000000 -s 20000000 -l 24 -g 20 -n 160000000`, puis `remoteid_rx.py --file essai.cs8 --summary`
- **ReceptionInfoDrone** (script Python) : https://github.com/GendarmerieNationale/ReceptionInfoDrone
- **Wireshark** avec un adaptateur en mode monitor, filtre : `wlan.tag.oui == 6a:5c:35`

## Crédits

- Librairie `droneID_FR` par **Pierre Kancir** / Airbot Systems (GPL v3) : https://github.com/khancyr/droneID_FR

## Licence

GPL v3 (conformément à la licence de droneID_FR)
