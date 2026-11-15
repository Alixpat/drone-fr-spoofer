# AGENTS.md

Émetteur d'essai du signalement électronique français (arrêté du 27 décembre 2019) sur
ESP32-S3. Il simule des drones en envoyant des beacons Wi-Fi, pour mettre au point un
récepteur, par exemple [gr-remoteid](https://github.com/Alixpat/gr-remoteid).

## Règles

1. C'est un outil d'essai. Les identifiants par défaut restent marqués comme tels
   (`TST000ESSAI...`, adresses MAC locales `02:52:49:44:...`) : ne pas imiter un vrai drone.
2. Puissance faible par défaut (`TX_POWER_DBM`, 2 dBm).
3. Émission en 802.11b DSSS à 1 Mbit/s, préambule long, canal 6. L'environnement
   `m5stack-stamps3-ofdm` émet les mêmes beacons en 802.11g OFDM à 6 Mbit/s, et
   `m5stack-stamps3-mixed` un drone sur deux dans chaque débit. Ne pas changer un débit sans
   vérifier la réception.
4. Le numéro de séquence 802.11 compte les beacons de chaque drone. Le garder : il sert à
   mesurer les pertes côté récepteur.
5. `src/droneID_FR.h` vient de Pierre Kancir (GPL v3). Ne pas le modifier sans raison.
6. Textes courts, en français, sans tiret cadratin.

## Commandes

PlatformIO s'installe dans un environnement virtuel, voir le README.

```bash
pio run              # compile
pio run -t upload    # écrit sur la carte
pio run -e m5stack-stamps3-ofdm -t upload   # variante OFDM à 6 Mbit/s
pio run -e m5stack-stamps3-mixed -t upload  # un drone sur deux dans chaque débit
pio device monitor   # lit le port série
```

La première compilation télécharge la chaîne d'outils : il faut un accès à Internet.

Le port série affiche une ligne par beacon : drone, numéro de séquence, position.

## Où lire

- `src/main.cpp` : drones simulés, construction et envoi des beacons, réglages en tête de
  fichier (canal, intervalle, puissance).
- `src/droneID_FR.h` : format du signalement français.
- `README.md` : installation, configuration des drones, vérification de la réception.
