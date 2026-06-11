# ⚓ AnchorWatch — ESP8266 Ankervagt med GPS

En ankervagt til båd baseret på ESP8266 og GPS-modul. Overvåger om båden driver fra ankerpositionen og alarmerer med buzzer + webinterface.

## Funktioner

- **GPS-overvågning** — løbende positionsberegning med Haversine-formel
- **Konfigurerbar radius** — standard 30m, justerbar 5–500m
- **Buzzer-alarm** — lyder når båden driver ud over radius
- **WiFi webinterface** — se status og styr fra telefonen
- **Fysisk knap** — sæt anker / stil alarm

## Hardware

| Komponent | Eksempel |
|-----------|----------|
| ESP8266MOD | NodeMCU, Wemos D1 Mini, etc. |
| GPS modul | NEO-6M, NEO-7M, BN-220, etc. |
| Buzzer | Aktiv buzzer 3.3V |
| Trykknap | Normalt åben |

## Tilslutning

```
ESP8266          GPS Modul
───────          ─────────
D5 (GPIO14) ◄── TX
D6 (GPIO12) ──► RX
3.3V        ──► VCC
GND         ──► GND

ESP8266          Buzzer
───────          ──────
D7 (GPIO13) ──► + (signal)
GND         ──► - (GND)

ESP8266          Knap
───────          ────
D3 (GPIO0)  ──► ene ben
GND         ──► andet ben
```

> **OBS:** GPS VCC kan være 3.3V eller 5V afhængig af modul — tjek databladet. De fleste NEO-6M moduler har en spændingsregulator og kan køre på 5V fra VIN.

## Installation

1. Installer **PlatformIO IDE** extension i VS Code
2. Åbn dette projekt i VS Code
3. Tilslut ESP8266 via USB
4. Klik **Upload** (→) i PlatformIO toolbar
5. Åbn Serial Monitor for at se status

## Brug

1. ESP8266 starter et WiFi access point: **AnchorWatch** (kode: `ankervagt`)
2. Forbind telefon til WiFi og gå til **192.168.4.1**
3. Vent på GPS-fix (LED blinker)
4. Tryk **Sæt Anker Her** på webinterface eller tryk på den fysiske knap
5. Alarmen lyder hvis båden driver ud over den indstillede radius
6. Tryk knap eller **Stil Alarm** for at slukke alarm midlertidigt