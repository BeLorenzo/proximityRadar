# ESP32 Sentinel: Hybrid Sensor Fusion Radar

Un sistema di sorveglianza attiva che combina la visione periferica degli ultrasuoni con la precisione di puntamento del laser.

## Obiettivo
Realizzare una "torretta sentinella" autonoma in grado di:
1.  **Scansionare** l'ambiente a 180° visualizzando una mappa radar.
2.  **Validare** i bersagli restringendo il campo visivo (Sector Search) per evitare falsi positivi.
3.  **Tracciare** l'oggetto in movimento con precisione una volta agganciato.
4.  **Allertare** tramite segnali visivi, sonori e notifiche Wi-Fi se la minaccia è imminente.

## La Logica Ibrida (Priority Sensor Fusion)
Il sistema non usa una semplice media, ma una **Fusione a Priorità** basata sulla fisica dei sensori.

**Fase 1: WIDE SCAN (Sorveglianza Globale)**

* **Sensori:** Laser (VL53L0X) + Ultrasuoni (Backup).
* **Comportamento:** Il servo scansiona 0°-180°.
* **Logica:** Il Laser ha priorità assoluta per la precisione. Se il raggio laser (troppo stretto) manca il bersaglio, l'Ultrasuono (cono ampio) rileva la presenza e suggerisce la zona.
* **Filtro:** Algoritmo **"Rule of Three"**: un bersaglio viene considerato reale solo se rilevato per 3 cicli di loop consecutivi (elimina ghosting e glitch elettrici).

**Fase 2: TRACKING LOCK (Aggancio Dinamico)**

* **Trigger:** Rilevamento confermato < 80cm.
* **Comportamento:** Il servo abbandona la scansione completa e si blocca in un settore ristretto (`Track Width` ±25°) attorno all'ultimo punto noto.
* **Scopo:** Aumentare la frequenza di aggiornamento sul bersaglio (sampling rate più alto) per non perderlo se si muove.

**Fase 3: DANGER & ALERT (Minaccia)**

* **Trigger:** Distanza < 40cm (Zona Rossa).
* **Azione:**
* Feedback Locale: LED Rosso + Buzzer intermittente.
* Feedback Remoto: Invio notifica **Telegram** tramite Wi-Fi (Core 0 asincrono) con dati di distanza e angolo.

---

###  Hardware & Pinout (ESP32)

| Componente | Pin ESP32 | Note |
| --- | --- | --- |
| **Servo** | GPIO 17 | Libreria ServoEasing |
| **Laser (SDA)** | GPIO 21 | I2C |
| **Laser (SCL)** | GPIO 22 | I2C |
| **Ultrasuoni (Trig)** | GPIO 15 |  |
| **Ultrasuoni (Echo)** | GPIO 23 |  |
| **Buzzer** | GPIO 4 | Attivo Alto |
| **LED Rosso** | GPIO 13 | Danger |
| **LED Giallo** | GPIO 27 | Tracking |
| **LED Verde** | GPIO 33 | Scanning |

###  Librerie Richieste

* `Adafruit_GFX` & `Adafruit_SH110X` (Display)
* `Adafruit_VL53L0X` (Laser)
* `NewPing` (Sonar)
* `ServoEasing` (Movimento fluido)

---

## Configurazione

Il progetto richiede un file `secrets.h` nella cartella principale per gestire le credenziali sensibili.
Crea un nuovo file chiamato `secrets.h` accanto al file `.ino` e incolla questo codice:

```cpp
// WiFi Credentials
#define SECRET_WIFI_SSID "IlTuoSSID"
#define SECRET_WIFI_PASS "LaTuaPassword"

// Telegram CallMeBot
#define TELEGRAM_USERNAME "@tuo_username" 
```
---

## Licenza
Questo progetto è rilasciato sotto licenza **GNU General Public License v3.0 (GPLv3)**.
Sei libero di copiare, distribuire e modificare il software, a patto che ogni modifica venga rilasciata sotto la stessa licenza. Vedi il file `LICENSE` per i dettagli legali completi.
