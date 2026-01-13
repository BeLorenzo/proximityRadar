# ESP32 Sentinel: Hybrid Sensor Fusion Radar

Un sistema di sorveglianza attiva che combina la visione periferica degli ultrasuoni con la precisione di puntamento del laser.

## Obiettivo
Realizzare una "torretta sentinella" autonoma in grado di:
1.  **Scansionare** l'ambiente a 180° visualizzando una mappa radar.
2.  **Validare** i bersagli restringendo il campo visivo (Sector Search) per evitare falsi positivi.
3.  **Tracciare** l'oggetto in movimento con precisione una volta agganciato.
4.  **Allertare** tramite segnali visivi, sonori e notifiche Wi-Fi se la minaccia è imminente.

## La Logica Ibrida (FSM)
Il sistema opera su una Macchina a Stati Finiti avanzata per ottimizzare la risposta dei sensori:

### Fase 1: WIDE SCAN (Sorveglianza)
* **Sensore:** Ultrasuoni (HY-SRF05).
* **Comportamento:** Il servo scansiona l'intera area (0°-180°).
* **Visualizzazione:** L'OLED disegna i "blip" rilevati dal cono ampio degli ultrasuoni.
* **Trigger:** Se viene rilevato un ostacolo il sistema memorizza l'angolo approssimativo e passa alla Fase 2.

### Fase 2: SECTOR SEARCH (Ricerca & Aggancio)
* **Sensore:** Laser ToF (VL53L0X).
* **Comportamento:** Il servo limita il movimento a un settore ristretto e rallenta la velocità.
* **Scopo:** Permettere al raggio laser (molto stretto) di analizzare l'area per trovare l'esatta posizione dell'oggetto senza perderlo tra una lettura e l'altra.
* **Trigger:** Se il laser legge una distanza valida, passa alla Fase 3. Se non trova nulla dopo N secondi, torna in Fase 1.

### Fase 3: PRECISION TRACK & ALERT (Inseguimento)
* **Sensore:** Laser ToF (VL53L0X).
* **Comportamento:** Il servo segue l'oggetto in tempo reale cercando di mantenere il laser centrato sul bersaglio.
* **Allarme:** Se la distanza scende sotto la soglia critica -> **Buzzer Attivo + LED Rosso + Notifica Wi-Fi**.

## Licenza
Questo progetto è rilasciato sotto licenza **GNU General Public License v3.0 (GPLv3)**.
Sei libero di copiare, distribuire e modificare il software, a patto che ogni modifica venga rilasciata sotto la stessa licenza. Vedi il file `LICENSE` per i dettagli legali completi.
