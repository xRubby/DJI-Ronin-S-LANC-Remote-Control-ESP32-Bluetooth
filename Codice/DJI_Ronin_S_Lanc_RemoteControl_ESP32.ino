#include <Ronin_SBUS.h>
#include <Bluepad32.h>
#include "LANC_CAM_CONTROL.h"

// --- Controller ---
ControllerPtr myController = nullptr;

// --- Interfaccia S.BUS per Ronin ---
Ronin_SBUS ronin;
#define SBUS_PIN 14         // PIN ESP32

// --- PIN LANC ---
#define COMMAND_PIN 13      // Mando dati LANC
#define LANC_PIN 12         // Leggo dati LANC

// --- Interfaccia LANC ---
LANC_CAM_CONTROL cameraControl(COMMAND_PIN, LANC_PIN);

// --- Variabile per la gestione dello start/stop della registrazione ---
bool prevAState = false;

// --- Costanti S.BUS ---
const int sbusMID = 1024;   // Neutral val
const int sbusMIN = 352;    // Min usable for analog val and switch val
const int sbusMAX = 1696;   // Max usable for analog val and switch val
const int sbusWAIT = 10;    // Frame timing delay in msecs

// --- Variabili per la gestione della modalità di movimento del Ronin (veloce o lenta) ---
bool lowspeed=false;
bool prevBState = false;

// --- Callback quando un controller si connette ---
void onConnectedController(ControllerPtr ctl) {
  if (myController == nullptr) {
    Serial.println("CALLBACK: Controller is connected");
    myController = ctl;
  }
}

// --- Callback quando un controller si disconnette ---
void onDisconnectedController(ControllerPtr ctl) {
  if (myController == ctl) {
    Serial.println("CALLBACK: Controller disconnected");
    myController = nullptr;
  }
}

// --- Funzione di mapping da joystick (-500..500) a valori S.BUS ---
int16_t mapToSbus(int16_t value) {
  // map da -500/500 → sbMIN/sbMAX
  const int deadzone = 50;  // intervallo attorno allo zero considerato neutro

  // Se siamo nella zona morta, restituisci valore neutro
  if(abs(value) < deadzone) {
    return sbusMID;
  }

  // Mappatura lineare da -500..500 a sbusMIN..sbusMAX
  int16_t mapped;

  // Se è attiva la modalità lenta, i movimenti verranno eseguiti a velocità ridotta;
  // altrimenti verranno eseguiti a velocità normale.
  if(lowspeed){
    mapped = map(value/2, -500, 500, sbusMIN, sbusMAX);
  }else{
    mapped = map(value, -500, 500, sbusMIN, sbusMAX);
  }

  // Assicura che il valore non superi i limiti S.Bus
  if(mapped < sbusMIN) return sbusMIN;
  if(mapped > sbusMAX) return sbusMAX;

  return mapped;
}

// --- Funzione per la gestione della velocità di movimento del Ronin ---
void changeSpeed(){
  lowspeed = !lowspeed;

  Serial.println("Velocità impostata su " + String(lowspeed));
}

void zoomCamera(int16_t value){
  const int deadzone = 50;  // intervallo attorno allo zero considerato neutro

  // Se siamo nella zona morta, esco senza fare nulla
  if(abs(value) < deadzone) {
    return;
  }

  int16_t speed;

  // Mappa il valore del joystick alla velocità di zoom:
  // - Se > 0: zoom out (da 0 a 500 → velocità 0–7)
  // - Se < 0: zoom in  (da 0 a -500 → velocità 0–7)
  if(value>0){
    speed = map(value, 0, 500, 0, 7);
    cameraControl.zoomOut(speed);
  }else{
    speed = map(value, 0, -500, 0, 7);
    cameraControl.zoomIn(speed);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Inizializza Ronin S.Bus
  ronin.begin(SBUS_PIN);

  // Inizializza Bluepad32 con callback di connessione e disconnessione
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Resetta le chiavi Bluetooth precedentemente salvate
  // Utile se ci sono problemi di riconnessione
  BP32.forgetBluetoothKeys();
}

void loop() {

  // Aggiorna lo stato dei controller
  bool dataUpdated = BP32.update();

  // Se ci sono dati nuovi e il controller è connesso
  if(dataUpdated && myController != nullptr) {
    
    //Se viene premuto il tasto B (A per Switch Pro Controller) la velocità del Ronin viene cambiata
    bool currentBState = myController->b();

    if (currentBState && !prevBState) {
      changeSpeed();
    }

    // Aggiorna lo stato precedente del tasto B (A per Switch Pro Controller)
    prevBState = currentBState;

    //Se viene premuto il tasto A (B per Switch Pro Controller) 
    //viene avviata/stoppata la registrazione sulla camera
    bool currentAState = myController->a();

    if (currentAState && !prevAState) {
      cameraControl.startStopRecording();
    }

    // Aggiorna lo stato precedente del tasto A (B per Switch Pro Controller) 
    prevAState = currentAState;

    // Leggo il valore dell'asse Y del controller per controllare lo zoom della camera
    zoomCamera(myController->axisY());

    // Leggi gli assi del controller e mappa su valori S.Bus
    int16_t pan  = mapToSbus(myController->axisRX());
    int16_t tilt = mapToSbus(myController->axisRY());

    // Aggiorna i canali Ronin
    ronin.SetValue(1, pan); // Canale 1 → Pan
    ronin.SetValue(2, tilt); // Canale 2 → Tilt

    // Aggiorna e invia il frame S.Bus
    ronin.Update();
    ronin.Send();

    // Piccola pausa per evitare di saturare la comunicazione S.Bus
    delay(sbusWAIT);
  }
}
