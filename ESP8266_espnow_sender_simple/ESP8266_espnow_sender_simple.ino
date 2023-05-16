#include "AUTOpairing.h"

//-----------------------------------------------------------
void setup() {

  // Init Serial Monitor
  Serial.begin(115200);
  AUTOpairing::channel = 6;  // canal donde empieza el scaneo
  AUTOpairing::timeOut=3000; // tiempo máximo
  AUTOpairing::SettimeOut=true; // tiempo máximo activo
  AUTOpairing::debug=true;   // depuración, inicializar Serial antes
  AUTOpairing::segundos_en_deepSleep= 10;  //tiempo dormido en segundos
   
  AUTOpairing::begin();
}

//-----------------------------------------------------------
void loop() {
  AUTOpairing::mantener_conexion();
  
  if (AUTOpairing::envio_disponible() ) { 
      char mensaje[256];
      sprintf(mensaje, "{\"topic\":\"datos\",\"temp\":%4.2f, \"hum\":%4.2f }", 24.2, 46.0);
      AUTOpairing::espnow_send(mensaje); // hará deepsleep por defecto
  }
}
