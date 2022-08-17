// W trakcie testow



// BIBLIOTEKI
//#include "PCF8574.h"               // https://drive.google.com/file/d/0B8JV5ebfS8ttU0pTUTJnUVpYOHM/view
#include <Wire.h>                    // komunikacja po I2C z ekspanderami
#include "EU07_MEGA_x27_test_CFG.h"  // Pliik konfiguracyjny - definicje expanderow, stalych i zmiennych

void setup() {
  // Obsługa komunikacji szeregowej
  Serial.begin(115200);   // 115200b/s dla PC
  Serial.setTimeout(10);
  while (!Serial) {};

// Ustawienia w bibliotece SwitecX12.cpp
//
//  static unsigned short defaultAccelTable[][2] = {
//  {   10, 10000},
//  {   50,  7000},
//  {  150,  3000},
//  {  300,  1500},
//  {  500,   900}
//  };
//
//  const int resetStepMicrosec = 900;  // default 300

  // wymuszenie powrotu wskazowek manometrow do pozycji zerowej (docelowo po przycisnieciu jakiegos przycisku, zeby nie robil tego niepotrzebnie przy kazdym resecie)
  //zerowanie_manometrow();
  
  // run the motor against the stops
//  motor1.zero();
//  motor2.zero();
//  motor3.zero();

  digitalWrite(RESET, HIGH);

  pinMode(pinZero,INPUT_PULLUP);
  pinMode(pinTest,INPUT_PULLUP);
  
} // KONIEC SETUP'a


void loop() {


  // --- ODCZYT STANU PINOW --- //
  // Funkcja odczytujaca stan pinow i wpisujaca je do zmiennych
  odczyt_stanu_pinow();

  // --- OBSLUGA URZĄDZEN --- //

    // Funkcja obslugujaca manometry
    //manometry();
    manometry_x25();
    while (digitalRead(pinZero) == LOW) {
      zerowanie_manometrow();
    }
    while (digitalRead(pinTest) == LOW) {
      test_manometrow();
    }

  // --- ZAPIS STANU PINOW --- //
  // Funkcja zapisujaca stan pinow ze zmiennych i wpisujaca je do bitow ramki doPC
  zapis_stanu_pinow();

  // --- KOMUNIKACJA UART --- //
  if (Serial.available()) {
    komunikacja_z_exe();  // Funkcja obslugujaca komunikacje z PC poprzez UART
  }

} // KONIEC LOOP'a



// FUNKCJE OBSLUGI URZADZEN PULPITU

void komunikacja_z_exe() {
  // Wysylanie i odbieranie danych zPC/doPC poprzez komunikacje szeregowa USB
  while (!Serial.available()) {};
  Serial.readBytes((char*)zPC, zPC_byteLength);
  Serial.write((char*)doPC, doPC_byteLength);
}


void odczyt_stanu_pinow()  {
  // Przypisanie danych z bitow ramki zPC do zmiennych
  // Jesli zmieni sie ramka albo przypisania bitow to mozna tutaj zmienic
  zPC_Pre_0 = zPC[0];      // Bajt 0 - preambula
  zPC_Pre_1 = zPC[1];      // Bajt 1 - preambula
  zPC_Pre_2 = zPC[2];      // Bajt 2 - preambula
  zPC_Pre_3 = zPC[3];      // Bajt 3 - preambula

  zPC_Tacho = zPC[4];                         // predkosc dla falownika

  zPC_LWO = !bitRead(zPC[6], 1);              // LWO - lampka wentylatorow oporow
  zPC_LWR = !bitRead(zPC[6], 2);              // LWR - lampka rysokiego rozruchu
  zPC_LOP = !bitRead(zPC[8], 0);              // LOP - lampka ogrzewania pociągu
  zPC_LJO = !bitRead(zPC[8], 1);              // LJO - lampka jazdy na oporach rozruchowych
  zPC_LP  = !bitRead(zPC[8], 2);              // LP - lampka poślizgu
  zPC_LCA = !bitRead(zPC[8], 6);              // LCA - lampka CA
  zPC_LSHP = !bitRead(zPC[8], 7);             // LSHP - lampka SHP
  zPC_LSL = !bitRead(zPC[9], 0);              // LSL - lampka styczników liniowych
  zPC_LNP = !bitRead(zPC[9], 2);              // LNP - lampka przekaźnika nadmiarowego przetwornicy
  zPC_LOG = !bitRead(zPC[9], 3);              // LOG - lampka przekaznika roznicowego obwodu glownego
  zPC_LNS = !bitRead(zPC[9], 4);              // LNS - lampka przekaźnika nadmiarowego silników trakcyjnych
  zPC_LWS = !bitRead(zPC[9], 5);              // LWS - lampka WS

  zPC_Rysik_1 = bitRead(zPC[10], 2);          // Hasler - rysik 1 - kabina A/B
  zPC_Rysik_2 = bitRead(zPC[10], 3);          // Hasler - rysik 2 - hamowanie
  zPC_Rysik_3 = !bitRead(zPC[10], 4);         // Hasler - rysik 3 - jazda pod pradem

  //zPC_SpringBrake = !bitRead(zPC[10], 6);     // hamulec sprezynowy (nieoprogramowany)
  zPC_AlerterSound = !bitRead(zPC[10], 7);    // Sygnal dzwiekowy - buczek - CA/SHP

  zPC_BrakePress = zPC[11];                   // Ciśnienie w cylindrze hamulcowym
  zPC_BrakePress1 = zPC[12];                   // Ciśnienie w cylindrze hamulcowym
  BrakePress = (zPC_BrakePress << 0) | (zPC_BrakePress1 << 8); // sklejone bity expanderow 0 i 1

  zPC_PipePress = zPC[13];                    // Ciśnienie w przewodzie glownym
  zPC_PipePress1 = zPC[14];                    // Ciśnienie w przewodzie glownym
  PipePress = (zPC_PipePress << 0) | (zPC_PipePress1 << 8); // sklejone bity expanderow 0 i 1

  zPC_TankPress = zPC[15];                    // Ciśnienie w zbiorniku glownym
  zPC_TankPress1 = zPC[16];                    // Ciśnienie w zbiorniku glownym
  TankPress = (zPC_TankPress << 0) | (zPC_TankPress1 << 8); // sklejone bity expanderow 0 i 1

  zPC_HVoltage = zPC[17];                     // Woltomierz wysokiego napiecia
  zPC_HCurrent_1 = zPC[19];                   // Amperomierz pierwszej grupy silnikow
  //zPC_HCurrent_1a = zPC[20];                  // Amperomierz pierwszej grupy silnikow - drugi bajt dla wersji 16 bitowej
  zPC_HCurrent_2 = zPC[21];                   // Amperomierz drugiej grupy silnikow
  //zPC_HCurrent_2a = zPC[22];                  // Amperomierz pierwszej grupy silnikow - drugi bajt dla wersji 16 bitowej

  //zPC_LVoltage = zPC[35];                   // Woltomierz niskiego napiecia (obslugiwane w exe ale nie uzywane w kodzie)
  //zPC_LCurrent = zPC[xx];                   // Amperomierz niskiego napiecia (nie obslugiwane w exe)

  zPC_RadioChannel = zPC[37];                 // Kanal radiowy
  zPC_RadioStop = bitRead(zPC[10], 5);       // RadioStop
}

void zapis_stanu_pinow()  {
  // Przypisanie wartosci zmiennych do bitow ramki doPC
  // Jesli zmieni sie ramka albo przypisania bitow to mozna tutaj zmienic

  doPC[0] = 0xEF;       // Bajt 0 - preambula
  doPC[1] = 0xEF;       // Bajt 1 - preambula
  doPC[2] = 0xEF;       // Bajt 2 - preambula
  doPC[3] = 0xEF;       // Bajt 3 - preambula

  bitWrite(doPC[4], 1, doPC_LineBrakerOpen);                     // Przycisk - wylaczenie WSa
  bitWrite(doPC[4], 2, doPC_LineBrakerClose);                    // Przycisk - zalaczenie WSa
  bitWrite(doPC[4], 3, doPC_MotorOverloadRelayReset);            // Przycisk - odblok. przekaźników nadmiarowych silników trakcyjnych i różnicowego obwodu głónego
  bitWrite(doPC[4], 4, doPC_CompressorOverloadRelayReset);       // Przycisk - odblok. przekaznika sprezarki i wentylatorow oporow
  bitWrite(doPC[4], 5, doPC_ConverterOverloadRelayReset);        // Przycisk - odblok. przekaznikow nadmiarowych przetwornicy, ogrzewania pociągu i różnicowych obw. pomocniczych
  bitWrite(doPC[4], 6, doPC_MotorConnectorsOpen);                // Przycisk - wylacznik stycznikow liniowych
  bitWrite(doPC[4], 7, doPC_AlerterAcknowledge);                 // Przycisk - kasowanie CA i SHP
  bitWrite(doPC[5], 0, doPC_BatteryEnable);                      // Hebelek - bateria akumulatorów
  bitWrite(doPC[5], 1, doPC_ConverterEnable);                    // Hebelek - przetwornica
  bitWrite(doPC[5], 2, doPC_CompressorEnable);                   // Hebelek - sprezarka
  bitWrite(doPC[5], 3, doPC_SandboxActivate);                    // Przycisk - piasecznica
  bitWrite(doPC[5], 4, doPC_HeatingEnable);                      // Hebelek - ogrzewanie pociagu
  bitWrite(doPC[5], 5, doPC_PantographCompressorActivate);       // Hebelek - aktywacja sprezarki pomocniczej
  bitWrite(doPC[5], 6, doPC_PantographCompressorValveEnable);    // Hebelek - przelaczenie stanu korka trojdrogowego
  bitWrite(doPC[5], 7, doPC_MotorOverloadRelayThresholdSetLow);  // Hebelek - przełącznik pakietowy wysokiego rozruchu
  bitWrite(doPC[6], 0, doPC_PantographRaiseFront);               // Hebelek - pantograf przedni
  bitWrite(doPC[6], 1, doPC_PantographRaiseRear);                // Hebelek - pantograf tylny
  bitWrite(doPC[6], 2, doPC_WheelSpinBrakeActivate);             // Przycisk - przyhamowanie przy poślizgu
  bitWrite(doPC[6], 3, doPC_HeadLightsDimEnable);                // Hebelek - przyciemnianie reflektorów
  bitWrite(doPC[6], 4, doPC_InteriorLightDimEnable);             // Hebelek - przyciemnianie oświetlenia kabiny
  bitWrite(doPC[6], 5, doPC_IndependentBrakeBailOff);            // Przycisk - odluzniacz
  bitWrite(doPC[6], 6, doPC_HornHighActivate);                   // Przycisk - syrena - ton wysoki
  bitWrite(doPC[6], 7, doPC_HornLowActivate);                    // Przycisk - syrena - ton niski

  bitWrite(doPC[7], 1, doPC_HeadLightEnableLeft);                // Hebelek - reflektor lewy
  bitWrite(doPC[7], 2, doPC_HeadLightEnableUpper);               // Hebelek - reflektor gorny
  bitWrite(doPC[7], 3, doPC_HeadLightEnableRight);               // Hebelek - reflektor prawy
  bitWrite(doPC[7], 4, doPC_RedMarkerEnableLeft);                // Hebelek - sygnał czerwony lewy
  bitWrite(doPC[7], 5, doPC_RedMarkerEnableRight);               // Hebelek - sygnał czerwony prawy
  bitWrite(doPC[7], 6, doPC_InteriorLightEnable);                // Hebelek - oświetlenie kabiny

  bitWrite(doPC[8], 0, doPC_ReverserForward);                    // Przelacznik - jazda do przodu
  bitWrite(doPC[8], 1, doPC_ReverserNeutral);                    // Przelacznik - pozycja neutralna nastawnika kierunku
  bitWrite(doPC[8], 2, doPC_ReverserBackward);                   // Przelacznik - jazda do tylu
  bitWrite(doPC[8], 3, doPC_BrakeActingSpeedSetCargo);           // Przelacznik - tryb hamulca dla pociągu towarowego
  bitWrite(doPC[8], 4, doPC_BrakeActingSpeedSetPassenger);       // Przelacznik - tryb hamulca dla pociągu osobowego
  bitWrite(doPC[8], 5, doPC_BrakeActingSpeedSetRapid);           // Przelacznik - tryb hamulca dla pociągu pospiecznego
  bitWrite(doPC[8], 6, doPC_CabChangeForward);                   // Przycisk - przejscie do sasiedniego pomieszczenia w kierunku czola pociagu
  bitWrite(doPC[8], 7, doPC_CabChangeBackward);                  // Przycisk - przejscie do sasiedniego pomieszczenia w kierunku tyłu pociagu

  bitWrite(doPC[9], 0, doPC_RadioToggle);                        // Przycisk - przelaczenie stanu radiotelefonu
  bitWrite(doPC[9], 1, doPC_RadioChannelIncrease);               // Przycisk - wybor kolejnego kanalu radia
  bitWrite(doPC[9], 2, doPC_RadioChannelDecrease);               // Przycisk - wybor poprzedniego kanalu radia
  bitWrite(doPC[9], 3, doPC_RadioStopSend);                      // Przycisk - emisja radiostopu
  bitWrite(doPC[9], 4, doPC_RadioStopTest);                      // Przycisk - lokalny test radiostopu
  bitWrite(doPC[9], 5, doPC_RadioCall3Send);                     // przycisk wywolania ZEW3
  bitWrite(doPC[9], 6, doPC_RadioVolumeIncrease);                // przycisk zwiekszenia glosnosci
  bitWrite(doPC[9], 7, doPC_RadioVolumeDecrease);                // przycisk zmniejszenia glosnosci

  doPC[10] = doPC_MasterController;                              // Pozycja nastawnika jazdy
  doPC[11] = doPC_SecondController;                              // Pozycja nastawnika oslabienia pola (bocznika)
  doPC[12] = doPC_TrainBrake;                                    // Pozycja kranu hamulca zespolonego
  doPC[14] = doPC_IndependentBrake;                              // Pozycja kranu hamulca pomocniczego
}


void manometry()  {
  /*
  if (exp9.digitalRead(pinForceAdjust) == true) { // manometry dzialaja normalnie gdy przelacznik dostosowania sil jest wylaczony, po zalaczeniu wskazania manometrow spadaja na minimum
    // Cylinder hamulcowy
    MCH = zPC_BrakePress; // wartosc odczytana z MaSzyny
    // kalibracja wielomianem
    MCHcal = byte ((MCHcal_4 * pow(MCH, 4)) + (MCHcal_3 * pow(MCH, 3)) + (MCHcal_2 * pow(MCH, 2)) + (MCHcal_1 * MCH) + MCHcal_0);
    analogWrite(pinMCH, MCHcal); // wartosc kalibrowana wielomianem

    // Przewod glowny
    MPG = zPC_PipePress; // wartosc odczytana z MaSzyny
    // kalibracja wielomianem
    MPGcal = byte ((MPGcal_4 * pow(MPG, 4)) + (MPGcal_3 * pow(MPG, 3)) + (MPGcal_2 * pow(MPG, 2)) + (MPGcal_1 * MPG) + MPGcal_0);
    analogWrite(pinMPG, MPGcal); // wartosc kalibrowana wielomianem

    // Zbiornik glowny
    MZG = zPC_TankPress; // wartosc odczytana z MaSzyny
    // kalibracja wielomianem
    MZGcal = byte ((MZGcal_4 * pow(MZG, 4)) + (MZGcal_3 * pow(MZG, 3)) + (MZGcal_2 * pow(MZG, 2)) + (MZGcal_1 * MZG) + MZGcal_0);
    analogWrite(pinMZG, MZGcal); // wartosc kalibrowana wielomianem
  }
  */
}

void manometry_x25()  {
  motor1.setPosition(map(BrakePress, 0, 1023, 0, 3126));  // przewod glowny X * 3780 / 1023
  motor1.update();
  motor2.setPosition(map(PipePress, 0, 1023, 0, 3126));   // przewod glowny X * 3780 / 1023
  motor2.update();
  motor3.setPosition(map(TankPress, 0, 1023, 0, 3318));   // przewod glowny X * 3780 / 1023
  motor3.update();
}

void zerowanie_manometrow() {
  // zerowanie wskazan manometrow
  motor1.zero();
  motor2.zero();
  motor3.zero();
}

void test_manometrow() {
  // ustawienie maksymalnych wartosci na skalach
  motor1.setPosition(3126);
    motor1.update();
  motor2.setPosition(3126);
    motor2.update();
  motor3.setPosition(3318);
    motor3.update();
}
