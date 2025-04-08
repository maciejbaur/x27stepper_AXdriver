// BIBLIOTEKI
#include <Wire.h>                    // komunikacja po I2C z ekspanderami
#include "EU07_MEGA_x27_test_CFG.h"  // Plik konfiguracyjny - definicje expanderow, stalych i zmiennych

void setup() {
  // Obsługa komunikacji szeregowej
  Serial.begin(115200);   // 115200b/s dla PC
  Serial.setTimeout(10);
  while (!Serial) {};

  digitalWrite(RESET, HIGH);

  pinMode(pinZero,INPUT_PULLUP);
  pinMode(pinTest,INPUT_PULLUP);

  zerowanie_manometrow();
  
} // KONIEC SETUP'a


void loop() {

  // --- ODCZYT STANU PINOW --- //
  // Funkcja odczytujaca stan pinow i wpisujaca je do zmiennych
  odczyt_stanu_pinow();

  // --- OBSLUGA URZĄDZEN --- //

    // WYBIERZ WLASCIWA FUNKCJE ZALEZNIE OD RODZAJU POLACZENIA
    manometry_AX(); // uaktywnij przy komunikacji przez AX
    //manometry_MEGA(); // uaktywnij przy komunikacji bezposrednio z Arduino

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
  zPC_Pre_0 = zPC[0];      // Bajt 0 - preambula
  zPC_Pre_1 = zPC[1];      // Bajt 1 - preambula
  zPC_Pre_2 = zPC[2];      // Bajt 2 - preambula
  zPC_Pre_3 = zPC[3];      // Bajt 3 - preambula


  zPC_BrakePress = zPC[11];                   // Ciśnienie w cylindrze hamulcowym
  zPC_BrakePress1 = zPC[12];                   // Ciśnienie w cylindrze hamulcowym
  BrakePress = (zPC_BrakePress << 0) | (zPC_BrakePress1 << 8); // sklejone bity expanderow 0 i 1

  zPC_PipePress = zPC[13];                    // Ciśnienie w przewodzie glownym
  zPC_PipePress1 = zPC[14];                    // Ciśnienie w przewodzie glownym
  PipePress = (zPC_PipePress << 0) | (zPC_PipePress1 << 8); // sklejone bity expanderow 0 i 1

  zPC_TankPress = zPC[15];                    // Ciśnienie w zbiorniku glownym
  zPC_TankPress1 = zPC[16];                    // Ciśnienie w zbiorniku glownym
  TankPress = (zPC_TankPress << 0) | (zPC_TankPress1 << 8); // sklejone bity expanderow 0 i 1

}

void zapis_stanu_pinow()  {
  // Przypisanie wartosci zmiennych do bitow ramki doPC

  doPC[0] = 0xEF;       // Bajt 0 - preambula
  doPC[1] = 0xEF;       // Bajt 1 - preambula
  doPC[2] = 0xEF;       // Bajt 2 - preambula
  doPC[3] = 0xEF;       // Bajt 3 - preambula

}

// funkcja komunikacji przez AX
void manometry_AX()  {
  motor1.setPosition(map(BrakePress, 0, 1023, 0, 3274));  // <wartosc maksymalna przy pelnym zakresie> * 3780 / 1023
  motor1.update();
  motor2.setPosition(map(PipePress, 0, 1023, 0, 3274));   // <wartosc maksymalna przy pelnym zakresie> * 3780 / 1023
  motor2.update();
  motor3.setPosition(map(TankPress, 0, 1023, 0, 3274));   // <wartosc maksymalna przy pelnym zakresie> * 3780 / 1023
  motor3.update();
}

// funkcja komunikacji bezposrednio z Arduino
void manometry_MEGA()  {
  motor1.setPosition(map(BrakePress, 0, 1023, 0, 1635));  // <wartosc maksymalna przy pelnym zakresie> * 1890 / 1023
  motor1.update();
  motor2.setPosition(map(PipePress, 0, 1023, 0, 1635));   // <wartosc maksymalna przy pelnym zakresie> * 1890 / 1023
  motor2.update();
  motor3.setPosition(map(TankPress, 0, 1023, 0, 1635));   // <wartosc maksymalna przy pelnym zakresie> * 1890 / 1023
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
  motor1.setPosition(1635);
    motor1.update();
  motor2.setPosition(1635);
    motor2.update();
  motor3.setPosition(1635);
    motor3.update();
}
