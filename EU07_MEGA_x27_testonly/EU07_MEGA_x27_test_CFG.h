// Definicje expanderow, stalych i zmiennych dla programu z pliku "EU07_sterowanie_COM_MEGA_v2.ino"

/*  // ekspandery 0-6 interfejsu I2CIN - interfejsu nastawników
    PCF8574 exp0;
    PCF8574 exp1;
    PCF8574 exp2;
    PCF8574 exp3;
    PCF8574 exp4;
    PCF8574 exp5;
    PCF8574 exp6;
    
  // ekspandery 7-8 interfejsy niezdefiniowane
    //PCF8574 exp7; // interfejs x - adres zajmowany przez LCD
    //PCF8574 exp8; // interfejs x

  // ekspandery 9-11 interfejsu I2CIPD - interfejsu panelu dolnego
    PCF8574 exp9;
    PCF8574 exp10;
    PCF8574 exp11;

  // ekspandery 12-13 interfejsu I2CILK - interfejsu lampek kontrolnych
    PCF8574 exp12;
    PCF8574 exp13;

  // ekspander 14 interfejsu I2CIPM - interfejsu przedziału maszynowego
    PCF8574 exp14;

  // ekspander 15 interfejs niezdefiniowany
    //PCF8574 exp15;
*/

  // Obsluga silnikow krokowych x25
  // #include <SwitecX12.h>
  #include <SwitecX25.h> // nowa biblioteka od @kaczor112

    
  // standard X25.168 range 315 degrees at 1/3 degree steps
    //const int STEPS (315*3);

  // x27.168 przez AX driver
    // const int STEPS (315*12);
    const int STEPS (315*6); // bezposrednio z Arduino i zmodyfikowana biblioteka
    const int A_STEP = 4;
    const int A_DIR = 5;
    const int B_STEP = 6;
    const int B_DIR = 7;
    const int C_STEP = 8;
    const int C_DIR = 9;
    const int RESET = 10;
    
    //  SwitecX12 motor1(STEPS, C_STEP, C_DIR);
    //  SwitecX12 motor2(STEPS, B_STEP, B_DIR);
    //  SwitecX12 motor3(STEPS, A_STEP, A_DIR);

  // For motors connected to digital pins 4,5,6,7
    SwitecX25 motor1(STEPS,22,23,24,25); // brake press
    SwitecX25 motor2(STEPS,4,5,6,7); // main pipe
    SwitecX25 motor3(STEPS,8,9,10,11); // main tank

  // Definicja stalych dla pinow wejscia / wyjscia

    const byte pinZero = 3; // do uruchomienia zerowania manometrow
    const byte pinTest = 2; // do uruchomienia testu manometrow
    
    //Plytka rozszerzen ACB (Arduino Connection Board)
    const byte pinMCH = 2;                      // ACB J8 - manometr cylindra hamulcowego
    const byte pinMPG = 3;                      // ACB J9 - manometr przewodu glownego
    const byte pinMZG = 4;                      // ACB J10 - manometr zbiornika glownego
    const byte pinHVoltage = 5;                 // ACB J5 - woltomierz napiecia trakcyjnego
    const byte pinHCurrent_1 = 6;               // ACB J6 - amperomierz pierwszej grupy silnikow
    const byte pinHCurrent_2 = 7;               // ACB J7 - amperomierz drugiej grupy silnikow
    const byte pinLVoltage = 8;                 // ACB J3 - woltomierz niskiego napiecia
    const byte pinLCurrent = 9;                 // ACB J4 - amperomierz niskiego napiecia

    const byte pinRysik_1H = 32;                // ACB J31-2
    const byte pinRysik_1L = 33;                // ACB J31-3
    const byte pinRysik_2H = 34;                // ACB J31-4
    const byte pinRysik_2L = 35;                // ACB J31-5
    const byte pinRysik_3 = 36;                 // ACB J31-6

    const byte pinSandboxActivate = 40;         // ACB J12-1 - pedal piasecznicy
    const byte pinAlerterAcknowledge = 41;      // ACB J13-J14-1 - przycisk i pedal kasowania CA/SHP
    const byte pinHornHighActivate = 42;        // ACB J19-1 - Przycisk - syrena - ton wysoki
    const byte pinHornLowActivate = 43;         // ACB J19-2 - Przycisk - syrena - ton niski

    const byte pinHeadLightEnableLeft = 47;     // ACB J26 - Hebelek - reflektor lewy
    const byte pinHeadLightEnableUpper = 48;    // ACB J27 - Hebelek - reflektor gorny
    const byte pinHeadLightEnableRight = 49;    // ACB J28 - Hebelek - reflektor prawy
    const byte pinRedMarkerEnableLeft = 50;     // ACB J29 - Hebelek - sygnał czerwony lewy
    const byte pinRedMarkerEnableRight = 51;    // ACB J30 - Hebelek - sygnał czerwony prawy
    const byte pinInteriorLightEnable = 52;     // ACB J34 - Hebelek - oświetlenie kabiny
    const byte pinInteriorLightDimEnable = 53;  // ACB J35 - Hebelek - przyciemniania oświetlenia kabiny

    const uint8_t pinTrainBrake = A0;           // ACB J1-2 - kran hamulca zespolonego
    const uint8_t pinIndependentBrake = A1;     // ACB J2-2 - kran hamulca pomocniczego

    // piny obslugi radiotelefonu (bezposrednio z Arduino - brak pinow na ACB)
    //const byte pinRadioToggle = 22;          // D22 - przycisk wlaczenia radiotelefonu (pomaranczowy)
    //const byte pinRadioChannelIncrease = 23; // D23 - wybor wyzszego kanalu radiotelefonu (zielony)
    //const byte pinRadioChannelDecrease = 24; // D24 - wybor nizszego kanalu radiotelefonu (bialo-zielony)
    //const byte pinRadioStopSend = 25;        // D25 - emisja RadioStopu (niebieski)
    //const byte pinRadioStopTest = 26;        // D26 - test RadioStopu (bialo-brazowy)

    //Plytka I2CLK (Interfejs Lampek Kontrolnych)
    const byte pinI2CLK_LED2 = 4; // LED2 - zolty - przekaznik nadmiarowy sprezarki
    const byte pinI2CLK_LED3 = 5; // LED3 - zielony - przekaznik nadmiarowy silnikow
    const byte pinI2CLK_LED4 = 6; // LED4 - niebieski - przekaznik nadmiarowy przetwornicy

    const byte pinLWO = 0;  // exp12 - LWO - lampka wentylatorów oporów
    const byte pinLWR = 1;  // exp12 - LWR - lampka wysokiego rozruchu
    const byte pinLOP = 2;  // exp12 - LOP - lampka ogrzewania pociągu
    const byte pinLJO = 3;  // exp12 - LJO - lampka jazdy na oporach rozruchowych
    const byte pinLP  = 4;  // exp12 - LP - lampka poślizgu
    const byte pinLCA = 5;  // exp12 - LCA - lampka CA
    const byte pinLSHP= 6;  // exp12 - LSHP - lampka SHP
    const byte pinLSL = 7;  // exp12 - LSL - lampka styczników liniowych
    const byte pinLNP = 0;  // exp13 - LNP - lampka przekaźnika nadmiarowego przetwornicy
    const byte pinLOG = 7;  // exp13 - LOG - lampka przekaznika roznicowego obwodu glownego
    const byte pinLNS = 1;  // exp13 - LNS - lampka przekaźnika nadmiarowego silników trakcyjnych
    const byte pinLWS = 2;  // exp13 - LWS - lampka WS
    const byte pinLNSP= 3;  // exp13 - LNSP - przekaźnik nadmiarowy sprezarek

    // Plytka I2CIPD (Interfejs Panelu Dolnego)
    const byte pinI2CIPD_LED2 = 7;    // exp11 - LED2 - zolty - buczek
    const byte pinI2CIPD_LED3 = 3;    // exp9 - LED3 - zielony - niezdefiniowany

    const byte pinRezerwa_1 = 0;                          // exp9 - J2 - przycisk Rezerwa P1
    const byte pinRezerwa_2 = 1;                          // exp9 - J3 - przycisk Rezerwa P2
    const byte pinForceAdjust = 2;                        // exp9 - J22 - przelacznik pakietowy dostosowania sil
    const byte pinAlerterSound = 4;                       // exp9 - J4 - sygnal dzwiekowy CA/SHP (buczek)
    const byte pinCompressorOverloadRelayReset = 5;       // exp9 - J5 - przycisk odblokowania przekaźnika sprężarki i wentylatorów oporów
    const byte pinLineBrakerOpen = 6;                     // exp9 - J6 - przycisk wyłącznika WSa
    const byte pinLineBrakerClose = 7;                    // exp9 - J7 - przycisk załączający WSa
    const byte pinMotorOverloadRelayReset = 0;            // exp10 - J8 - przycisk odblokowania przekaźników nadmiarowych silników trakcyjnych i różnicowego obwodu głównego
    const byte pinConverterOverloadRelayReset = 1;        // exp10 - J9 - przycisk odblokowania przekaźników nadmiarowych przetwornicy, ogrzewania pociągu i różnicowych obw. pomocniczych
    const byte pinMotorConnectorsOpen = 2;                // exp10 - J10 - przycisk wyłącznika styczników liniowych
    const byte pinConverterEnable = 3;                    // exp10 - J11 - hebelek załączania przetwornicy
    const byte pinCompressorEnable = 4;                   // exp10 - J12 - hebelek załączania sprężarki
    const byte pinHeatingEnable = 5;                      // exp10 - J13 - hebelek ogrzewania składu
    const byte pinBrakeActingSpeedSetCargo = 6;           // exp10 - J14-3 - przełącznik rodzaju hamulca (B)
    const byte pinBrakeActingSpeedSetRapid = 7;           // exp10 - J14-1 - przełącznik rodzaju hamulca (Shift/Ctrl + B)
    const byte pinMotorOverloadRelayThresholdSetLow = 1;  // exp11 - J15 - przełącznik rozruchu wysoki/niski
    const byte pinPantographRaiseFront = 0;               // exp11 - J16 - hebelek pantografu przedniego
    const byte pinPantographRaiseRear = 2;                // exp11 - J17 - hebelek pantografu tylnego
    const byte pinWheelSpinBrakeActivate = 3;             // exp11 - J18 - przycisk przyhamowania przy poślizgu
    const byte pinIndependentBrakeBailOff = 4;            // exp11 - J19 - przycisk odluźniacza
    const byte pinBatteryEnable = 5;                      // exp11 - J20 - hebelek załączenia baterii
    const byte pinHeadLightsDimEnable = 6;                // exp11 - J21 - przyciemnianie reflektorów

    // Plytka I2CIPM (Interfejs Przedzialu Maszynowego)
    const byte pinI2CIPM_LED1 = 4;                      // exp14 - LED1 - zielony - kabina A (HOME)
    const byte pinI2CIPM_LED2 = 5;                      // exp14 - LED2 - zielony - kabina B (END)
    
    const byte pinCabChangeBackward = 0;                // exp14 - przycisk przejscia w kierunku tylu (End)
    const byte pinCabChangeForward = 1;                 // exp14 - przycisk przejscia w kierunku czola (Home)
    const byte pinPantographCompressorActivate = 2;     // exp14 - przycisk sprezarki pomocniczej
    const byte pinPantographCompressorValveEnable2 = 3; // exp14 - przelacznik zaworu trojdrogowego - pozycja 2
    const byte pinPantographCompressorValveEnable1 = 6; // exp14 - przelacznik zaworu trojdrogowego - pozycja 1
    //const byte pinNiepodlaczony = 7;                   // exp14 - niepodlaczony

    // Plytka I2CIN (Interfejs Nastawnikow)
    const byte pinI2CIN_LED2 = 7;     // exp5 - LED2 - zielony - pozycje nastawnika kierunku
    const byte pinI2CIN_LED3 = 7;     // exp6 - LED3 - zolty - pozycja bocznika
    const byte pinI2CIN_LED4 = 5;     // exp5 - LED4 - niebieski - pozycje nastawnika jazdy
      // piny nastawnika kierunku
    const byte pinReverserForward = 4;    // exp5 - jazda do przodu
    const byte pinReverserBackward = 6;   // exp5 - jazda do tylu
      // piny nastawnika bocznikowania (camX - numer krzywki)
    //const byte pinSecondController_cam3 = 1;  // exp6 - krzywka 3, bit 1, pozycja 0 - krzywka i styk dla pozycji 0 nie sa wykorzystywane
    const byte pinSecondController_cam5 = 0;  // exp6 - krzywka 5, bit 0, pozycja 1
    const byte pinSecondController_cam7 = 3;  // exp6 - krzywka 7, bit 3, pozycja 2
    const byte pinSecondController_cam9 = 2;  // exp6 - krzywka 9, bit 2, pozycja 3
    const byte pinSecondController_cam11 = 6; // exp6 - krzywka 11, bit 6, pozycja 4
    const byte pinSecondController_cam12 = 5; // exp6 - krzywka 12, bit 5, pozycja 5
    const byte pinSecondController_cam13 = 4; // exp6 - krzywka 13, bit 4, pozycja 6
      // piny nastawnika jazdy (camX - numer zlacza opisany na przewodach stykow i jednoczesnie numer krzywki)
    //const byte MasterController_cam1 = 2; // exp0 - zlacze 1 - niepodlaczone
    const byte MasterController_cam2 = 3;   // exp0
    const byte MasterController_cam3 = 0;   // exp0
    const byte MasterController_cam4 = 1;   // exp0
    const byte MasterController_cam5 = 5;   // exp0
    const byte MasterController_cam6 = 4;   // exp0
    const byte MasterController_cam7 = 7;   // exp0
    const byte MasterController_cam8 = 6;   // exp0
    const byte MasterController_cam9 = 2;   // exp1
    const byte MasterController_cam10 = 3;  // exp1
    const byte MasterController_cam11 = 0;  // exp1
    const byte MasterController_cam12 = 1;  // exp1
    const byte MasterController_cam13 = 5;  // exp1
    const byte MasterController_cam14 = 4;  // exp1
    const byte MasterController_cam15 = 7;  // exp1
    const byte MasterController_cam16 = 6;  // exp1
    const byte MasterController_cam17 = 2;  // epx2
    const byte MasterController_cam18 = 3;  // epx2
    const byte MasterController_cam19 = 0;  // epx2
    const byte MasterController_cam20 = 1;  // epx2
    const byte MasterController_cam21 = 5;  // epx2
    const byte MasterController_cam22 = 4;  // epx2
    const byte MasterController_cam23 = 7;  // epx2
    const byte MasterController_cam24 = 6;  // epx2
    const byte MasterController_cam25 = 2;  // exp3
    const byte MasterController_cam26 = 3;  // exp3
    const byte MasterController_cam27 = 0;  // exp3
    const byte MasterController_cam28 = 1;  // exp3
    const byte MasterController_cam29 = 5;  // exp3
    const byte MasterController_cam30 = 4;  // exp3
    const byte MasterController_cam33 = 2;  // exp4
    const byte MasterController_cam34 = 3;  // exp4

    
  // Tablice do komunikacji UART z exe
    volatile uint8_t zPC[52] = {0};                       // dane przekazywane z exe do Arduino
    const byte zPC_byteLength= 52;                        // liczba bajtow wejsciowych
    volatile uint8_t doPC[20] = {0xEF, 0xEF, 0xEF, 0xEF}; // dane wysylane z Arduino do exe
    const byte doPC_byteLength= 20;                       // liczba bajtow wyjsciowych

  // Tablice do komunikacji UART z Arduino MEGA do NANO
    volatile uint8_t zNANO[1] = {0};
    const byte zNANO_byteLength = 1;
    volatile uint8_t doNANO[2] = {0};
    const byte doNANO_byteLength = 2;

  // Zmienne dla bitow ramki zNANO i doNANO
    byte doNANO_RadioChannel = 0;             // numer kanalu radiowego
    
    bool zNANO_RadioToggle = false;           // przycisk wlaczenia radiotelefonu
    bool zNANO_RadioChannelIncrease = false;  // wybor wyzszego kanalu radiotelefonu
    bool zNANO_RadioChannelDecrease = false;  // wybor nizszego kanalu radiotelefonu
    bool zNANO_RadioStopSend = false;         // emisja RadioStopu
    bool zNANO_RadioStopTest = false;         // test RadioStopu
    bool zNANO_RadioCall3Send = false;        // przycisk wywolania ZEW3
    bool zNANO_RadioVolumeIncrease = false;   // przycisk zwiekszenia glosnosci
    bool zNANO_RadioVolumeDecrease = false;   // przycisk zmniejszenia glosnosci


  // Zmienne dla bitow ramki zPC
    byte zPC_Pre_0 = 0xEF;      // Bajt 0 - preambula wysylana przez nowe exe
    byte zPC_Pre_1 = 0xEF;      // Bajt 1 - preambula wysylana przez nowe exe
    byte zPC_Pre_2 = 0xEF;      // Bajt 2 - preambula wysylana przez nowe exe
    byte zPC_Pre_3 = 0xEF;      // Bajt 3 - preambula wysylana przez nowe exe
  
    byte zPC_Tacho = 0;         // predkosc dla falownika

    bool zPC_LWO = false;       // LWO - lampka wentylatorow oporow
    bool zPC_LWR = false;       // LWR - lampka rysokiego rozruchu
    bool zPC_LOP = false;       // LOP - lampka ogrzewania pociągu
    bool zPC_LJO = false;       // LJO - lampka jazdy na oporach rozruchowych
    bool zPC_LP  = false;       // LP - lampka poślizgu
    bool zPC_LCA = false;       // LCA - lampka CA
    bool zPC_LSHP= false;       // LSHP - lampka SHP
    bool zPC_LSL = false;       // LSL - lampka styczników liniowych
    bool zPC_LNP = false;       // LNP - lampka przekaźnika nadmiarowego przetwornicy
    bool zPC_LOG = false;       // LOG - lampka przekaznika roznicowego obwodu glownego
    bool zPC_LNS = false;       // LNS - lampka przekaźnika nadmiarowego silników trakcyjnych
    bool zPC_LWS = false;       // LWS - lampka WS

    bool zPC_Rysik_1 = false;   // Hasler - rysik 1 - kabina A/B
    bool zPC_Rysik_2 = false;   // Hasler - rysik 2 - hamowanie
    bool zPC_Rysik_3 = false;   // Hasler - rysik 3 - jazda pod pradem

    //bool zPC_SpringBrake = false;   // hamulec sprezynowy (nieoprogramowany)
    bool zPC_AlerterSound = false;  // Sygnal dzwiekowy - buczek - CA/SHP

    byte zPC_BrakePress = 0;    // Ciśnienie w cylindrze hamulcowym
    byte zPC_BrakePress1 = 0;    // Ciśnienie w cylindrze hamulcowym
    int BrakePress = 0;    // Ciśnienie w cylindrze hamulcowym
    byte zPC_PipePress = 0;     // Ciśnienie w przewodzie glownym
    byte zPC_PipePress1 = 0;     // Ciśnienie w przewodzie glownym
    int PipePress = 0;     // Ciśnienie w przewodzie glownym
    byte zPC_TankPress = 0;     // Ciśnienie w zbiorniku glownym
    byte zPC_TankPress1 = 0;     // Ciśnienie w zbiorniku glownym
    int TankPress = 0;     // Ciśnienie w zbiorniku glownym

    byte zPC_HVoltage = 0;      // Woltomierz wysokiego napiecia
    byte zPC_HCurrent_1 = 0;    // Amperomierz pierwszej grupy silnikow
    //byte zPC_HCurrent_1a = 0;   // Amperomierz pierwszej grupy silnikow - drugi bajt dla wersji 16 bitowej
    byte zPC_HCurrent_2 = 0;    // Amperomierz drugiej grupy silnikow
    //byte zPC_HCurrent_2a = 0;   // Amperomierz drugiej grupy silnikow - drugi bajt dla wersji 16 bitowej
  
    //byte zPC_LVoltage = 0;      // Woltomierz niskiego napiecia (obslugiwane w exe ale nie uzywane w kodzie)
    //byte zPC_LCurrent = 0;      // Amperomierz niskiego napiecia (nieobslugiwane w exe)

    byte zPC_RadioChannel = 0;     // Kanal radiowy
    bool zPC_RadioStop = false;     // RadioStop

  // Zmienne dla bitow z ramki doPC
    byte doPC_MasterController = 0;                       // Pozycja nastawnika jazdy
    byte doPC_SecondController = 0;                       // Pozycja nastawnika oslabienia pola (bocznika)
    byte doPC_TrainBrake = 0;                             // Pozycja kranu hamulca zespolonego
    byte doPC_IndependentBrake = 0;                       // Pozycja kranu hamulca pomocniczego
  
    bool doPC_LineBrakerOpen = false;                     // Przycisk - wylaczenie WSa
    bool doPC_LineBrakerClose = false;                    // Przycisk - zalaczenie WSa
    bool doPC_MotorOverloadRelayReset = false;            // Przycisk - odblok. przekaźników nadmiarowych silników trakcyjnych i różnicowego obwodu głónego
    bool doPC_CompressorOverloadRelayReset = false;       // Przycisk - odblok. przekaznika sprezarki i wentylatorow oporow
    bool doPC_ConverterOverloadRelayReset = false;        // Przycisk - odblok. przekaznikow nadmiarowych przetwornicy, ogrzewania pociągu i różnicowych obw. pomocniczych
    bool doPC_MotorConnectorsOpen = false;                // Przycisk - wylacznik stycznikow liniowych
    bool doPC_AlerterAcknowledge = false;                 // Przycisk - kasowanie CA i SHP
    bool doPC_BatteryEnable = false;                      // Hebelek - bateria akumulatorów
    bool doPC_ConverterEnable = false;                    // Hebelek - przetwornica
    bool doPC_CompressorEnable = false;                   // Hebelek - sprezarka
    bool doPC_SandboxActivate = false;                    // Przycisk - piasecznica
    bool doPC_HeatingEnable = false;                      // Hebelek - ogrzewanie pociagu
    bool doPC_PantographCompressorActivate = false;       // Hebelek - aktywacja sprezarki pomocniczej
    bool doPC_PantographCompressorValveEnable = false;    // Hebelek - przelaczenie stanu korka trojdrogowego
    bool doPC_MotorOverloadRelayThresholdSetLow = false;  // Hebelek - przełącznik pakietowy wysokiego rozruchu
    bool doPC_PantographRaiseFront = false;               // Hebelek - pantograf przedni
    bool doPC_PantographRaiseRear = false;                // Hebelek - pantograf tylny
    bool doPC_WheelSpinBrakeActivate = false;             // Przycisk - przyhamowanie przy poślizgu
    bool doPC_HeadLightsDimEnable = false;                // Hebelek - przyciemnianie reflektorów
    bool doPC_InteriorLightDimEnable = false;             // Hebelek - przyciemnianie oświetlenia kabiny
    bool doPC_IndependentBrakeBailOff = false;            // Przycisk - odluzniacz
    bool doPC_HornHighActivate = false;                   // Przycisk - syrena - ton wysoki
    bool doPC_HornLowActivate = false;                    // Przycisk - syrena - ton niski

    bool doPC_HeadLightEnableLeft = false;                // Hebelek - reflektor lewy
    bool doPC_HeadLightEnableUpper = false;               // Hebelek - reflektor gorny
    bool doPC_HeadLightEnableRight = false;               // Hebelek - reflektor prawy
    bool doPC_RedMarkerEnableLeft = false;                // Hebelek - sygnał czerwony lewy
    bool doPC_RedMarkerEnableRight = false;               // Hebelek - sygnał czerwony prawy
    bool doPC_InteriorLightEnable = false;                // Hebelek - oświetlenie kabiny

    bool doPC_ReverserForward = false;                    // Przelacznik - jazda do przodu
    bool doPC_ReverserNeutral = false;                    // Przelacznik - pozycja neutralna nastawnika kierunku
    bool doPC_ReverserBackward = false;                   // Przelacznik - jazda do tylu
    bool doPC_BrakeActingSpeedSetCargo = false;           // Przelacznik - tryb hamulca dla pociągu towarowego
    bool doPC_BrakeActingSpeedSetPassenger = false;       // Przelacznik - tryb hamulca dla pociągu osobowego
    bool doPC_BrakeActingSpeedSetRapid = false;           // Przelacznik - tryb hamulca dla pociągu pospiecznego
    bool doPC_CabChangeForward = false;                   // Przycisk - przejscie do sasiedniego pomieszczenia w kierunku czola pociagu
    bool doPC_CabChangeBackward = false;                  // Przycisk - przejscie do sasiedniego pomieszczenia w kierunku tyłu pociagu

    bool doPC_RadioToggle = false;                        // przycisk wlaczenia radiotelefonu
    bool doPC_RadioChannelIncrease = false;               // wybor wyzszego kanalu radiotelefonu
    bool doPC_RadioChannelDecrease = false;               // wybor nizszego kanalu radiotelefonu
    bool doPC_RadioStopSend = false;                      // emisja RadioStopu
    bool doPC_RadioStopTest = false;                      // test RadioStopu
    bool doPC_RadioCall3Send = false;                     // przycisk wywolania ZEW3
    bool doPC_RadioVolumeIncrease = false;                // przycisk zwiekszenia glosnosci
    bool doPC_RadioVolumeDecrease = false;                // przycisk zmniejszenia glosnosci


  // Zmienne do kalibracji manometrow
    byte MCH;
    byte MCHcal;
    byte MPG;
    byte MPGcal;
    byte MZG;
    byte MZGcal;

    // wspolczynniki wielomianow kalibracji manometrow
    const float MCHcal_4 =  -0.0000001016142106406820 ;
    const float MCHcal_3 =  0.0000468328606375054000  ;
    const float MCHcal_2 =  -0.0049928797333640500000 ;
    const float MCHcal_1 =  0.9126620656032350000000  ;
    const float MCHcal_0 =  0.0 ;
        
    const float MPGcal_4 =  -0.0000000671782837013405 ;
    const float MPGcal_3 =  0.0000286987227455072000  ;
    const float MPGcal_2 =  -0.0028263350985381000000 ;
    const float MPGcal_1 =  0.9674990224009840000000  ;
    const float MPGcal_0 =  0.0 ;
        
    const float MZGcal_4 =  -0.0000000510127365529333 ;
    const float MZGcal_3 =  0.0000342288693121966000  ;
    const float MZGcal_2 =  -0.0053916654947057800000 ;
    const float MZGcal_1 =  0.9958145762419480000000  ;
    const float MZGcal_0 =  0.0 ;


  // Zmienne do kalibracji hamulcow
    byte HZ;
    byte HZcal;

  // Zmienne do uzaleznien przekaznikow
    bool PrzekRoznObwGl = true;
    bool PrzekNadmSiln = false;
    bool PrzekNadmPrzetw = false;
    bool PrzekNadmSprez = false;
    unsigned long licz_okres_przek_nadm_spr = 0; // zmienna dla opoznienia zadzialania nadmiarowego sprezarki
    const int ZwlokaPrzekNadmSpr = 1000; // czas od zadzialania przekaznika do zapalenia lampki LNSP
     
  // Zmienne nastawnika kierunkowego
    bool ReverserForward; // jazda do przodu
    bool ReverserBackward; // jazda do tylu

  // Zmienne dla ekspandera (6) nastawnika bocznikowania
    uint8_t B_exp6;
    bool B_OK; // potwierdzenie prawidlowej pozycji bocznika
  
  // Zmienne dla ekspanderow (0, 1, 2 i 3) stykow nastawnika jazdy
    uint8_t N_exp0;
    uint8_t N_exp1;
    uint8_t N_exp2;
    uint8_t N_exp3;
  // Zmienne pomocnicze do obslugi pozycji nastawnika
    uint16_t N_styki1 = 0;  // 16-bitowa dla ekspanderow 0 i 1
    uint16_t N_styki2 = 0;  // 16-bitowa dla ekspanderow 2 i 3
    uint32_t N_styki = 0;   // 32-bitowa dla wszystkich ekspanderow 0, 1, 2 i 3 (sklejana z dwoch 16-bitowych)
  // Zmienne stykow pomocniczych nastawnika jazdy
    bool N_jazda; // stan styku 34 - jazda pod pradem
    bool N_uklad_I; // stan styku 30 - jazda na I ukladzie szeregowym
    bool N_uklad_II; // stan styku 33 - jazda na II ukladzie rownoleglym
    bool N_OK; // potwierdzenie prawidlowej pozycji nastawnika
  
  // Zmienne do sterowania woltomierzem i amperomierzem NN
    const byte BatteryVoltage = 90;                     //stala wskazanie napiecia z baterii
    const byte LVoltage = 114;                          //stala wskazanie napiecia z przetwornicy
    const byte BatteryCurrent = 32;                     //stala wskazanie natezenia ladowania baterii
    const byte okres_rozp_przetwornicy = 20;            //stala "okres rozpędzania przetwornicy = 20ms" - czyli co 20ms napięcie dawane przez przetwornicę przy starcie będzie się zwiększać o 1V
    const int czas_zwal_przetwornicy = 7000;            //stala czas, przez jaki wyłączona i zwalnijąca przetwornica będzie jeszcze utrzymywać napięcie
    const byte okres_zatrz_przetwornicy = 150;          //stala "okres zatrzymania przetwornicy" - czyli co 150ms będzie na koniec obniżać dawane napięcie o 1V
    byte woltomierz = 0;                                //zmienna wartości napięcia na woltomierzu
    byte napiecie = 0;                                  //zmienna wartości napięcia dawanego przez przetwornicę
    byte amperomierz = 0;                               //zmienna wartości natęrzenia na amperomierzu
    byte natezenie = 0;                                 //zmienna wartości natężenia pobieranego podczas łądowania baterii
    bool przetw_zal = false;                            //zmienna rzeczywistego stanu przetwornicy
    unsigned long licz_okres_rozp_przetwornicy = 0;     //zmienna o dużej "pojemności" wykorzystywana w timerze zamiast funkcji "delay", żeby nie zatrzymywać działania całego programu
    unsigned long licz_czas_zwal_przetwornicy = 0;      //podobnie jak wyżej
    unsigned long licz_okres_zatrz_przetwornicy = 0;    //podobnie jak wyżej
