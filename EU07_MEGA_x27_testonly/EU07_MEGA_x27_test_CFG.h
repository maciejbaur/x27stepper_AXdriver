// Definicje expanderow, stalych i zmiennych

  // Obsluga silnikow krokowych x25
  #include <SwitecX12.h> // biblioteka do silniczkow podlaczenie przez AX
  #include <SwitecX25.h>    // biblioteka do silniczkow bezposrednie podlaczenie do Arduino


  const int STEPSAX (315*12);   // AX obsluguje 3780 krokow
  const int STEPSMEGA (315*6);  // Arduino obsluguje 1890 krokow

  // WYBIERZ WLASCIWA FUNKCJE ZALEZNIE OD RODZAJU POLACZENIA

  // Odkomentuj gdy polaczenie przez AX
  // AX - poczatek
    const int A_STEP = 4;
    const int A_DIR = 5;
    const int B_STEP = 6;
    const int B_DIR = 7;
    const int C_STEP = 8;
    const int C_DIR = 9;
    const int RESET = 10;
    
    SwitecX12 motor1(STEPSAX, C_STEP, C_DIR);
    SwitecX12 motor2(STEPSAX, B_STEP, B_DIR);
    SwitecX12 motor3(STEPSAX, A_STEP, A_DIR);
  // AX - koniec

  // Odkomentuj gdy bezposrednie podlaczenie do Arduino
  // Arduino - poczatek
    // SwitecX25 motor1(STEPSMEGA,22,23,24,25); // brake press
    // SwitecX25 motor2(STEPSMEGA,4,5,6,7); // main pipe
    // SwitecX25 motor3(STEPSMEGA,8,9,10,11); // main tank
  // Arduino - koniec

  // Definicja stalych dla pinow wejscia / wyjscia

    const byte pinZero = 3; // do uruchomienia zerowania manometrow
    const byte pinTest = 2; // do uruchomienia testu manometrow
    
  // Tablice do komunikacji UART z exe
    volatile uint8_t zPC[52] = {0};                       // dane przekazywane z exe do Arduino
    const byte zPC_byteLength= 52;                        // liczba bajtow wejsciowych
    volatile uint8_t doPC[20] = {0xEF, 0xEF, 0xEF, 0xEF}; // dane wysylane z Arduino do exe
    const byte doPC_byteLength= 20;                       // liczba bajtow wyjsciowych


  // Zmienne dla bitow ramki zPC
    byte zPC_Pre_0 = 0xEF;      // Bajt 0 - preambula wysylana przez nowe exe
    byte zPC_Pre_1 = 0xEF;      // Bajt 1 - preambula wysylana przez nowe exe
    byte zPC_Pre_2 = 0xEF;      // Bajt 2 - preambula wysylana przez nowe exe
    byte zPC_Pre_3 = 0xEF;      // Bajt 3 - preambula wysylana przez nowe exe
  

    byte zPC_BrakePress = 0;    // Ciśnienie w cylindrze hamulcowym
    byte zPC_BrakePress1 = 0;   // Ciśnienie w cylindrze hamulcowym
    int BrakePress = 0;         // Ciśnienie w cylindrze hamulcowym
    byte zPC_PipePress = 0;     // Ciśnienie w przewodzie glownym
    byte zPC_PipePress1 = 0;    // Ciśnienie w przewodzie glownym
    int PipePress = 0;          // Ciśnienie w przewodzie glownym
    byte zPC_TankPress = 0;     // Ciśnienie w zbiorniku glownym
    byte zPC_TankPress1 = 0;    // Ciśnienie w zbiorniku glownym
    int TankPress = 0;          // Ciśnienie w zbiorniku glownym

