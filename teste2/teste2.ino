#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <PID_v1_bc.h>  // Biblioteca para controle PID
#include <math.h>
#include <SD.h>         // Biblioteca para comunicação com o cartão SD

LiquidCrystal_I2C lcd(0x27, 20, 4); // Endereço do LCD
File myFile;                        // Arquivo para armazenar dados no SD

#define MAX6675_SO 2
#define MAX6675_CS 3
#define MAX6675_SCK 4
#define RELE_RESISTENCIA 5  // Pino do relé da resistência
#define RELE_VENTILACAO 6   // Pino do relé da ventilação
#define ADCPIN A3           // Pino de leitura do sensor de temperatura (NTC 10k)
#define CS_PIN 10           // Pino de Chip Select (CS) para o SD Card

// Constantes para o NTC 10k
const double SERIES_RESISTOR = 10000.0;
const double BETA_COEFFICIENT = 3950.0;
const double ROOM_TEMP_RESISTANCE = 10000.0;
const double NOMINAL_TEMP = 25.0;

double temperatura;
double tempNTC;
double setPoint_resistencia = 24.5;
//double setPoint_ventilacao = 24.5;
double input, output_resistencia, output_ventilacao;
double Kp = 2.0, Ki = 5.0, Kd = 1.0; // Ajustar conforme necessário

unsigned long tempoAnterior = 0;
const long intervaloTela = 10000;     // Intervalo de 10 segundos para atualizar a tela
unsigned long intervaloPID = 200;     // Intervalo para atualizar o PID

// Inicializar objetos PID
PID pid_resistencia(&input, &output_resistencia, &setPoint_resistencia, Kp, Ki, Kd, DIRECT);
//PID pid_ventilacao(&input, &output_ventilacao, &setPoint_ventilacao, Kp, Ki, Kd, DIRECT);

void setup() {
    analogReference(DEFAULT);
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    pinMode(RELE_RESISTENCIA, OUTPUT);
    //pinMode(RELE_VENTILACAO, OUTPUT);

    lcd.setCursor(0, 1);
    lcd.print("Bem Vindo!");
    delay(1000);
    lcd.clear();

    lcd.setCursor(0, 1);
    lcd.print("Projeto Capeta");
    delay(1000);
    lcd.clear();

    // Inicializar PID
    pid_resistencia.SetMode(AUTOMATIC);
    pid_resistencia.SetOutputLimits(0, 255);
    //pid_ventilacao.SetMode(AUTOMATIC);
    //pid_ventilacao.SetOutputLimits(0, 255);

    // Inicializar o cartão SD
    if (!SD.begin(CS_PIN)) {
        Serial.println("Falha ao inicializar o cartão SD");
        lcd.setCursor(0, 0);
        lcd.print("Falha no SD!");
        while (1);
    }
    
    myFile = SD.open("datalog.csv", FILE_WRITE);
    if (myFile) {
        myFile.println("Tempo,Temp In (C),Temp Out (C)");
        myFile.close();
        Serial.println("Arquivo CSV criado com sucesso");
    } else {
        Serial.println("Erro ao criar arquivo CSV");
    }

    tempoAnterior = millis();
}

void loop() {
    unsigned long currentMillis = millis();

    // Atualizar leitura e controle PID em intervalos rápidos
    if (currentMillis - tempoAnterior >= intervaloPID) {
        tempoAnterior = currentMillis;

        // Ler temperatura do termopar e atualizar PID
        temperatura = leer_termopar();
        input = temperatura;
        
        // Controle PID para resistência
        pid_resistencia.Compute();
        digitalWrite(RELE_RESISTENCIA, output_resistencia > 127 ? LOW : HIGH);

        // Controle PID para ventilação
        //pid_ventilacao.Compute();
        //digitalWrite(RELE_VENTILACAO, temperatura > setPoint_ventilacao ? HIGH : LOW);

        // Ler a temperatura do sensor NTC
        tempNTC = lerNTC(ADCPIN);

        // Enviar dados para monitor serial
        Serial.print("Temp. In: ");
        Serial.print(temperatura, 2);
        Serial.print(" C, Temp. Out: ");
        Serial.print(tempNTC, 2);
        Serial.println(" C");

        // Registrar dados no arquivo CSV
        logToFile(temperatura, tempNTC);
    }

    // Alternar entre telas a cada 10 segundos
    if (currentMillis % intervaloTela < intervaloPID) {
        lcd.clear();
        if ((currentMillis / intervaloTela) % 2 == 0) {
            lcd.setCursor(0, 0);
            lcd.print("Set Res: ");
            lcd.print(setPoint_resistencia, 1);

            //lcd.setCursor(0, 1);
            //lcd.print("Set Vent: ");
            //lcd.print(setPoint_ventilacao, 1);
        } else {
            lcd.setCursor(0, 0);
            lcd.print("Temp In: ");
            lcd.print(temperatura, 1);

            lcd.setCursor(0, 1);
            lcd.print("Temp Out: ");
            lcd.print(tempNTC, 1);
        }
    }
}

void logToFile(double tempIn, double tempOut) {
    myFile = SD.open("datalog.csv", FILE_WRITE);
    if (myFile) {
        unsigned long currentTime = millis() / 1000;
        myFile.print(currentTime);
        myFile.print(",");
        myFile.print(tempIn, 2);
        myFile.print(",");
        myFile.println(tempOut, 2);
        myFile.close();
        Serial.println("Dados gravados no arquivo CSV");
    } else {
        Serial.println("Erro ao abrir o arquivo CSV para escrita");
    }
}

double leer_termopar() {
    uint16_t v;
    pinMode(MAX6675_CS, OUTPUT);
    pinMode(MAX6675_SO, INPUT);
    pinMode(MAX6675_SCK, OUTPUT);

    digitalWrite(MAX6675_CS, LOW);

    // Usar micros() para medir o tempo ao invés de delay()
    unsigned long startMicros = micros();
    while (micros() - startMicros < 1);  // Esperar 1 microsegundo

    v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
    v <<= 8;
    v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);

    digitalWrite(MAX6675_CS, HIGH);
    if (v & 0x4) {
        return NAN;  // Termopar desconectado
    }

    v >>= 3;
    return v * 0.25;
}

double lerNTC(int sensorPin) {
    int adcValue = analogRead(sensorPin);

    if (adcValue == 0) {
        return NAN;
    }

    double resistance = SERIES_RESISTOR * (1023.0 / adcValue - 1.0);
    double steinhart;
    steinhart = resistance / ROOM_TEMP_RESISTANCE;
    steinhart = log(steinhart);
    steinhart /= BETA_COEFFICIENT;
    steinhart += 1.0 / (NOMINAL_TEMP + 273.15);
    steinhart = 1.0 / steinhart;
    steinhart -= 273.15;
    return steinhart;
}
