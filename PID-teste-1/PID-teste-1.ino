#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <math.h>
#include <max6675.h>
#include <SD.h>

LiquidCrystal_I2C lcd(0x27, 20, 4); 
File myFile;  

#define MAX6675_SO 2
#define MAX6675_CS 3
#define MAX6675_SCK 4
#define RELE_RESISTENCIA 5  

const int pinLM35 = A0; 
#define CS_PIN 10  

double temperatura;
double setPoint_resistencia = 38.0;
double input;
float intercepto = 3.267;
float slope = 0.938;
unsigned long tempoAnterior = 0;
const long intervaloTela = 10000;  
unsigned long intervaloPID = 200;  

bool resistenciaLigada = false;  // Inicialmente desligado

// Parâmetros PID
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
double erroAnterior = 0.0, somaErro = 0.0;

void setup() {
    analogReference(DEFAULT);
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    pinMode(RELE_RESISTENCIA, OUTPUT);
    
    // Inicializa o relé como desligado
    digitalWrite(RELE_RESISTENCIA, LOW);
    
    if (!SD.begin(CS_PIN)) {
        Serial.println("Falha ao inicializar o cartão SD");
        lcd.setCursor(0, 0);
        lcd.print("Falha no SD!");
        while (1);
    }

    myFile = SD.open("datalog.csv", FILE_WRITE);
    if (myFile) {
        myFile.println("Tempo,Temp In (C),Temp Amb (C)");  
        myFile.close();
        Serial.println("Arquivo CSV criado com sucesso");
    } else {
        Serial.println("Erro ao criar arquivo CSV");
    }

    lcd.setCursor(0, 1);
    lcd.print("Bem Vindo!");
    delay(1000);
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Projeto Capeta");
    delay(1000);
    lcd.clear();
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - tempoAnterior >= intervaloPID) {
        tempoAnterior = currentMillis;

        temperatura = leer_termopar();
        float temperaturaCorrigida = (slope * temperatura) + intercepto;
        input = temperaturaCorrigida - 5.5; // Corrigindo a temperatura

        // Controle do relé com base no PID
        controlarRele(input); // Chamada para a nova função

        float temperaturaAmbiente = lerTemperaturaLM35();

        Serial.print("Temp. Bruta: ");
        Serial.print(temperatura, 2);
        Serial.print(" C, Temp. Corrigida: ");
        Serial.print(input, 2);
        Serial.print(" C, Temp. Ambiente: ");
        Serial.print(temperaturaAmbiente, 2);
        Serial.println(" C");

        logToFile(input, temperaturaAmbiente);
    }

    // Atualização do LCD
    if (currentMillis % intervaloTela < intervaloPID) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp In: ");
        lcd.print(input, 1);
        lcd.setCursor(0, 1);
        lcd.print("Temp Amb: ");
        lcd.print(lerTemperaturaLM35(), 1);
    }
}

void controlarRele(double temperaturaAtual) {
    double erro = setPoint_resistencia - temperaturaAtual;
    somaErro += erro * intervaloPID / 1000.0; // Integral acumulada
    double dErro = (erro - erroAnterior) / (intervaloPID / 1000.0); // Derivada

    // Saída PID
    double pidOut = Kp * erro + Ki * somaErro + Kd * dErro;
    erroAnterior = erro;

    // Controle do relé com base na saída PID
    if (pidOut > 0) {
        digitalWrite(RELE_RESISTENCIA, LOW); // Liga o relé
        resistenciaLigada = false;
    } else {
        digitalWrite(RELE_RESISTENCIA, HIGH); // Desliga o relé
        resistenciaLigada = true;
    }
}

double leer_termopar() {
    uint16_t v;
    pinMode(MAX6675_CS, OUTPUT);
    pinMode(MAX6675_SO, INPUT);
    pinMode(MAX6675_SCK, OUTPUT);

    digitalWrite(MAX6675_CS, LOW);
    unsigned long startMicros = micros();
    while (micros() - startMicros < 1);  

    v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
    v <<= 8;
    v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
    digitalWrite(MAX6675_CS, HIGH);

    if (v & 0x4) {
        return NAN;
    }
    v >>= 3;
    return v * 0.25;
}

float lerTemperaturaLM35() {
    int leituraLM35 = analogRead(pinLM35);
    float temperatura = (leituraLM35 * (5.0 / 1023.0)) * 100.0;
    return temperatura;
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
