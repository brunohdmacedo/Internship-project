#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <PID_v1_bc.h>  // Biblioteca para controle PID
#include <math.h>
#include <SD.h>         // Biblioteca SD inclusa

LiquidCrystal_I2C lcd(0x27, 20, 4); // Endereço do LCD
File myFile;                        // Cria um ponteiro para arquivo

// Mapeamento de hardware
#define MAX6675_SO 2
#define MAX6675_CS 3
#define MAX6675_SCK 4
#define RELE_RESISTENCIA 5  // Pino do relé da resistência
#define RELE_VENTILACAO 6   // Pino do relé da ventilação
#define ADCPIN A3           // Pino de leitura do sensor de temperatura (NTC 10k)
#define CS_pin 10           // Comunicação SPI, CS_pin no digital 10

// Constantes para o NTC 10k
const double SERIES_RESISTOR = 10000.0;
const double BETA_COEFFICIENT = 3950.0;
const double ROOM_TEMP_RESISTANCE = 10000.0;
const double NOMINAL_TEMP = 25.0;

double temperatura;         // Temperatura medida pelo MAX6675
double tempNTC;             // Temperatura medida pelo sensor NTC 10k
double setPoint_resistencia = 29.0;
double setPoint_ventilacao = 27.0;
double input, output_resistencia, output_ventilacao;
double Kp = 2.0, Ki = 5.0, Kd = 1.0; // Valores PID

// Variáveis para controle de tempo
unsigned long tempoAnterior = 0;
unsigned long tempoTela = 0;
const long intervalo = 10000;    // Intervalo para atualização de temperatura (10s)
const long intervaloTela = 30000; // Alternância entre as telas 3 e 4 (30s)
unsigned long minuto = 1;        // Armazena os minutos transcorridos

int telaAtual = 0;  // Indica a tela atual (usado para alternância)

PID pid_resistencia(&input, &output_resistencia, &setPoint_resistencia, Kp, Ki, Kd, DIRECT);
PID pid_ventilacao(&input, &output_ventilacao, &setPoint_ventilacao, Kp, Ki, Kd, DIRECT);

void setup() {
    analogReference(DEFAULT);
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    pinMode(RELE_RESISTENCIA, OUTPUT);
    pinMode(RELE_VENTILACAO, OUTPUT);

    tempoAnterior = millis();
    tempoTela = millis();

    // Inicializar PID
    pid_resistencia.SetMode(AUTOMATIC);
    pid_resistencia.SetOutputLimits(0, 255);
    pid_ventilacao.SetMode(AUTOMATIC);
    pid_ventilacao.SetOutputLimits(0, 255);

    // Inicialização do SD card
    pinMode(CS_pin, OUTPUT);
    if (!SD.begin(CS_pin)) {
        Serial.println("Falha no cartao!");
        return;
    }
    Serial.println("Sucesso na inicializacao!");

    myFile = SD.open("logger.csv", FILE_WRITE);
    if (myFile) {
        String header = "Minuto, T_Celsius, T_NTC, T_Fahrenheit";
        myFile.println(header);
        myFile.close();
        Serial.println(header);
    } else {
        Serial.println("Erro ao abrir arquivo");
    }

    exibirTelaInicial();
}

void loop() {
    unsigned long tempoAtual = millis(); // Tempo atual para comparação

    // Ler a temperatura a cada 10 segundos
    if (tempoAtual - tempoAnterior >= intervalo) {
        temperatura = leer_termopar();  // Ler temperatura do MAX6675
        tempNTC = lerNTC(ADCPIN);  // Ler temperatura do NTC 10k
        input = temperatura;

        // Controle PID para resistência
        pid_resistencia.Compute();
        if (temperatura < setPoint_resistencia - 0.5) {
            digitalWrite(RELE_RESISTENCIA, LOW);  // Ligar a resistência
        } else if (temperatura > setPoint_resistencia + 0.5) {
            digitalWrite(RELE_RESISTENCIA, HIGH);   // Desligar a resistência
        }

        // Controle PID para ventilação
        pid_ventilacao.Compute();
        if (temperatura > setPoint_ventilacao) {
            digitalWrite(RELE_VENTILACAO, HIGH);   // Ligar a ventilação
        } else {
            digitalWrite(RELE_VENTILACAO, LOW);    // Desligar a ventilação
        }

        // Atualizar as temperaturas no monitor serial
        Serial.print("Temperatura MAX6675 (IN): ");
        Serial.print(temperatura, 2);
        Serial.print(" C, Temperatura NTC (OUT): ");
        Serial.print(tempNTC, 2);
        Serial.println(" C");

        // Salvar os dados no cartão SD
        double temp_fahrenheit = (temperatura * 1.8) + 32;
        String dataString = String(minuto) + ", " + String(temperatura, 2) + ", " + String(tempNTC, 2) + ", " + String(temp_fahrenheit, 2);
        myFile = SD.open("logger.csv", FILE_WRITE);
        if (myFile) {
            myFile.println(dataString);
            myFile.close();
            Serial.println(dataString);
        } else {
            Serial.println("Erro ao abrir arquivo para escrita");
        }

        minuto++;
        tempoAnterior = tempoAtual;  // Atualiza o tempo da última leitura
    }

    // Alternar entre as telas 3 e 4 a cada 30 segundos
    if (tempoAtual - tempoTela >= intervaloTela) {
        alternarTela();
        tempoTela = tempoAtual;
    }
}

// Exibe as telas de boas-vindas
void exibirTelaInicial() {
    // Tela 1: "Bem vindo"
    lcd.clear();
    lcd.setCursor(5, 1);
    lcd.print("Bem Vindo!");
    delay(2000);

    // Tela 2: "Proj. do Diabo"
    lcd.clear();
    lcd.setCursor(3, 1);
    lcd.print("Projeto capeta");
    delay(2000);

    // Tela 3: Mostrar set points
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SetPoint Res: ");
    lcd.print(setPoint_resistencia, 1);
    lcd.setCursor(0, 1);
    lcd.print("SetPoint Vent: ");
    lcd.print(setPoint_ventilacao, 1);
    delay(2000);

    // Após exibição inicial, começa a alternância entre telas 3 e 4
    telaAtual = 3;
}

// Alterna entre as telas 3 e 4
void alternarTela() {
    lcd.clear();
    if (telaAtual == 3) {
        // Tela 3: Mostrar set points
        lcd.setCursor(0, 0);
        lcd.print("SetPoint Res: ");
        lcd.print(setPoint_resistencia, 1);
        lcd.setCursor(0, 1);
        lcd.print("SetPoint Vent: ");
        lcd.print(setPoint_ventilacao, 1);
        telaAtual = 4;
    } else {
        // Tela 4: Mostrar as temperaturas
        lcd.setCursor(0, 0);
        lcd.print("Temp IN: ");
        lcd.print(temperatura, 1);
        lcd.setCursor(0, 1);
        lcd.print("Temp OUT: ");
        lcd.print(tempNTC, 1);
        telaAtual = 3;
    }
}

// Função para ler a temperatura do termopar (MAX6675)
double leer_termopar() {
    uint16_t v;
    pinMode(MAX6675_CS, OUTPUT);
    pinMode(MAX6675_SO, INPUT);
    pinMode(MAX6675_SCK, OUTPUT);

    digitalWrite(MAX6675_CS, LOW);
    delay(1);

    v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
    v <<= 8;
    v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);

    digitalWrite(MAX6675_CS, HIGH);
    if (v & 0x4) {
        return NAN; // Termopar desconectado
    }

    v >>= 3;
    return v * 0.25;
}

// Função para calcular a temperatura do NTC 10k
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
