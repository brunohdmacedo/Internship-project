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
double setPoint_resistencia = 62.0;
double input;
float intercepto = 3.267;
float slope = 0.938;
unsigned long tempoAnterior = 0;
const long intervaloTela = 10000;  
unsigned long intervaloPID = 200;  

bool resistenciaLigada = false;  // Inicialmente ligada

// Parâmetros PID
double Kp = 2.0, Ki = 0.5, Kd = 1.0;
double erroAnterior = 0.0, somaErro = 0.0;

// Filtro passa-baixo
double temperaturaFiltrada = 0.0;
double alpha = 0.1;  // Fator de suavização para o filtro passa-baixo

// Variáveis para média das últimas 5 leituras de temperatura corrigida
#define NUM_LEITURAS 5
double leituras[NUM_LEITURAS];
int leituraIndex = 0;

// Zona morta (histerese)
double zonaMorta = 2.0; // Intervalo de ±2°C em torno do set point

// Flag para o comportamento baseado no setpoint
int flag = 0;  // Flag começa em 0, muda para 1 depois que o setpoint é ultrapassado

// Flag para o modo de aquecimento inicial
bool modoAquecimento = true;
unsigned long tempoAquecimento = 0;
const unsigned long tempoAquecimentoMax = 600000; // 10 minutos

void setup() {
    analogReference(DEFAULT);
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    pinMode(RELE_RESISTENCIA, OUTPUT);
    
    // Inicializa a resistência como ligada
    digitalWrite(RELE_RESISTENCIA, HIGH);
    resistenciaLigada = true;
    
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

    // Modo de aquecimento inicial: mais agressivo
    if (modoAquecimento) {
        // Durante o aquecimento, usar um Kp maior para acelerar o processo
        Kp = 4.0;
        Ki = 1.0;
        Kd = 0.5;
        if (currentMillis - tempoAquecimento >= tempoAquecimentoMax) {
            modoAquecimento = false;  // Depois de 10 minutos, desativa o modo de aquecimento
        }
    } else {
        // Parâmetros PID normais depois do aquecimento
        Kp = 2.0;
        Ki = 0.5;
        Kd = 1.0;
    }

    if (currentMillis - tempoAnterior >= intervaloPID) {
        tempoAnterior = currentMillis;

        temperatura = leer_termopar();
        float temperaturaCorrigida = (slope * temperatura) + intercepto;

        // Aplica o filtro passa-baixo na leitura do termopar
        temperaturaFiltrada = alpha * temperaturaCorrigida + (1 - alpha) * temperaturaFiltrada;

        // Atualiza a média das últimas 5 leituras para o PID
        leituras[leituraIndex] = temperaturaFiltrada;
        leituraIndex = (leituraIndex + 1) % NUM_LEITURAS;
        double temperaturaMedia = 0.0;
        for (int i = 0; i < NUM_LEITURAS; i++) {
            temperaturaMedia += leituras[i];
        }
        temperaturaMedia /= NUM_LEITURAS;

        input = temperaturaMedia - 5.5; // Corrigindo a temperatura

        // Verifica se a temperatura ultrapassou o setpoint para mudar o flag para 1
        if (flag == 0 && input > setPoint_resistencia) {
            flag = 1;  // Muda o flag para 1 depois que o setpoint é ultrapassado
        }

        // Controle do relé com base no PID e zona morta
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

    if (flag == 0) {
        // Modo pré-aquecimento: mantém o relé ligado
        digitalWrite(RELE_RESISTENCIA, LOW); 
        resistenciaLigada = true;
        if (temperaturaAtual >= setPoint_resistencia) {
            flag = 1;  // Ativa controle PID após atingir o setpoint
        }
    } else if (flag == 1) {
        // Controle PID com histerese
        if (pidOut > zonaMorta) {
            digitalWrite(RELE_RESISTENCIA, HIGH); // Desliga o relé
            resistenciaLigada = false;
        } else if (pidOut < -zonaMorta) {
            digitalWrite(RELE_RESISTENCIA, LOW); // Liga o relé
            resistenciaLigada = true;
        }
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
