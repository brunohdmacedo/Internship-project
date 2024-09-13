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
const double SERIES_RESISTOR = 10000.0; // Resistor em série de 10k ohms
const double BETA_COEFFICIENT = 3950.0; // Coeficiente beta do NTC
const double ROOM_TEMP_RESISTANCE = 10000.0; // Resistência a 25ºC (10k ohms)
const double NOMINAL_TEMP = 25.0; // Temperatura nominal em graus Celsius

double temperatura;         // Temperatura medida pelo MAX6675
double tempNTC;             // Temperatura medida pelo sensor NTC 10k
double setPoint_resistencia = 29.0;  // Temperatura desejada para a resistência
double setPoint_ventilacao = 27.0;   // Temperatura para acionar a ventilação
double input, output_resistencia, output_ventilacao;
double Kp = 2.0, Ki = 5.0, Kd = 1.0; // Valores PID (devem ser ajustados)

// Variáveis para controle de tempo
unsigned long tempoAnterior = 0; // Tempo da última atualização de tela
const long intervaloTela = 20000; // Intervalo de 30 segundos para alternar a tela

// Inicializar objetos PID
PID pid_resistencia(&input, &output_resistencia, &setPoint_resistencia, Kp, Ki, Kd, DIRECT);
PID pid_ventilacao(&input, &output_ventilacao, &setPoint_ventilacao, Kp, Ki, Kd, DIRECT);

void setup() {
    analogReference(DEFAULT);  // Usar a referência de 5V para leitura precisa de NTC
    Serial.begin(9600);         // Comunicação serial
    lcd.init();
    lcd.backlight();
    pinMode(RELE_RESISTENCIA, OUTPUT);  // Configura o pino da resistência
    pinMode(RELE_VENTILACAO, OUTPUT);   // Configura o pino da ventilação

    lcd.setCursor(5, 1);
    lcd.print("Bem Vindo!");
    delay(1000);
    lcd.clear();

    lcd.setCursor(5, 1);
    lcd.print("Projeto Capeta");
    delay(1000);
    lcd.clear();

    // Inicializar PID
    pid_resistencia.SetMode(AUTOMATIC);
    pid_resistencia.SetOutputLimits(0, 255); // Limites de saída do PID para resistência
    pid_ventilacao.SetMode(AUTOMATIC);
    pid_ventilacao.SetOutputLimits(0, 255);  // Limites de saída do PID para ventilação

    // Inicializar o cartão SD
    if (!SD.begin(CS_PIN)) {
        Serial.println("Falha ao inicializar o cartão SD");
        lcd.setCursor(0, 0);
        lcd.print("Falha no SD!");
        while (1); // Para o código se o cartão SD falhar
    }
    
    // Abrir o arquivo e escrever o cabeçalho
    myFile = SD.open("datalog.csv", FILE_WRITE);
    if (myFile) {
        myFile.println("Tempo,Temp In (C),Temp Out (C)");
        myFile.close();
        Serial.println("Arquivo CSV criado com sucesso");
    } else {
        Serial.println("Erro ao criar arquivo CSV");
    }

    // Registrar o tempo inicial
    tempoAnterior = millis();
}

void loop() {
    // Ler a temperatura atual do MAX6675
    temperatura = leer_termopar();
    input = temperatura;  // Atualizar o valor de entrada para o PID

    // Controle PID para resistência
    pid_resistencia.Compute();
    if (output_resistencia > 127) {
        digitalWrite(RELE_RESISTENCIA, LOW);  // Ligar a resistência
    } else {
        digitalWrite(RELE_RESISTENCIA, HIGH);   // Desligar a resistência
    }

    // Controle PID para ventilação
    pid_ventilacao.Compute();
    if (temperatura > setPoint_ventilacao) {
        digitalWrite(RELE_VENTILACAO, HIGH);   // Ligar a ventilação
    } else {
        digitalWrite(RELE_VENTILACAO, LOW);    // Desligar a ventilação
    }

    // Ler a temperatura do sensor NTC 10k conectado ao pino ADC
    tempNTC = lerNTC(ADCPIN);             

    unsigned long currentMillis = millis();

    // Alternar entre telas a cada 30 segundos
    if (currentMillis - tempoAnterior >= intervaloTela) {
        lcd.clear();
        if ((currentMillis / intervaloTela) % 2 == 0) {
            // Tela de Setpoints
            lcd.setCursor(0, 0);  // Primeira linha
            lcd.print("Set Res: ");
            lcd.print(setPoint_resistencia, 1);  // Exibe o setpoint da resistência

            lcd.setCursor(0, 1);  // Segunda linha
            lcd.print("Set Vent: ");
            lcd.print(setPoint_ventilacao, 1);  // Exibe o setpoint da ventilação
        } else {
            // Tela de Temperaturas
            lcd.setCursor(0, 0);  // Primeira linha
            lcd.print("Temp In: ");
            lcd.print(temperatura, 1);  // Exibe a temperatura medida pelo MAX6675

            lcd.setCursor(0, 1);  // Segunda linha
            lcd.print("Temp Out: ");
            lcd.print(tempNTC, 1);  // Exibe a temperatura medida pelo NTC 10k
        }

        // Registrar as temperaturas no arquivo CSV
        logToFile(temperatura, tempNTC);

        // Atualizar o tempo da última atualização de tela
        tempoAnterior = currentMillis;
    }

    // Enviar as temperaturas atuais para o monitor serial
    Serial.print("Temp. In: ");
    Serial.print(temperatura, 2);
    Serial.print(" C, Temp. Out: ");
    Serial.print(tempNTC, 2);
    Serial.println(" C");

    delay(1000); // Aguardar 1 segundo antes da próxima leitura
}

// Função para registrar os dados no arquivo CSV
void logToFile(double tempIn, double tempOut) {
    myFile = SD.open("datalog.csv", FILE_WRITE);
    if (myFile) {
        unsigned long currentTime = millis() / 1000; // Tempo em segundos
        myFile.print(currentTime);
        myFile.print(",");
        myFile.print(tempIn, 2);  // Temperatura In (MAX6675)
        myFile.print(",");
        myFile.println(tempOut, 2);  // Temperatura Out (NTC)
        myFile.close();
        Serial.println("Dados gravados no arquivo CSV");
    } else {
        Serial.println("Erro ao abrir o arquivo CSV para escrita");
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

    v >>= 3; // Descartar bits de status
    return v * 0.25; // Converter para graus Celsius
}

// Função para calcular a temperatura do NTC 10k
double lerNTC(int sensorPin) {
    int adcValue = analogRead(sensorPin);  // Ler o valor ADC (0-1023)
    
    if (adcValue == 0) {
        return NAN;  // Evita divisão por zero, possivelmente o sensor não está conectado
    }

    double resistance = SERIES_RESISTOR * (1023.0 / adcValue - 1.0);  // Calcula a resistência do NTC
    double steinhart;
    steinhart = resistance / ROOM_TEMP_RESISTANCE;                     // (R/Ro)
    steinhart = log(steinhart);                                        // ln(R/Ro)
    steinhart /= BETA_COEFFICIENT;                                     // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_TEMP + 273.15);                        // + (1/To)
    steinhart = 1.0 / steinhart;                                       // Inverte
    steinhart -= 273.15;                                               // Converte para Celsius
    return steinhart;                                                  // Retorna a temperatura em Celsius
}
