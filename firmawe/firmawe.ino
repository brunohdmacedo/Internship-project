#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <PID_v1_bc.h>  // Biblioteca para controle PID

LiquidCrystal_I2C lcd(0x27, 20, 4); // Endereço do LCD

#define MAX6675_SO 2
#define MAX6675_CS 3
#define MAX6675_SCK 4
#define RELE_RESISTENCIA 5  // Pino do relé da resistência
#define RELE_VENTILACAO 6   // Pino do relé da ventilação

double temperatura;  // Temperatura medida
double setPoint_resistencia = 24.50;  // Temperatura desejada para a resistência
double setPoint_ventilacao = 28.0;   // Temperatura para acionar a ventilação
double input, output_resistencia, output_ventilacao;
double Kp = 2.0, Ki = 5.0, Kd = 1.0; // Valores PID (devem ser ajustados)

// Inicializar objetos PID
PID pid_resistencia(&input, &output_resistencia, &setPoint_resistencia, Kp, Ki, Kd, DIRECT);
PID pid_ventilacao(&input, &output_ventilacao, &setPoint_ventilacao, Kp, Ki, Kd, DIRECT);

void setup() {
    Serial.begin(9600); // Comunicação serial
    lcd.init();
    lcd.backlight();
    pinMode(RELE_RESISTENCIA, OUTPUT);  // Configura o pino da resistência
    pinMode(RELE_VENTILACAO, OUTPUT);   // Configura o pino da ventilação

    lcd.setCursor(5, 1);
    lcd.print("Bem Vindo!");
    delay(1000);
    lcd.clear();

    // Inicializar PID
    pid_resistencia.SetMode(AUTOMATIC);
    pid_resistencia.SetOutputLimits(0, 255); // Limites de saída do PID para resistência
    pid_ventilacao.SetMode(AUTOMATIC);
    pid_ventilacao.SetOutputLimits(0, 255);  // Limites de saída do PID para ventilação
}

void loop() {
    // Ler a temperatura atual
    temperatura = leer_termopar();
    input = temperatura;  // Atualizar o valor de entrada para o PID

    // Controle PID para resistência
    pid_resistencia.Compute();
    if (temperatura < setPoint_resistencia - 0.05) {
        // Ligar a resistência se a temperatura estiver abaixo de 49,5°C
        digitalWrite(RELE_RESISTENCIA, HIGH);
    } else if (temperatura > setPoint_resistencia + 0.05) {
        // Desligar a resistência se a temperatura estiver acima de 50,5°C
        digitalWrite(RELE_RESISTENCIA, LOW);
    }

    // Controle PID para ventilação
    pid_ventilacao.Compute();
    if (temperatura > setPoint_ventilacao) {
        // Ligar a ventilação se a temperatura estiver acima de 27°C
        digitalWrite(RELE_VENTILACAO, HIGH);
    } else {
        // Desligar a ventilação se a temperatura estiver abaixo de 27°C
        digitalWrite(RELE_VENTILACAO, LOW);
    }

    // Exibir temperatura setada e temperatura atual no LCD
    lcd.setCursor(0, 0);  // Primeira linha
    lcd.print("Setpoint: ");
    lcd.print(setPoint_resistencia, 1);  // Exibe a temperatura desejada (setpoint)

    lcd.setCursor(0, 1);  // Segunda linha
    lcd.print("Temp Real: ");
    lcd.print(temperatura, 1);  // Exibe a temperatura medida em tempo real

    // Enviar os dados para o monitor serial
    Serial.print("Temperatura: ");
    Serial.print(temperatura, 2);
    Serial.print(", Timestamp: ");
    Serial.println(millis());  // Tempo em milissegundos desde o início do programa

    delay(10000); // Aguardar 1 segundo antes da próxima leitura
}

// Função para ler a temperatura do termopar
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
