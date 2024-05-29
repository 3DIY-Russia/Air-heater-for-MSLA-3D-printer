#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "GyverPID.h"
#include "PIDtuner2.h"
PIDtuner2 tuner;

#include "GyverOLED.h"
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled;

#define ONE_WIRE_BUS 2
#define BUTTON_PIN 3
#define RELAY_PIN 4
#define FAN_PIN 13

unsigned long last_time;            // в этой переменной будем хранить время, когда светодиод последний раз мигнул
const long period = 10000;          // интервал времени в миллисекундах, через который светодиоду нужно мигать

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// PID tuning parameters
GyverPID regulator(82, 12, 370);

bool heaterState = false;
bool buttonState = false;
bool lastButtonState = false;
unsigned long buttonPressTime = 0;
unsigned long lastButtonPressTime = 0;
unsigned long longPressDuration = 8000; // Duration for long press in milliseconds

int buttonPressCount = 0;
unsigned long maxPressInterval = 1000; // Maximum interval between presses for it to be considered a sequence

void setup() {
  oled.init();        // инициализация
  oled.clear();       // очистка

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(FAN_PIN, OUTPUT);
  Serial.begin(9600);
  sensors.begin();

  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  regulator.setLimits(0, 255);    // пределы (ставим для 8 битного ШИМ). ПО УМОЛЧАНИЮ СТОЯТ 0 И 255
  regulator.setpoint = 35;        // сообщаем регулятору температуру, которую он должен поддерживать

  tuner.setParameters(NORMAL, 0, 255, 6000, 0.05, 500);

}

void loop() {
  oled.circle(112, 32, 15, OLED_CLEAR);           // четвёртый аргумент: параметр фигуры
  sensors.requestTemperatures();
  regulator.input = sensors.getTempCByIndex(0);

  oled.setScale(3);   // масштаб текста (1..4)
  oled.home();        // курсор в 0,0
  oled.print(regulator.input);

  oled.setScale(1);
  // курсор на начало 3 строки
  oled.setCursor(0, 3);
  oled.print("темп. текущая");

  oled.setScale(2);
  oled.setCursorXY(0, 35);
  oled.print(regulator.setpoint);

  oled.setScale(1);
  oled.setCursorXY(0, 55);
  oled.print("темп. целевая");

  //oled.roundRect(105, 0, 127, 22, OLED_STROKE);  // прямоугольник скруглённый (лев. верхн, прав. нижн)

  //oled.setCursorXY(15, 55);
  //oled.print("нагрев ");
  //oled.print(heaterState ? "включен" : "выключен");

  // Read button state
  buttonState = !digitalRead(BUTTON_PIN); // Invert the logic due to INPUT_PULLUP

  // Check for button press and release
  if (buttonState && !lastButtonState) {
    buttonPressTime = millis(); // Record the time when button is pressed
  }

  if (!buttonState && lastButtonState) {
    unsigned long pressDuration = millis() - buttonPressTime;
    unsigned long timeSinceLastPress = millis() - lastButtonPressTime;

    // Check for long press
    if (pressDuration >= longPressDuration) {
      Serial.println("Долгое нажатие распознано. АвтокалибровкаPID");
      tuner.setInput(sensors.getTempCByIndex(0));
      tuner.compute();
      analogWrite(RELAY_PIN, tuner.getOutput());
      tuner.debugText();

  // выводит в порт текстовые отладочные данные, включая коэффициенты
  tuner.debugText();
    } else {
      // Check for triple press
      if (timeSinceLastPress < maxPressInterval) {
        buttonPressCount++;
        if (buttonPressCount == 3) {
          regulator.setpoint += 5.0;
          if (regulator.setpoint > 40.0) regulator.setpoint = 40.0;
          Serial.print("Тройное нажатие распознано. Установка изменена на ");
          Serial.print(regulator.setpoint);
          Serial.println(" C.");
          buttonPressCount = 0;
        }
      } else {
        buttonPressCount = 1;
      }
      lastButtonPressTime = millis();
      heaterState = !heaterState;
      digitalWrite(RELAY_PIN, heaterState ? HIGH : LOW);
      Serial.print("Короткое нажатие распознано. Нагрев: ");
      Serial.println(heaterState ? "вкл" : "выкл");
    }
  }
  lastButtonState = buttonState;

  // Run PID control if heater is on
  if (heaterState) {
    oled.circle(112, 32, 15, OLED_FILL);           // четвёртый аргумент: параметр фигуры
    analogWrite(RELAY_PIN, regulator.getResultTimer());
    digitalWrite(FAN_PIN, HIGH);
  } else {
    oled.circle(117, 32, 5, OLED_FILL);              // четвёртый аргумент: параметр фигуры
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(FAN_PIN, LOW);
  }

  // Serial output for debugging
  Serial.print("Текущая температура: ");
  Serial.print(regulator.input);
  Serial.print(" C, нагрев: ");
  Serial.print(heaterState ? "вкл" : "выкл");
  Serial.print(", PID инфо: ");
  Serial.print(regulator.output);
  Serial.print(", установка на: ");
  Serial.println(regulator.setpoint);

  delay(500);
}