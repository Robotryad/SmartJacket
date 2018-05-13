/*
   SmartJacket (Умная одежда)
   Проект умной одежды

   Версия: 0.1 (апрель 2018)

   (c) Суслова Яна,Воробьёв Артём,Старинин Андрей, 2018 (Robotryad)

  MIT License

  Copyright (c) 2018 Robotryad (Суслова Яна,Воробьёв Артём,Старинин Андрей, 2018)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

// подключение библиотек
#include <TinyGPS.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SoftwareSerial.h>
#include <iarduino_RTC.h>

//TIME
iarduino_RTC timer(RTC_DS3231);

// GPS
#define OK 1
#define NOTOK 2
#define RST 3

//TEMP
#define tempBodyPin 4 // Пин подключения датчика температуры тела
#define tempOutPin 5 // Пин подключения датчика наружной температуры
#define powerTempBodyPin 6 // Пин управления питанием датчика температуры тела
#define powerTempOutPin 7 // Пин управления питанием датчика наружной температуры
#define powerEPHeatPin 12 // Пин подключения элемента-пельтье (для подогрева)
#define powerEPCoolPin 13 // Пин подключения элемента-пельтье (для охлождения)


// Константы критических температур
#define TEMPBODYHIGHCRITIC 38 //критически-высокая температура тела
#define TEMPBODYLOWCRITIC 34 //критически-низкая температура тела
#define TEMPOUTHIGHCRITIC 40 //критически-высокая наружня температура
#define TEMPOUTLOWCRITIC -20 //критически-низкая наружня температура

// Инициализируем GPS
SoftwareSerial gpsSerial(10, 11); // 10,11 пины для подключения gps модуля (10-RX; 11-TX)
TinyGPS gps;

// Инициализируем GSM
SoftwareSerial SIM800(8, 9); // 8 - RX Arduino (TX SIM800L), 9 - TX Arduino (RX SIM800L)

// Температурные переменные
int tempBody, tempOut;
bool tempHigh, tempLow;
bool tempBodyHighCritic, tempBodyLowCritic;
bool tempOutHighCritic, tempOutLowCritic;

// Переменные для GPS
bool newdata = false;
unsigned long start;
long lat, lon;
unsigned long time, date;

void setup() {
  delay(300);
  gpsSerial.begin(9600); // скорость обмена с GPS-приемником
  SIM800.begin(9600);//скорость обмена с GPS-приемником
  Serial.begin(9600);//скорость обмена серийным портом
  timer.begin();
  timer.settime(0, 50, 17, 14, 04, 18, 6); // 0  сек, 51 мин, 21 час, 27, октября, 2015 года, вторник
  pinMode (tempBodyPin, INPUT);//измерение температуры тела - в сирийный порт
  pinMode (tempOutPin, INPUT);//измерение температуры воздуха - в сирийный порт
  pinMode (powerTempBodyPin, OUTPUT);//вывод температуры тела (элемент-пельтье)
  pinMode (powerTempOutPin, OUTPUT);//вывод температуры воздуха ()
}

void loop() {
  GPS(); //Получение координат

  tempBody = TempBody(); //Считывание температуры тела
  tempOut = TempOut();//Считывание температуры воздуха
  tempLow = false;   //Охлаждение
  tempHigh = false;   //Подогрев
  tempBodyHighCritic = false;   //Критически высокая температура тела
  tempBodyLowCritic = false;  //Критически низкая температура тела
  tempOutHighCritic = false;   //Критически высокая температура на улице
  tempOutLowCritic = false;   //Критически низкая температура на улице

  if (tempBody > tempOut) {
    //Если температура тела больше температуры воздуха - устанавливаем tempLow в true
    tempLow = true;
  }
  if (tempBody < tempOut) {
    //Если температура тела больше температуры воздуха - устанавливаем tempHigh в true
    tempHigh = true;
  }
  if (tempBody > TEMPBODYHIGHCRITIC) {
    tempBodyHighCritic = true;                 // если температура тела больше чем критическая (высокая) температура тела, то tempBodyHighCritic в true
  }
  if (tempBody < TEMPBODYLOWCRITIC) {
    tempBodyLowCritic = true;                 // если температура тела ниже чем критическая (низкая) температура тела, то tempBodyLowCritic в true
  }
  if (tempOut > TEMPOUTHIGHCRITIC) {
    tempOutHighCritic = true;                 // если наружная температура больше чем критическая (высокая) температура наружности, то tempOutHighCritic в true
  }
  if (tempOut < TEMPOUTLOWCRITIC) {
    tempOutLowCritic = true;                  // если наружная температура ниже чем критическая (низкая) температура наружности, то tempOutLowCritic в true
  }

  if (tempLow) {
    digitalWrite(powerEPCoolPin, HIGH);
    digitalWrite(powerEPHeatPin, LOW);
  } // включение элемент-пельтье на охлаждение
  if (tempHigh) {
    digitalWrite(powerEPCoolPin, LOW);
    digitalWrite(powerEPHeatPin, HIGH);
  } // включение элемент-пельтье на подогрев
  if (tempHigh && tempOutLowCritic) {
    digitalWrite(powerEPCoolPin, LOW);
    digitalWrite(powerEPHeatPin, HIGH);
    SMS("Температура наружности критическая!");
  }
  if (tempLow && tempOutHighCritic) {
    digitalWrite(powerEPCoolPin, HIGH);
    digitalWrite(powerEPHeatPin, LOW);
    SMS("Температура наружности критическая!");
    //отправляем тревожный сигнал
  }
  if (tempHigh && tempBodyLowCritic) {
    SMS("Температура тела критическая! Переохлаждение!");
    digitalWrite(powerEPHeatPin, HIGH);
    digitalWrite(powerEPCoolPin, LOW);
    //Отправка тревожного сигнала и включение Элементов-Пельтье на подогрев
  }
  if (tempLow && tempBodyHighCritic) {
    SMS("Температура тела критическая! !Перегрев");
    digitalWrite(powerEPCoolPin, HIGH);
    digitalWrite(powerEPHeatPin, LOW);
    //Отправка тревожного сигнала и включение Элементов-Пельтье на охлаждение
  }
  //Время
if ((millis() % 1000) == 0) { // если прошла 1 секунда
   Serial.println (timer.gettime("H:i:s")); // выводим время
  delay(1); // приостанавливаем на 1 мс, чтоб не выводить время несколько раз за 1мс
  }
}


// Температура тела
int TempBody() {
  OneWire oneWire(tempBodyPin);
  DallasTemperature sensorTempBody(&oneWire);
  sensorTempBody.begin();
  digitalWrite(powerTempBodyPin, HIGH);
  sensorTempBody.requestTemperatures();
  delay(500);
  sensorTempBody.requestTemperatures();
  int tempBody1 = round(float(sensorTempBody.getTempCByIndex(0)));
  digitalWrite(powerTempBodyPin, LOW);
  return tempBody1;
}

// Температура наружняя
int TempOut() {
  OneWire oneWire1(tempOutPin);
  DallasTemperature sensorsTempOutPin(&oneWire1);
  sensorsTempOutPin.begin();
  digitalWrite(powerTempOutPin, HIGH);
  sensorsTempOutPin.requestTemperatures();
  delay(500);
  sensorsTempOutPin.requestTemperatures();
  int tempOut1 = round(float(sensorsTempOutPin.getTempCByIndex(0)));
  digitalWrite(powerTempOutPin, LOW);
  return tempOut1;
  
}

//акселерометр
void accel() {
   long int t_next;
   long int t = millis();
  if ( t_next < t ) {
    int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
    float ay, gx;
      long int T_OUT;
   t_next = t + T_OUT;
    accel.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

    // сырые данные акселерометра нужно преобразовать в единицы гравитации
    // при базовых настройках 1G = 4096
    ay = ay_raw / 4096.0;
    // на случай, если на акселерометр действуют посторонние силы, которые могут
    // увеличить ускорение датчика свыше 1G, установим границы от -1G до +1G
    ay = clamp(ay, -1.0, 1.0);

    // функция acos возвращает угол в радианах, так что преобразуем
    // его в градусы при помощи коэффициента TO_DEG
    if ( ay >= 0) {
      angle_ax = 90 - TO_DEG * acos(ay);
    } else {
      angle_ax = TO_DEG * acos(-ay) - 90;
    }
  }
}

// Получение координат
void GPS() {
  // задержка в секунду между обновлениями координат
  if (millis() - start > 1000)
  {
    newdata = readgps();
    if (newdata)
    {
      start = millis();
      gps.get_position(&lat, &lon);
      gps.get_datetime(&date, &time);
      Serial.print("Lat: "); Serial.print(lat);
      Serial.print(" Long: "); Serial.print(lon);
      Serial.print(" Date: "); Serial.print(date);
      Serial.print(" Time: "); Serial.println(time);
    }
  }
}
bool readgps()
{
  while (gpsSerial.available())
  {
    int b = gpsSerial.read();
    //в TinyGPS есть ошибка: не обрабатываются данные с \r и \n
    if ('\r' != b)
    {
      if (gps.encode(b))
        return true;
    }
  }
  return false;
}

// Отправка СМС
void SMS(String text) {
  // Устанавливает текстовый режим для SMS-сообщений
  SIM800.println("AT+CMGF=1");
  delay(300); // даём время на усваивание команды
  SIM800.println("AT+CSCS=\"GSM\"");
  delay(300);
  // Устанавливаем адресата: телефонный номер в международном формате
  SIM800.print("AT+CMGS=\"");
  SIM800.print("+79042144491");
  SIM800.println("\"");
  delay(300);
  SIM800.print(text);
  delay(300);
  // Отправляем Ctrl+Z, обозначая, что сообщение готово
  SIM800.print((char)26);
  delay(300);
  Serial.println("SMS send finish");
}

// Вывод на монитор серийного порта
void PrintSerial() {
  Serial.print("tempLow");
  Serial.println(tempLow);
  Serial.print("tempHigh");
  Serial.println(tempHigh);
  Serial.print("tempBodyHighCritic");
  Serial.println(tempBodyHighCritic);
  Serial.print("tempBodyLowCritic");
  Serial.println(tempBodyLowCritic);
  Serial.print("tempOutHighCritic");
  Serial.println(tempOutHighCritic);
  Serial.print("tempOutLowCritic");
  Serial.println(tempOutLowCritic);
}
