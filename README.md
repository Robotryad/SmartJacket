# SmartJacket
Проект по созданию "умной куртки"

## Аппаратная часть
+ Arduino LillyPad
+ DS18B20 (2 шт.)
+ GPS GY-NEO6MV2
+ GSM Sim800l
+ аксилерометр
+ элемент-пельтье


## Библиотеки
+ TinyGPS - GPS-модуль
+ DallasTemperature - датчик температуры DS18B20
+ OneWire - 1-Wire 
+ SoftwareSerial - вывод на сирийный порт, создание нескольких программных портов
+ Encoder = время DHT3231
+ RTC = время DHT3231

## Функции
+ TempBody() - Температура тела
+ TempOut() - Наружняя температура
+ GPS() - Получение координат
+ SMS(String text) - Отправка СМС
+ PrintSerial() - Вывод на монитор серийного порта

Переменные
// Температурные переменные
int tempBody, tempOut
bool tempHigh, tempLow
bool tempBodyHighCritic, tempBodyLowCritic
bool tempOutHighCritic, tempOutLowCritic

Переменные для GPS
bool newdata = false

