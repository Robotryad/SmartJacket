# SmartJacket
Проект по созданию "Умной куртки"
Разработчики: Суслова Яна, Воробьёв Артём, Старинин Андрей (Robotryad).
Год начала разработки - 2017

## Аппаратная часть
+ Arduino LillyPad
+ DS18B20 (2 шт.)
+ GPS GY-NEO6MV2
+ GSM Sim800l
+ аксилерометр MPU6050
+ элемент-пельтье(4 шт.)


## Библиотеки
+ TinyGPS - GPS-модуль
+ DallasTemperature - датчик температуры DS18B20
+ OneWire - 1-Wire - для температуры
+ include <SD.h> - карта памяти
+ SoftwareSerial - вывод на сирийный порт, создание нескольких программных портов


## Функции
+ TempBody() - Температура тела
+ TempOut() - Наружняя температура
+ GPS() - Получение координат
+ SMS(String text) - Отправка СМС
+ PrintSerial() - Вывод на монитор серийного порта
+ Temp() - вся температура
+ FileSd() - сохрание на карту памяти
+ ACCEL() - аксилерометр

## Переменные
### Температурные переменные
* int tempBody - температура тела
* int tempOut - наружняя температура
* int tempBody1 - температура тела
* int tempOut1 - наружняя температура
* bool tempHigh - подогрев
* bool tempLow - охлаждение
* bool tempBodyHighCritic - критически-высокая температура тела 
* bool tempBodyLowCritic - критически-низкая температура тела
* bool tempOutHighCritic - критически-высокая наружня температура 
* bool tempOutLowCritic - критически-низкая наружня температура

### Переменные для GPS
* int b = gpsSerial.read() - в TinyGPS есть ошибка: не обрабатываются данные с \r и \n

* bool newdata = false
* unsigned long start;
* long lat, lon;
* unsigned long time, date;

