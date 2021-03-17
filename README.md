Описание
========

Пакет предназначен для развертывания на Ubuntu 20.04 (Focal Fossa)

Рекомендуется использовать "чистую" ОС с утановленными обновлениями и драйверами (install third-party software for graphics and Wi-Fi harware and additional media formats)

Для своей работы использует:
- библиотеки и окружение ROS Noetic (https://www.ros.org)
- программный пакет 3D симуляции и моделирования Gazebo (http://gazebosim.org)
- код открытого автопилота PX4 (https://px4.io)

Содержит:
- скрипты запуска симуляции в Gazebo
- модели объектов и описание миров для Gazebo
- примеры кода для запуска группы аппаратов
- служебные файлы и исполняемые скрипты

Минимальные требования к оборудованию : CPU Intel Core i7 4770, RAM 8Гб, Nvidia GTX650 1Гб


Подготовка к установке
======================

Установка Git

```
sudo apt install git
```

Скачивание репозитория

```
git clone https://github.com/acsl-mipt/drone-games.git
```

Переход в скачанный репозиторий

```
cd drone-games/
```


Установка
=========

Для установки всех зависимостей и компиляции (сборки) необходимых файлов:

```
./install.sh
```

Если в системе уже установлены все зависимости:

```
./install.sh build
```

После установки необходимо перезапустить терминал для продолжения работы (закрыть и открыть новый)


Проверка работоспособности
==========================

Запуск с моделью аппарата iris в количестве 1 шт.:

```
./start.sh iris 1
```

После успешного выполнения скрипта вы должны увидеть следующее:

![Start](https://github.com/acsl-mipt/drone-games/blob/main/.imgs/start.png)


Отановка

```
./stop.sh
```

Помощь по имеющимся аргументам командной строки:

```
./start.sh iris 1 --help
```

Работа с группой аппаратов
==========================

Пример с коптерами:

```
./test-group.sh
```

Пример с планерами вертикального взлета и посадки (VTOL):

```
./test-group.sh vtol
```


Тестовое окружение симулятора
=============================

Обратите внимание на то, что точкой (0; 0) в локальной системе координат аппарат считает ту точку, что вы передали в опции --ref_point. То есть она вовсе не обязательно должна совпадать с нулем среды симуляции.

#### Стадион (формация)

Запуск для пролета по стадиону:

```
./formation.sh
```

После успешного выполнения скрипта вы должны увидеть следующее:

![Formation](https://github.com/acsl-mipt/drone-games/blob/main/.imgs/formation.png)

Зелеными полупрозрачными блоками обозначены зоны, в которых необходимо двигаться в формации, желтыми - в которых должно происходить перестроение.

Длина зеленого блока - 124 метра, желтого - 102 метра, ширина и высота обоих - 20 метров и 50 метров, соответственно.

Центры зеленых блоков расположены по следующим координатам: (41; 0) и (-41; 0), желтых: (0; 72) и (0; -72).

При запуске скрипта ./formation.sh также запускается ROS-нода, выдающая в топик /formations_generator/formation текущие относительные координаты для построения формации в строковом формате: "НАЗВАНИЕ\_ФОРМАЦИИ КООРДИНАТА\_X\_1 КООРДИНАТА\_Y\_1 КООРДИНАТА\_Z\_1 КООРДИНАТА\_X\_2 КООРДИНАТА\_Y\_2 КООРДИНАТА\_Z\_2 ...". Для тестового задания формаций четыре: 

* первая формация - буква Т: (0, -3, 11), (0, -1, 11), (0, 1, 11), (0, 3, 11), (0, 0, 8), (0, 0, 5)
* вторая - Е: (0, 0, 11), (0, 3, 11), (0, 0, 8), (0, 3, 8), (0, 0, 5), (0, 3, 5)
* третья - С: (0, 1, 9), (0, 0, 11), (0, -2, 9), (0, -2, 7), (0, 0, 5), (0, 1, 6) 

Обратите внимание, что координаты относительные.

#### Лабиринт (гонка)

Запуск для пролета по лабиринту:

```
./race.sh
```

После успешного выполнения скрипта вы должны увидеть следующее:

![Race](https://github.com/acsl-mipt/drone-games/blob/main/.imgs/race.png)

