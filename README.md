Описание
========

Пакет предназначен для развертывания на Ubuntu 20.04 (Focal Fossa)

Рекомендуется использовать "чистую" ОС с утановленными обновлениями и драйверами (install third-party software for graphics and Wi-Fi harware and additional media formats)

Для своей работы использует:
- библиотеки и окружение ROS Noetic (https://www.ros.org) ([Подробный учебник ROS](http://wiki.ros.org/ROS/Tutorials))
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

Обновление локального репозитория (получение изменений из удаленного репозитория; команда выполняется из директории drone-games)

```
git pull
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

Тестовые задания
================

[Тестовое задание по дисциплине Синхронный полет](https://github.com/acsl-mipt/drone-games/blob/main/TASK.md)
