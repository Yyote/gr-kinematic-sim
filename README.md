# README

## Установка

Зайдите в `gr-kinematic-sim/` и выполните следующие команды

```shell
sudo apt update
```

```shell
sudo apt install ros-$ROS_DISTRO-tf-transformations python3-pip
pip3 install -r requirements.txt
```
Если установка зависимостей прошла успешно, соберите воркспэйс:
```shell
colcon build
```

## Запуск симулятора 

Чтобы запустить симулятор, выполните слудющие команды:

```
source install/setup.bash
ros2 run gr_kinematic_sim sim
```

## Создание сценариев

Чтобы создать сценарий с заранее заданными моделями, позициями роботов и картой, используйте launch-файлы. Пример такого лаунч файла можете найти в `gr-kinematic-sim/src/gr_kinematic_sim/launch/test_scenario.launch.py`.

Чтобы заупстить launch-файл сценария, выполните следующие команды:

```
source install/setup.bash
ros2 launch gr_kinematic_sim test_scenario.launch.py
```

## Лицензия

Лицензия

Этот проект распространяется по лицензии Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0). © 2025 Yyote (yyootttaa@gmail.com)

