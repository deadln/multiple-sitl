# Описание
Этот форк позволяет запускать симуляцию gazebo с двумя разными типасм моделями, работающими на прошивке PX4 и управляемыми с помощью MAVROS.
# Установка
На вашем компьютере с ОС Ubuntu 20.04 должны быть установлены git (sudo apt install git) и [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).
Скачайте исходники PX4 с тегом vX.Y.Z и установите модуль sitl_gazebo
```
 git clone --branch vX.Y.Z https://github.com/PX4/PX4-Autopilot.git
 mv PX4-Autopilot Firmware
 cd Firmware
 git submodule update --init Tools/sitl_gazebo
```
Загрузите multiple-sitl с веткой multiple-models
```
cd Tools
git clone --branch multiple-models https://github.com/acsl-mipt/multiple-sitl.git
cd multiple-sitl/install
```
Установить окружение и симуляцию
```
./all.sh
cd ..
```
# Запуск симуляции
Для того чтобы запустить симуляцию с двумя моделями выполните:
```
./start_2models.rb -n количество_первых_моделей --gazebo_model первая_модель -n2 количество_вторых_моделей --gazebo_model2 вторая_модель
```
Например, запустить симуляцию с одним ровером и двумя коптерами
```
./start_2models.rb -n 1 --gazebo_model r1_rover -n2 2 --gazebo_model2 iris
```
