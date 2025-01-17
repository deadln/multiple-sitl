# Описание
Этот форк позволяет запускать симуляцию gazebo с двумя разными типасм моделями, работающими на прошивке PX4 и управляемыми с помощью MAVROS. Оригинальный [README](https://github.com/deadln/multiple-sitl/blob/develop/README).
# Установка
На вашем компьютере с ОС Ubuntu 20.04 должны быть установлены pip3 (sudo apt install python3-pip), git (sudo apt install git), [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) и набор библиотек Ignition Common (sudo apt install libignition-common3-dev). При установке ROS не забудьте прописать в консоли автозагрузку окружения:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Установите необходимые библиотеки для Python:
```
pip3 install toml pyros-genmsg packaging
```
Установите пакет gazebo_ros:
```
ros-noetic-gazebo-ros
```
Скачайте исходники PX4 с тегом версии vX.Y.Z и установите модуль sitl_gazebo
```
 git clone --branch vX.Y.Z https://github.com/PX4/PX4-Autopilot.git
 mv PX4-Autopilot Firmware
 cd Firmware
 git submodule update --init Tools/sitl_gazebo
```
Загрузите multiple-sitl с веткой multiple-models
```
cd Tools
git clone --branch multiple-models https://github.com/deadln/multiple-sitl.git
cd multiple-sitl/install
```
Установить окружение и симуляцию
```
./all.sh
cd ..
```
Импортировать модели мира 10hect.world:
```
cp -r models/x_balk/ /usr/share/gazebo-11/models/
cp -r models/y_balk/ /usr/share/gazebo-11/models/
cp -r models/grass_plane/ /usr/share/gazebo-11/models/
```

# Запуск симуляции
Для того чтобы запустить симуляцию с двумя моделями выполните:
```
./start_2models.rb -n количество_первых_моделей --gazebo_model первая_модель -n2 количество_вторых_моделей --gazebo_model2 вторая_модель [файл мира gazebo]
```
Примечание: для того чтобы в симуляции был приемлемый Real Time Factor рекомендуется загружать первой моделью менее стабильную и оптимизированную. Например, запустить симуляцию с одним ровером и двумя коптерами:
```
./start_2models.rb -n 1 --gazebo_model r1_rover -n2 2 --gazebo_model2 iris
```
Чтобы остановить симуляцию выполните:
```
./kill_sitl.sh
```
# Программное взаимодействие
Управление моделями осуществляется с помощью пакета MAVROS, передающего в симуляцию MAVLINK сообщения. Для каждой модели создаётся MAVROS-нода с именами:
* Для первой модели: mavros[1..n]
* Для второй модели: mavros[n+1..n+n2-1]
# Облёт поля
Для запуска симуляции облёта и объезда поля выполните команду:
```
./start_2models.rb -n 1 --gazebo_model r1_rover -n2 2 --gazebo_model2 iris --debug 10hect.world
```
После того как мир и модели загрузятся, выполнить:
```
python3 field_flyover.py
```
и ввести требуемые данные.
## Описание облёта поля
Программа полёта принимает на вход количество коптеров и роверов и два угла поля, после чего строит маршрут облёта и объезда поля зигзагом вдоль оси Y и распределяет точки маршрута между моделями. Для того чтобы облететь и объездить поле 10 гектар из файла 10hect.world необходимо прописать:
* Количество дронов: 2
* Количество роверов: 1
* Первый угол поля: 0 0
* Первый угол поля: 250 400
## Важно
Плагин определения местоположения модели r1_rover работает крайне нестабильно, и на поворотах он может перестать адекватно работать. Из-за этого скрипт облёта поля использует для ориентации ровера в пространстве данные о положении модели из самого gazebo. Однако через некоторое время после того как плагин перестаёт нормально работать ровер может остановиться или резко сбиться с траектории. Чтобы этого избежать, выполняйте следующие действия:
1) Всегда запускайте симуляцию с ровером с параметром --debug чтобы иметь доступ к консоли PX4
2) Найдите консоль ровера. Если вы запускали симуляцию по примеру выше, то консоль будет называться <b>px4-1</b>
![консоль ровера](https://github.com/deadln/multiple-sitl/blob/multiple-models/rover_terminal.png)
4) Если ровер остановился или резко поехал в случайную сторону, пропишите в консоли эти команды:
```
ekf2 stop
ekf2 start
```
После этого ровер вернётся к своему прежнему маршруту. Также вы можете автоматизировать эти действия:
1) Откройте отдельное окно терминала и перейдите в папку со скриптом облёта поля
2) Запустите скрипт:
```
sudo python3 ekf2_restart.py [периодичность_перезапуска]
```
3) Выделите консоль ровера. Теперь каждые 30 (или указанный период) секунд скрипт будет перезапускать систему ориентации ровера.
