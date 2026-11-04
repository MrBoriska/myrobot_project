Протестировано Orange Pi Zero 3

## Зависимости:

- ros2 humble
- libboost-all-dev
- libi2c-dev

## Компоненты:
Orange Pi Zero 3
MPU9250
LDS01RR LIDAR (https://aliexpress.ru/item/1005004231727158.html?spm=a2g2w.orderdetail.0.0.4cd74aa6REUCSn&sku_id=12000028465958093)

## Конфигурирование OrangePi

OrangePi-Config:
```
enable: hardware->ph-pwm34
enable: hardware->ph-uart5
enable: hardware->ph-i2c3
```

## Подключение
### LDS01RR LIDAR 
https://aliexpress.ru/item/1005004231727158.html?spm=a2g2w.orderdetail.0.0.65dd4aa6eVYCE1&sku_id=12000028465958093
 подключается к UART5 (TX -> RX-PH3, VCC -> 5V, GND->GND). Двигатель запитывается от 3.3 вольт, для поддержания +- 300rpm отдельным источником питания (3.3V/0.5A)

Двигатели колес управляются двумя PWM аппаратно реализованых на пинах PWM3 (UART0 TX) и PWM4 (UART0 RX). Пример управления через терминал:

PWM1 25kHZ 5% Duty cycle
```
root@orangepi:~# echo 3 > /sys/class/pwm/pwmchip0/export
root@orangepi:~# echo 40000 > /sys/class/pwm/pwmchip0/pwm3/period
root@orangepi:~# echo 40000 > /sys/class/pwm/pwmchip0/pwm3/duty_cycle
root@orangepi:~# echo 1 > /sys/class/pwm/pwmchip0/pwm3/enable
```

PWM2 25kHZ 5% Duty cycle

```
root@orangepi:~# echo 4 > /sys/class/pwm/pwmchip0/export
root@orangepi:~# echo 40000 > /sys/class/pwm/pwmchip0/pwm4/period
root@orangepi:~# echo 40000 > /sys/class/pwm/pwmchip0/pwm4/duty_cycle
root@orangepi:~# echo 1 > /sys/class/pwm/pwmchip0/pwm4/enable
```

100% - остановка, 0% максимальная скорость
Необходимо подобрать частоту ШИМ такую, чтобы обеспечивался минимальный шум. 25кГц выше порога в 20кГц, при котором человек слышит писк.

### Мотор-колесо
https://aliexpress.ru/item/1005006213650744.html?spm=a2g2w.orderdetail.0.0.901d4aa6gsSRwk&sku_id=12000036308723032: 

- красный/черный питание 6-12 вольт
- синий - управление ШИМ (5 вольт)
- коричневый - обратная связь, необходимо подтяжка к 5в. (выдает частоту. максимальная скорость без нагрузки соответсвует примерно 1.6кГц).
- белый - переключение направления 0в/+5в
- зеленый - тормоз 0в/+5в

```
PC8
echo 72 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio72/direction
echo falling > /sys/class/gpio/gpio72/edge
gpio mode 8 up
```
```
PC10
echo 74 > /sys/class/gpio/export
echo in > /sys/class/gpio/gpio74/direction
echo falling > /sys/class/gpio/gpio74/edge
gpio mode 16 up
```

get counters:
```
cat /proc/interrupts | grep gpiolib

 69:     258617          0          0          0  sunxi_pio_edge  40 Edge      gpiolib
 71:      24687          0          0          0  sunxi_pio_edge  42 Edge      gpiolib
```

Направление:
PC5
```
echo 69 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio69/direction
echo 1 > /sys/class/gpio/gpio69/value
```
PC6
```
echo 70 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio70/direction
echo 1 > /sys/class/gpio/gpio70/value
```
Тормоз:
PH9
```
echo 233 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio233/direction
echo 1 > /sys/class/gpio/gpio233/value
```



## Запуск

```
colcon build
sudo chmod 0666 /dev/ttyS5
ros2 run xv_11_driver xv_11_driver --ros-args -p port:=/dev/ttyS5 -p frame_id:=xv11_scan
```

## SLAM

Пакет `src/myrobot_custom_slam` — собственный SLAM на IMU + лидаре по
формулировке FAST-LIO2, оптимизированный под Orange Pi Zero 3 (подробности,
замеры и тюнинг: [docs/slam_design.md](docs/slam_design.md)).

Коротко: iterated error-state Kalman filter (18 DoF), распространение по IMU на
100 Гц, прямой point-to-line скан-матчинг по сырому скану, компенсация искажений
скана (deskew) по истории поз, ограниченная по памяти хэш-сетка 0.1 м вместо
дерева, проекция на SE(2) (одноплоскостной лидар не наблюдает z/roll/pitch).
Публикует `map -> odom`, `nav_msgs/OccupancyGrid` на `/map` и
`/slam/registered_scan`.

Полный стек на роботе:

```
ros2 launch myrobot_custom_slam slam.launch.py use_rviz:=true
```

Отладка без робота — три способа (подробно в
[docs/slam_design.md](docs/slam_design.md), §8):

```
# 1. синтетические сенсоры, без Gazebo и без GPU: комната + истинная поза
ros2 launch myrobot_custom_slam debug_fake.launch.py
ros2 run myrobot_custom_slam check_slam_accuracy.py --duration 30 --verbose

# 2. Gazebo (headless: gui:=false запускает только gzserver)
ros2 launch myrobot_custom_slam slam_gazebo.launch.py gui:=false rviz:=false

# 3. dev-контейнер (собирает gazebo_ros, xacro, libi2c-dev, монтирует воркспейс)
./create_docker.sh build
./create_docker.sh sim      # синтетика + SLAM
./create_docker.sh check    # + отчёт по точности
./create_docker.sh gazebo   # Gazebo server + SLAM
./create_docker.sh shell    # интерактивно, с X11
```

На синтетике точность закрытого контура (30 с, круг 1 м, шум лидара 1 см,
ошибка масштаба одометрии 2 %): средняя ошибка позы 0.67 см, худшая 1.34 см,
курс 0.12°/0.29° — систематического дрейфа нет.

Отдельно (если драйверы уже подняты):

```
ros2 run myrobot_custom_slam slam_node --ros-args \
  --params-file $(ros2 pkg prefix myrobot_custom_slam)/share/myrobot_custom_slam/config/slam.yaml
```

Тесты и бенчмарк ядра:

```
colcon test --packages-select myrobot_custom_slam
ros2 run myrobot_custom_slam bench_slam 720 300
```

Важно: `myrobot_driver` публикует одометрию как `odom -> base_link` (стандарт
REP-105), имена джойнтов в `joint_states` — `base_to_l_wheel` /
`base_to_r_wheel`, геометрия (`wheel_radius`, `track`) вынесена в параметры
(по умолчанию — как в URDF: 0.0315 м и 0.17 м). Если реальные колёса другие —
задать через `--ros-args -p wheel_radius:=... -p track:=...`, иначе одометрия и
`map -> odom` будут систематически смещены.
