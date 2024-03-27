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

PWM1 50HZ 5% Duty cycle
```
root@orangepi:~# echo 3 > /sys/class/pwm/pwmchip0/export
root@orangepi:~# echo 20000000 > /sys/class/pwm/pwmchip0/pwm3/period
root@orangepi:~# echo 1000000 > /sys/class/pwm/pwmchip0/pwm3/duty_cycle
root@orangepi:~# echo 1 > /sys/class/pwm/pwmchip0/pwm3/enable
```

PWM2 50HZ 5% Duty cycle

```
root@orangepi:~# echo 4 > /sys/class/pwm/pwmchip0/export
root@orangepi:~# echo 20000000 > /sys/class/pwm/pwmchip0/pwm4/period
root@orangepi:~# echo 1000000 > /sys/class/pwm/pwmchip0/pwm4/duty_cycle
root@orangepi:~# echo 1 > /sys/class/pwm/pwmchip0/pwm4/enable
```

100% - остановка, 0% максимальная скорость
Необходимо подобрать частоту ШИМ такую, чтобы обеспечивался минимальный шум. 50гц

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