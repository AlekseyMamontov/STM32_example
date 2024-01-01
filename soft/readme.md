Linux

CANGAROO
https://github.com/normaldotcom/cangaroo

SavvyCAN
https://github.com/collin80/SavvyCAN

https://www.csselectronics.com/pages/can-bus-interface-savvycan



работа в терминале
sudo apt-get install can-utils
sudo ip link set can0 up type can bitrate 500000 (скорость на шине, можно от 10кб до 1Мб)
candump can0 (мониторинг шины, номер шины)




Windows 10
CANGAROO
https://canable.io/utilities/cangaroo-win32-ccdcb64.zip

SavvyCAN (через slcan)

https://github.com/collin80/SavvyCAN




Прошивки


candlelight Firmware

flash_usb_can_adapter_STM32F072.bin
-----------------------

Производительность выше, чем у прошивки с последовательным интерфейсом, поскольку slcand полностью обходится. 
С Linux и Socketcan вы можете использовать все стандартные утилиты командной строки can-utils и даже Wireshark для взаимодействия с шиной. 
https://github.com/candle-usb/candleLight_fw


Canable slcan firmware
-----------------------

Canable canable-.bin (slcan)
https://github.com/normaldotcom/canable-fw?tab=readme-ov-file
https://canable.io/getting-started.html

Данная прошивка позволит работать c SavvyCAN в Windows

