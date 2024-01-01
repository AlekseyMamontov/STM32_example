Linux

CANGAROO
https://github.com/normaldotcom/cangaroo

SavvyCAN
https://www.csselectronics.com/pages/can-bus-interface-savvycan


работа в терминале
sudo apt-get install can-utils
sudo ip link set can0 up type can bitrate 500000 (скорость на шине, можно от 10кб до 1Мб)
candump can0 (мониторинг шины, номер шины)

Windows 10
https://canable.io/utilities/cangaroo-win32-ccdcb64.zip

Прошивки
Canable  (SLCAND) https://github.com/normaldotcom/canable-fw?tab=readme-ov-file
