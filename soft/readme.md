## flash

### for usbcan stm32f072      

https://github.com/AlekseyMamontov/STM32_example/blob/main/soft/CAN2.0_STD_Flash_5_variations_.zip

### for usbcanfd solo, dual 

https://github.com/AlekseyMamontov/STM32_example/blob/main/soft/CANFD_SOLO_DUAL_flash_4_variations.zip

-------------------------------------------
## SOFT 

https://github.com/AlekseyMamontov/CANnectivity-CANFD-adapters

*CAN Bus Software Overview* 

# SocketCAN, can-utils

### sudo apt-get install can-utils

### sudo ip link set can0 up type can bitrate 500000 (скорость на шине, можно от 10кб до 1Мб)

candump can0 (мониторинг шины, номер шины)

<img width="50"  alt="image" src="https://github.com/user-attachments/assets/4599cf1c-6836-4b64-87ad-56a6bc628432" />

https://www.kernel.org/doc/html/latest/networking/can.html

https://github.com/linux-can



# <img src="https://github.com/Schildkroet/CANgaroo/raw/master/src/assets/cangaroo.png" width="48" height="48"> CANgaroo

Jayachandran Dharuman (https://github.com/OpenAutoDiagLabs/cangaroo)

<img width="500"  alt="image" src="https://github.com/user-attachments/assets/581d26fd-8e25-40ec-a364-900e12229ea9" />

Schildkroet (https://github.com/Schildkroet/CANgaroo)

<img width="500"  alt="image" src="https://github.com/user-attachments/assets/8da40527-1ded-48d2-a7d4-de4685cd5034" />

Wikilift (https://github.com/wikilift/CANgaroo

<img width="500"  alt="image" src="https://github.com/user-attachments/assets/f8424309-0d9e-4b3b-a7cb-6bd4fdba01cb" />

#  SavvyCAN

<img width="500"  alt="image" src="https://github.com/user-attachments/assets/b4a26fa0-ea29-4713-bd0c-ee2b131f507d" />

https://github.com/collin80/SavvyCAN


# <img width="200"  alt="image" src="https://github.com/user-attachments/assets/7878c9a6-fd5d-461f-a3a0-53f5431f9938" />  

https://www.wireshark.org/tools/

<img width="600" alt="image" src="https://github.com/user-attachments/assets/4ab0cf8e-a04c-4f79-9e5e-7fb9f0894ae9" />


# CAN Analyzer

https://github.com/phnahes/can-bus-analyzer

<img width="500"  alt="image" src="https://github.com/user-attachments/assets/09a1846d-35a9-4b4b-a970-f32ae07cef1d" />


<img width="500"  alt="image" src="https://github.com/user-attachments/assets/c630a2d3-ac7f-4b76-a1b5-97889007edbf" />


# 🖥️ RCAN Tools

https://rcantools.com/index.html  
Proprietary commercial software (not open source). SocketCAN native with CAN FD support.

# CANopen lib

https://github.com/CANopenNode

https://github.com/CANopenNode/CANopenEditor

https://github.com/CANopenNode/CanOpenSTM32

 python-can

https://github.com/hardbyte/python-can
https://github.com/cantools/cantools
https://github.com/canopen-python/canopen CANopen

# QT can-bus-api
https://www.qt.io/blog/qt-can-bus-api-extensions


CANGAROO https://github.com/OpenAutoDiagLabs/CANgaroo/releases/tag/v0.10.0


SavvyCAN

https://github.com/collin80/SavvyCAN (https://appimage.github.io/SavvyCAN/);

https://www.csselectronics.com/pages/can-bus-interface-savvycan

https://appimage.github.io/SavvyCAN/


WireShark

https://www.wireshark.org/


Прошивки

----------------------------
candlelight Firmware

flash_usb_can_adapter_STM32F072.bin
-----------------------

Производительность выше, чем у прошивки с последовательным интерфейсом, поскольку slcand полностью обходится. 
С Linux и Socketcan вы можете использовать все стандартные утилиты командной строки can-utils и даже Wireshark для взаимодействия с шиной. 
https://github.com/candle-usb/candleLight_fw

--------------------------------

Canable slcan firmware
-----------------------

Canable canable-.bin (slcan)

https://github.com/normaldotcom/canable-fw?tab=readme-ov-file

info

https://canable.io/getting-started.html

Данная прошивка позволит работать c SavvyCAN в Windows

---------------------------------------------------

Поддержка Python

https://python-can.readthedocs.io/

Для еще большей гибкости библиотека python-can позволяет напрямую общаться с CAN-шиной из Python . Библиотека является кроссплатформенной и может подключаться напрямую к интерфейсу виртуального последовательного порта CANable или собственному интерфейсу SocketCan. Всего с помощью пары строк кода вы можете декодировать трафик на шине, отправлять сообщения и многое другое.

Посетите страницу начала работы для получения дополнительной информации.

----------------------------------------------------


Поддержка Javascript (NodeJS, Linux -> Electron)



