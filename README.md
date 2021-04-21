Probado en raspberry pi 3b+ con raspbian stretch

Installar opencv3

https://www.pyimagesearch.com/2017/09/04/raspbian-stretch-install-opencv-3-python-on-your-raspberry-pi/


Configurar ble 

sudo nano /etc/systemd/system/dbus-org.bluez.service

ExecStart=/usr/lib/bluetooth/bluetoothd -E

sudo reboot now

ahora corre python3 main.py