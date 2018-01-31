# Noise Detector project

This implement custom BLE (Bluetooth Low Energe) service. I call this 'Noise Detector service'.
You can see details of this project and related project in this blog, which is written Korean but, I have plan to tranlate English.
To test this service, I made [microphone-amplifier circuit](http://joondong.tistory.com/38?category=685026) and [Android application](https://github.com/JoonDong2/Android)
The noise signal amplified through LM358 input P0.04(AIN5) and converted to 8-bits data, then broadcasted to a peer through 'Detected Noise Value characteristic'
This project is controlled by client(Android application) through 'Detected Noise Value characteristic'.

'ble_app_nds_gcc_nrf51.ld' file need to be modified for porting nRF51822 because this project is based on PCA10028.

Noise Detector service is composed of two characteristics (Detected Noise Value characteristic and Noise Detector Service Control Point characteristic.)

'Base UUID' f673**XXXX**-0994-4967-bdf9-5e7702990a50
'Noise Detector Service UUID' **8d00**
'Detected Value Characteristic' **8d01**
'Noise Detector Controlpoint Characteristic' **8d02**

## Detected Noise Value characteristic

Data collected through ADC is written to this characteristic's value attribute and broadcasted to a peer.

## Noise Detector Service Control Point characteristic.

### Video

You can see vedio of entire projects including android application in [this link](http://joondong.tistory.com/28?category=651762)

# Additional component

This project include battery service and [checking battery levle circuit](http://joondong.tistory.com/41?category=685026).

# Dependancy

This project is based on nRF SDK 12.3, so you have to overwrite it on nRF SDK 12.3 path.