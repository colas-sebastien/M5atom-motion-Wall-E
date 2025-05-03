![M5Atom WALL-E](img/wall-e-02.jpg)

# M5Atom WALL-E

Create your Wall-E thanks to M5atom

## Hardware
- [1x Zumo Chassis Kit](https://www.pololu.com/product/1418) 
- [2x Micro Metal Gearmotor HP 6V](https://www.pololu.com/product/1101) 
- [Grove - I2C Motor Driver](https://www.seeedstudio.com/Grove-I2C-Motor-Driver-TB6612FNG-p-3220.html) 
- [M5Atom](https://shop.m5stack.com/products/atomic-motion-base-stm32f030?variant=44393917219073) 
- [Atomic Motion Base](https://shop.m5stack.com/products/atom-tailbat) 
- [Battery 16340 2600mAh](https://www.amazon.fr/Rechargeables-Batterie-2800mAh-Capacit%C3%A9-Frontale/dp/B0CTQ4X244)
- [Battery holder for 16340](https://www.amazon.fr/dp/B07MC7WR8J?ref=ppx_yo2ov_dt_b_fed_asin_title)
- [3x ServoMotors sg90](https://www.amazon.fr/dp/B0DFCGXY2J)
- [Grove Ultrasonic distance sensor](https://www.seeedstudio.com/Grove-Ultrasonic-Distance-Sensor.html)
- [A bluetooth gamepad supported by BluePad32 for Arduino](https://gitlab.com/ricardoquesada/bluepad32-arduino) 

![zumo](img/zumo.jpg)
![motor](img/motor.jpg)
![I2C motor driver](img/i2c_motor.jpg)
![M5Atom lite](img/m5atom.jpg)
![Atomic Motion Base](img/atomic-motion.jpg)
![Battery holder for 16340](img/support_battery.jpg)
![Battery 16340 2600mAh](img/16340.jpg)
![Servo Motor](img/servomotor.jpg)
![Grove Ultrasonic distance sensor](img/grove-ultrasonic-distance-sensor.jpg)
![Steam Controller](img/steam-controller.jpg)

## 3D Model
### FreeCAD
![FreeCAD](img/freecad.png)
You can fonud the FreeCAD model into the *FreeCAD* directory.

### STL
![MeshLab](img/MeshLab.png)

You can fonud the FreeCAD model into the *stl* directory.

## Connect everything together
![Connect everything together](driagram/components.drawio.png)
### Atomic Motion Modification (optional)
![Atom Modification](img/atom-modif-0.jpg)
To power on/off WALL-E and easily replace the battery the following modifications can be on the Atomic module
- Add an external battery holder
- Add an external switch

![Atom Modification](img/atom-modif-1.jpg)

### Inside WALL-E
![Atom Modification](img/connect-everything.jpg)

## Software
- Install thanks to Board Manager: 
    - BluePad for Arduino [BluePad for Arduino](https://bluepad32.readthedocs.io/en/latest/plat_arduino/)
    - M5Stack
- Install thanks to Library Manager:
    - Grove Motor Driver TB6612FNG
    - Grove Ultrasonic Ranger
    - Adafruit NeoPixel
    - M5Atom    
- Select for Board: *ESP32 + Bluepad32 Arduino / M5Stack-ATOM*