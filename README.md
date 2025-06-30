# TopTurret
The intention of the project is to create a fully autonomous airsoft sentry-turret *(a rotating turret with a mounted airsoft replica weapon capable of defending a perimeter on its own)*. The turret should be capable of detecting people and then deciding whether it is a friendly/hostile team *(whether to shoot)*.

> project under the program [DELTA-topgun](https://delta-topgun.cz)

> ![original czech readme](./README.cz.md)

# technology
## hardware
- Raspberry pi 5
- raspberry pi pico (*controls pwm*)
- rpi camera module v3
- 3 servomotors:
    - yaw (**360° MG996R**)
    - pitch (**180° MG996R**)
    - trigger (**180° MG996R**)
- gyroscope/accelerometer (**mpu6050**)
- power supply (11.1V li-pol battery)
    - 11.1V li-pol => step-down to 5V capable usb pd and 5A current => rpi usb-c port / direct power via 5V and GND pins
- *(gun)*

## software
- os: raspberry pi os (raspbian)
- python *(main language)*
- openCV
- ultralytics - YOLOv8 (pytorch)
- picamera2 *<s>libcamera</s>*
- *<s>gpiozero</s>, <s>RPI.GPIO</s>*

# functioning
The program is divided into two main parts + subparts (mainly in the form of python modules)
- controller
    - cameraControll
    - servoControll
- tracker
    - main loop
    - person detection
    - "sort"
    - team detection/evaluation

## Controller
### camera Controll
Takes care of camera handling both with **picamera2** and **videoCapture**, allowing you to run the same code on both raspberry pi and PC for testing without changes.

### servoControll
Receives a "movement vector" from the tracker - a vector indicating the desired weapon angle (left-right, up-down). The movement vector can be given in angle or pixels, which are then converted to angle, it can be absolute or relative to the current rotation. With each frame comes new angle corrections and the controller adapts to them. The operation depends on the type of servo:
- in the case of a 180⁰ servo, the angle is only converted to the corresponding PWM signal.
- In the case of a 360⁰ servo, the gyroscope is used for the correct rotation, and the rotation control happens in parallel with the rest of the code. While approaching the correct angle, the servo will slow down for higher accuracy.

## Tracker
"Primary part", contains the Main loop around which the rest of the program is built. On each pass of the main loop, a snapshot is captured from the camera (cameraControll).
### Person detection
Using the YOLOv8 object recognition model, the tracker finds all people in the frame (returns the px area where the person is).
### Sort
Takes care of indexing people and remembering the indexes from frame to frame. Records of detected people that have not been detected again for a long time are deleted after a certain period of time.
### team detection/evaluation
The tracker decides teams based on the colored bands on their hands. If a team cannot be distinguished, the detected person is marked as `unknown`. Detected teams are assigned to the index of the detected person and averaged. (e.g.: if a person with a red band is wrongly detected as another team in a few frames, he is still indexed as red.)
## Diagram
![turretDiagramNOBG](https://github.com/X3nom/topTurret/assets/100533068/a26700b2-5d5b-498a-afba-398a7786a85b)
![final_flowchart drawio](https://github.com/X3nom/topTurret/assets/100533068/5d4683f4-5822-410c-9ed6-5722fcc6aa7d)
