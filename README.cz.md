# TopTurret
Záměr projektu je vytvořit plně autonomní airsoft sentry-turret *(otočná věž s namontovanou airsoftovou replikou zbraně schopná sama bránit perimetr)*. Turreta by měla být schopná detekce lidí a následného rozhodování, zda se jedná o přátelský/nepřátelský team *(zda střílet)*.

> projekt v rámci programu https://delta-topgun.cz

# technologie
## hardware
- Raspberry pi 5
- raspberry pi pico (*ovladani pwm*)
- rpi camera module v3
- 3 servomotory:
    - otáčení okolo osy Y (**360° MG996R**)
    - sklon (**180° MG996R**)
    - aktivace spouště (**180° MG996R**)
- gyroskop/akcelerometr (**mpu6050**)
- napájení (11.1V li-pol baterie)
    - 11.1V li-pol => step-down na 5V schopný usb pd a proudu 5A => rpi usb-c port / direct power přes 5V a GND piny
- *(zbraň)*

## software
- os: raspberry pi os (raspbian)
- python *(main jazyk)*
- openCV
- ultralytics - YOLOv8 (pytorch)
- picamera2 *<s>libcamera</s>*
- *<s>gpiozero</s>, <s>RPI.GPIO</s>*

# fungování
Program je rozdělen do dvou hlavních částí + subčástí (hlavně v podobě python modulů)
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
Stará se o handeling kamery jak pomocí **picamera2** tak i opencv **videoCapture**, umožňuje tím beze změn spouštět stejný kód jak na raspberry pi tak i na PC pro testování.
### servoControll
Dostává od trackeru "movement vector" - vektor udávající požadovaný úhel zbraně (left-right, up-down). Movement vector může být udáván v úhlu nebo pixelech, které jsou následně na úhel přepočítány, může být absolutní nebo relativní k momentálnímu natočení. S každým snímkem přicházejí nové korekce úhlů a controller se jim přizpůsobuje. Operace záleží na typu serva:
- v případě 180⁰ serva se úhel pouze převede na odpovídající PWM signál.
- v případě 360⁰ serva je využíván pro správné natočení gyroskop, kontrola natočení se děje paralerně se zbytkem kódu. Během přibližování se správnému úhlu se servo bude zpomalovat pro vyšší přesnost.

## Tracker
"Primární část", obsahuje Main loop, okolo kterého je zbytek programu postaven. Při každém průchodu main loopu je zachycen snímek z kamery (cameraControll).
### Person detection
Pomocí YOLOv8 object recognition modelu najde tracker ve snímku všechny lidi (vrátí oblast px kde se člověk nachází).
### Sort
Stará se o indexování lidí a pamatování si indexů ze snímku na snímek. Záznamy o detekovaných lidech, kteří nebyli dlouho znovy detekováni jsou po určité době mazány.
### team detection/evaluation
Tracker rozhoduje teamy na základě barevných pásek na ruce. Pokud nelze team rozlišit, detekovaný člověk je označen jako `unknown`. Detekované teamy se připisují pod index detekovaného člověka a průměrují se. (př.: pokud je člověk s červenou páskou na pár snímcích špatně detekován jako jiný team, je pořád indexovaný jako červený.)

## diagram fungování
![turretDiagramNOBG](https://github.com/X3nom/topTurret/assets/100533068/a26700b2-5d5b-498a-afba-398a7786a85b)
![final_flowchart drawio](https://github.com/X3nom/topTurret/assets/100533068/5d4683f4-5822-410c-9ed6-5722fcc6aa7d)
# milestones
- [x] přepočet px -> rad
- [x] implementace 360⁰ servo ovládání
- [x] napájení
    - [x] provizorní ze zdi
    - [ ] baterie
- [x] prototyp věžě (pohyblivá na x,y + trigger servo)
