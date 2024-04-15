''' Raspberry Pi Pico - Voltmeter
 Sends via uart
 Works with ADC pins 0 to 4 
 pins 0 to 2 are gp26 to gp28
 pin 4 is internal temp sensor
 See: www.penguintutor.com/projects/pico '''

'''
n(byte) <pin_number>(byte) //setup pwm on pin
w(byte) <pin_number>(byte) <value>(byte string until \\n char)

'''

from machine import UART, Pin, PWM
import sys

print(sys.stdin.read(10))

uart = UART(1,115200)

pwm_pins = {}

while True:
    print(1)
    instruction = uart.read(1)
    print(2)
    pin = uart.read(1)
    print(pin)
    pin = int.from_bytes(pin,'little')
    print(4)
    if instruction == 'n':
        pwm = PWM(Pin(pin))
        pwm.freq(2000)
        pwm_pins.update((pin, pwm))

    elif instruction == 'w':
        value = uart.readline()

        pwm = pwm_pins[pin]
        pwm.duty_u16(value)
        

    # convert read value (byte into a number for the pin)
    '''
    if (read_pin >= ord('0') and read_pin <= ord('4')):
        new_pin = read_pin - ord('0')
        if (new_pin != adc_pin):
            # Select new pin
            adc_pin = new_pin
            adc = machine.ADC(adc_pin)
        # Now read adc value
        adc_value = adc.read_u16()
        #print ("Pin {} value {}".format(adc_pin, adc_value * conversion_factor))
        uart.write ("Pin {} value {}\n".format(adc_pin, adc_value * conversion_factor))
    '''
    
