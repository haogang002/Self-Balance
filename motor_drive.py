import time
from machine import Pin,PWM,Timer
from array import array
# import mpu6050
from encoder_rp2 import Encoder
import rp2

# import importlib.util
# import sys
# spec = importlib.util.spec_from_file_location("module.name", "D:\Robot1\mpu6050.py")
# foo = importlib.util.module_from_spec(spec)
# sys.modules["module.name"] = foo
# spec.loader.exec_module(foo)
# foo.MPU6050()
# import sys
# sys.path.append('/path/to/directory')
# import mpu6050
# sys.path.append( '.' )
# sys.path.append( '../Robot1' )
# import encoder_rp2

# import sys, os
# print(sys.path)
# sys.path.append('os.path.join(sys.path[0])')
# sys.path.append(os.path.join(sys.path[0],'bar'))
# print(sys.path)
# import mpu6050
#config LED
led = Pin(25, Pin.OUT)
# Lefe side control
ina1 = Pin(18, Pin.OUT)
ina2 = Pin(17, Pin.OUT)
pwma = PWM(Pin(16))
# Right side control
inb1 = Pin(21, Pin.OUT)
inb2 = Pin(20, Pin.OUT)
pwmb = PWM(Pin(19))

pwma.freq(1000)
pwmb.freq(1000)
led.toggle()

def RotateCW(duty):
    ina1.value(1)
    ina2.value(0)
    inb1.value(1)
    inb2.value(0)
    duty_16 = int((duty*65535)/100)
    pwma.duty_u16(duty_16)
    pwmb.duty_u16(duty_16)
    
def RotateCCW(duty):
    ina1.value(0)
    ina2.value(1)
    inb1.value(0)
    inb2.value(1)
    duty_16 = int((duty*65535)/100)
    pwma.duty_u16(duty_16)
    pwmb.duty_u16(duty_16)

def StopMotor():
    ina1.value(0)
    ina2.value(0)
    inb1.value(0)
    inb2.value(0)
    pwma.duty_u16(0)
    pwmb.duty_u16(0)
    
def make_isr(pos):
    old_x = array('i', (0,))
    @micropython.viper
    def isr(sm):
        i = ptr32(pos)
        p = ptr32(old_x)
        while sm.rx_fifo():
            v : int = int(sm.get()) & 3
            x : int = v & 1
            y : int = v >> 1
            s : int = 1 if (x ^ y) else -1
            i[0] = i[0] + (s if (x ^ p[0]) else (0 - s))
            p[0] = x
    return isr

# Args:
# StateMachine no. (0-7): each instance must have a different sm_no.
# An initialised input Pin: this and the next pin are the encoder interface.
class Encoder:
    def __init__(self, sm_no, base_pin, scale=1):
        self.scale = scale
        self._pos = array("i", (0,))  # [pos]
        self.sm = rp2.StateMachine(sm_no, self.pio_quadrature, in_base=base_pin)
        self.sm.irq(make_isr(self._pos))  # Instantiate the closure
        self.sm.exec("set(y, 99)")  # Initialise y: guarantee different to the input
        self.sm.active(1)

    @rp2.asm_pio()
    def pio_quadrature(in_init=rp2.PIO.IN_LOW):
        wrap_target()
        label("again")
        in_(pins, 2)
        mov(x, isr)
        jmp(x_not_y, "push_data")
        mov(isr, null)
        jmp("again")
        label("push_data")
        push()
        irq(block, rel(0))
        mov(y, x)
        wrap()

    def position(self, value=None):
        if value is not None:
            self._pos[0] = round(value / self.scale)
        return self._pos[0] * self.scale

    def value(self, value=None):
        if value is not None:
            self._pos[0] = value
        return self._pos[0]
#To power the encoder
encode_vcc1 = Pin(2, Pin.OUT)
encode_vcc1.value(1)
encode_vcc2 = Pin(13, Pin.OUT)
encode_vcc2.value(1)

# def tick(Timer):
#     e = Encoder(0, Pin(0))
#     time.sleep(0.5)
#     eR = Encoder(0, Pin(0))
#     eL = Encoder(0, Pin(14))
#     print(eR.value())
#     print("eL.value()")
#     e = Encoder(0, Pin(0))
#     print(e.value())
#     eL = Encoder(0, Pin(14))
#     print(eL.value())
#     time.sleep(0.5)
#     print(e.value())
# 
#     e = Encoder(0, Pin(14))
#     print(e.value())
#     eL = Encoder(0, Pin(14))
#     print(eL.value())
#     time.sleep(0.5)
#     print(e.value())
#    
# Timer().init(freq=1, mode=Timer.PERIODIC, callback=tick)

# while True:
    
#     duty_cycle=float(input("Enter PWM duty cycle"))
#     e = Encoder(0, Pin(0))
#     print(e.value())
#     eL = Encoder(0, Pin(14))
#     print(eL.value())
#     time.sleep(1)
#     print(e.value())

#     e = Encoder(0, Pin(14))
#     print(e.value())
#     eL = Encoder(0, Pin(14))
#     print(eL.value())
#     time.sleep(1)
#     print(e.value())
#     duty_cycle=50
#     print(duty_cycle)
# 
#     RotateCCW(duty_cycle)
#     time.sleep(5)
#     print("change direction")
#     RotateCCW(0)
#     time.sleep(1)
#     RotateCW(duty_cycle)
#     time.sleep(5)    
#     StopMotor()
#     print("stop")
    


# while True:
#     led.high()
#     time.sleep(0.5)
#     led.low()
#     time.sleep(0.5)