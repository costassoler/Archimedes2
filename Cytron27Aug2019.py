import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

DirRV=17
PwmRV=27
GPIO.setup(PwmRV,GPIO.OUT)
GPIO.setup(DirRV,GPIO.OUT)
rv=GPIO.PWM(PwmRV,100)

DirR=22
PwmR=10
GPIO.setup(DirR,GPIO.OUT)
GPIO.setup(PwmR,GPIO.OUT)
r=GPIO.PWM(PwmR,100)

DirL=9
PwmL=11
GPIO.setup(DirL,GPIO.OUT)
GPIO.setup(PwmL,GPIO.OUT)
l=GPIO.PWM(PwmL,100)

DirLV=5
PwmLV=6
GPIO.setup(DirLV,GPIO.OUT)
GPIO.setup(PwmLV,GPIO.OUT)
lv=GPIO.PWM(PwmLV,100)

def RV(w):
    if(w<=0):
        GPIO.output(DirRV,GPIO.HIGH)
        rv.start(abs(w))
    if(w>0):
        GPIO.output(DirRV,GPIO.LOW)
        rv.start(abs(w))

def R(x):
    if(x<=0):
        GPIO.output(DirR,GPIO.HIGH)
        r.start(abs(x))
    if(x>0):
        GPIO.output(DirR,GPIO.LOW)
        r.start(abs(x))

def LV(y):
    if(y>=0):
        GPIO.output(DirLV,GPIO.HIGH)
        lv.start(abs(y))
    if(y<0):
        GPIO.output(DirLV,GPIO.LOW)
        lv.start(abs(y))
def L(z):
    if(z<=0):
        GPIO.output(DirL,GPIO.HIGH)
        l.start(abs(z))
    if(z>0):
        GPIO.output(DirL,GPIO.LOW)
        l.start(abs(z))
