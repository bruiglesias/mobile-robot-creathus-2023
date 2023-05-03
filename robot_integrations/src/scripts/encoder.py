import Jetson.GPIO as GPIO
import 
import math
import threading
import time

class RotaryEncoder:


    def __init__(self, gpio: GPIO, a_pin, b_pin, callback=None):
        self.gpio = gpio
        self.a_pin = a_pin
        self.b_pin = b_pin

        self.gpio.setup(self.a_pin, self.gpio.IN, self.gpio.PUD_UP)
        self.gpio.setup(self.b_pin, self.gpio.IN, self.gpio.PUD_UP)

        self.steps = 0
        self.last_delta = 0
        self.r_seq = self.rotation_sequence()

        # Callback function gets called when a rotation is detected
        # Function format should be:
        # FuncName(x) where x is 1 or -1 depending on the detected rotation
        # direction
        self.callback = callback

        # steps_per_cycle and self.remainder are only used in get_cycles which
        # returns a coarse-granularity step count.  By default
        # steps_per_cycle is 4 as there are 4 steps per
        # detent on my encoder, and get_cycles() will return a signed
        # count of full detent steps.
        self.steps_per_cycle = 4
        self.remainder = 0

    # Gets the 2-bit rotation state of the current position
    # This is deprecated - we now use rotation_sequence instead.
    def rotation_state(self):
        a_state = self.gpio.input(self.a_pin)
        b_state = self.gpio.input(self.b_pin)
        r_state = a_state | b_state << 1
        return r_state

    # Returns the quadrature encoder state converted into
    # a numerical sequence 0,1,2,3,0,1,2,3...
    #
    # Turning the encoder clockwise generates these
    # values for switches B and A:
    #  B A
    #  0 0
    #  0 1
    #  1 1
    #  1 0
    # We convert these to an ordinal sequence number by returning
    #   seq = (A ^ B) | B << 2
    #
    def rotation_sequence(self):
        a_state = self.gpio.input(self.a_pin)
        b_state = self.gpio.input(self.b_pin)
        r_seq = (a_state ^ b_state) | b_state << 1
        return r_seq

    def update(self):
        delta = 0
        r_seq = self.rotation_sequence()
        if r_seq != self.r_seq:
            delta = (r_seq - self.r_seq) % 4
            if delta == 3:
                delta = -1
            elif delta == 2:
                delta = int(math.copysign(delta, self.last_delta))  # same direction as previous, 2 steps

            self.last_delta = delta
            self.r_seq = r_seq
        self.steps += delta
        if(self.callback is not None):
            cycles = self.get_cycles()
            if(cycles != 0):
                self.callback(cycles)
            

    def get_steps(self):
        steps = self.steps
        self.steps = 0
        return steps

    # get_cycles returns a scaled down step count to match (for example)
    # the detents on an encoder switch.  If you have 4 delta steps between
    # each detent, and you want to count only full detent steps, use
    # get_cycles() instead of get_delta().  It returns -1, 0 or 1.  If
    # you have 2 steps per detent, set encoder.steps_per_cycle to 2
    # before you call this method.
    def get_cycles(self):
        # python negative integers do not behave like they do in C.
        #   -1 // 2 = -1 (not 0)
        #   -1 % 2 =  1 (not -1)
        # // is integer division operator.  Note the behaviour of the / operator
        # when used on integers changed between python 2 and 3.
        # See http://www.python.org/dev/peps/pep-0238/
        self.remainder += self.get_steps()
        cycles = self.remainder // self.steps_per_cycle
        self.remainder %= self.steps_per_cycle # remainder always remains positive
        return cycles

    def start(self):
        def isr():
            self.update()
        self.gpio.add_event_detect(self.a_pin, self.gpio.BOTH, isr)
        self.gpio.add_event_detect(self.b_pin, self.gpio.BOTH, isr)

    class Worker(threading.Thread):
        def __init__(self, gpio, a_pin, b_pin):
            threading.Thread.__init__(self)
            self.lock = threading.Lock()
            self.stopping = False
            self.encoder = RotaryEncoder(gpio, a_pin, b_pin)
            self.daemon = True
            self.delta = 0
            self.delay = 0.001

        def run(self):
            while not self.stopping:
                self.encoder.update()
                time.sleep(self.delay)

        def stop(self):
            self.stopping = True

        def get_steps(self):
            return self.encoder.get_steps()


if __name__ == "__main__":

    A_PIN = 7
    B_PIN = 9
    gpio = GPIO
    encoder = RotaryEncoder(gpio, A_PIN, B_PIN)
    
    encoder.start()
   
    while True:
      delta = encoder.get_cycles()
      if delta!=0:
        print("rotate %d" % delta)
      else:
        time.sleep(0.1)