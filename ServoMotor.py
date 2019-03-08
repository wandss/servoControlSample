import RPi.GPIO as GPIO
from time import sleep


class Servo(object):
    """Controls a servo motor connected to the GPIO"""

    def __init__(self, pin_number=3):
        self.pin = pin_number
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        self.servo = GPIO.PWM(self.pin, 50)
        self.servo.start(1)
        self.isAtStart = True

    def sweep(self):
        """Runs the servo motor for start to end and back."""
        position = 0
        countdown = False

        for i in range(14):

            if position == 14:
                countdown = True
            if countdown:
                position -= 2
            else:
                position += 2

            self.servo.ChangeDutyCycle(position)
            GPIO.output(self.pin, False)
            sleep(0.1)

    def toggle(self, position=4):
        """Move servo to a given position case it is not at 0 position"""

        self.isAtStart = not self.isAtStart
        if self.isAtStart:
            position = 1

        self.servo.ChangeDutyCycle(position)
        sleep(0.1)
        self.servo.ChangeDutyCycle(0)


    def clean(self):
        self.toggle(1)
        self.servo.stop()
        GPIO.cleanup()
