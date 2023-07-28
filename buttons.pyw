from gpiozero import Button
from signal import pause
import keyboard

def butt_1():
    keyboard.press_and_release('a')
def butt_2():
    keyboard.press_and_release('b')
def butt_3():
    keyboard.press_and_release('c')
def butt_4():
    keyboard.press_and_release('d')

button_1 = Button(17)
button_2 = Button(22)
button_3 = Button(23)
button_4 = Button(27)

button_1.when_pressed = butt_1
button_2.when_pressed = butt_2
button_3.when_pressed = butt_3
button_4.when_pressed = butt_4

pause()

