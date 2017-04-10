#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16

from Tkinter import *
import subprocess

import math

voltage_reading = 0
calculated_angle = 0

main_circle = 'bigCircle'
moving_circle = 'smallCircle'

img_width = 400
img_height = 400


def callback(data):
    global voltage_reading
    global calculated_angle

    voltage_reading = data.data / 1000.0

    if 5.532 <= voltage_reading <= 9.66:
        calculated_angle = (33.111 * voltage_reading) - 178.372
    elif 0.353 <= voltage_reading <= 5.99:
        calculated_angle = (38.841 * voltage_reading) + 144.235
    else:
        voltage_reading = 'within shorting range'
        calculated_angle = 'within shorting range'


def draw_circle(draw_canvas, x, y, radius, colour, obj_name, thickness=1, fill_colour=''):

    p1x = x - radius
    p1y = y - radius

    p2x = x + radius
    p2y = y + radius

    draw_canvas.create_oval(p1x, p1y, p2x, p2y, outline=colour, tags=obj_name, width=thickness, fill=fill_colour)


def task(canvas, win, tp, angle):
    global voltage_reading
    global calculated_angle

    global img_width
    global img_height

    global moving_circle
    global main_circle

    distance = img_width / 2 - 20

    canvas.delete(moving_circle)

    if isinstance(voltage_reading, basestring):
        tp.configure(text=str(voltage_reading), fg='red')
        angle.configure(text=str(calculated_angle), fg='red')
    else:
        tp.configure(text=str(voltage_reading), fg='black')
        angle.configure(text=str(calculated_angle), fg='black')

    if not isinstance(calculated_angle, basestring):
        xc = (img_width / 2) + distance * math.sin(math.radians(calculated_angle))
        yc = (img_height / 2) - distance * math.cos(math.radians(calculated_angle))

        draw_circle(canvas, xc, yc, 10, 'red', moving_circle, fill_colour='red')

    win.after(1, task, canvas, win, tp, angle)


def main():
    global main_circle
    global moving_circle

    rospy.init_node('gui', anonymous=True)
    rospy.Subscriber("/voltage/gui", Int16, callback)

    output = subprocess.check_output('sleep 2 ; date', shell=True)

    win = Tk()
    win.title("slipCoder")
    f1 = Frame(win)
    tp = Label(f1, text='Date: ' + str(output[:-1]))
    v_label = Label(f1, text='Voltage:')
    angle_label = Label(f1, text='Angle Calculated')
    angle = Label(f1)

    canvas = Canvas(win, width=img_width, height=img_height)

    f1.pack()
    v_label.pack()
    tp.pack()
    angle_label.pack()
    angle.pack()
    canvas.pack()

    draw_circle(canvas, img_height / 2, img_width / 2, img_height / 2 - 5, 'green', main_circle, 10)
    draw_circle(canvas, 20, 20, 30, 'red', moving_circle, fill_colour='red')

    win.after(1, task, canvas, win, tp, angle)
    win.mainloop()


if __name__ == '__main__':
    main()
