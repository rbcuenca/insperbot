#!/usr/bin/python
# -*- coding: latin-1 -*-
#Author: Lícia Sales Date: 06-28-2021
#Adapt: Rogerio B Cuenca Date: 08-16-2023

#imports
import RPi.GPIO as GPIO
import socket
import os
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
from PIL import Image, ImageDraw, ImageFont

#Raspberry Pi pin configuration?
RST = 24
DC = 23
SPI_PORT = 0
SPI_DEVICE = 0
menu = 0
channel = None

# 128x64 display with hardware I2C:
disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST)

# Initialize library
disp. begin()

# Get display width and height.
width = disp.width
height = disp.height

# Clear display
disp.clear()
disp.display()

# Create image buffer.
# Make sure to create image with mode '1' for 1-bit color.
image = Image.new('1', (width, height))

# Create drawing object
draw = ImageDraw.Draw(image)

# Draw a black filled box to clear the image.
draw.rectangle((0,0,width,height), outline=0, fill=0)

# First define some constants to allow easy resizing of shapes.
padding = 2
shape_width = 20
top = padding
emy=top
bottom = height-padding
# Move left to right keeping track of the current x position for drawing shapes.
x = padding
# Load default font.
#font = ImageFont.load_default()
font = ImageFont.truetype('LiberationMono-Regular.ttf', 10)

#Setup I/O
gpio_pin_down = 27
gpio_pin_up = 17
gpio_pin_left = 22
gpio_pin_right = 23

GPIO.setmode(GPIO.BCM)
GPIO.setup(gpio_pin_down, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(gpio_pin_up, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(gpio_pin_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(gpio_pin_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)


# Display image.
disp.image(image)
disp.display()


def stop_robot(channel):
    global menu
    menu = 10
    if menu ==10:
        print("menu= ",menu)
        print("entrei no stop_robot")
        print("menu= ",menu)    

        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, 0),"     STOPING          ",font=font,fill=255)
        draw.text((x,20),"     INSPERBOT         ",font=font,fill=255)
        draw.text((x, 40),"                     ",font=font,fill=255)

        os.system('ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"')  
        time.sleep(3)
        menu=0
    return 
   

def shutdown_robot(channel):
    global menu

    print("menu= ",menu)	
    print("entrei no shutdown_robot")
    menu=menu-30
    print("menu= ",menu)
    if menu==-30:
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, 0),"     SHUTDOWN          ",font=font,fill=255)
        draw.text((x,20),"     INSPERBOT         ",font=font,fill=255)
        draw.text((x, 40),"  Press \/ to confirm ",font=font,fill=255)
    if menu==-60:
        print("menu= ",menu)
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, 0),"                               ",font=font,fill=255)
        draw.text((x,20),"                               ",font=font,fill=255)
        draw.text((x, 40),"                              ",font=font,fill=255)
        time.sleep(1)
        os.system("sudo shutdown now") 
        menu=0               
    return 


def reboot_nodes(channel):
    global menu
    print("menu= ",menu)
    menu=menu+30
    print("entrei no reboot_nodes")
    print("menu= ",menu)
    if menu==30:
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, 0),"       REBOOT          ",font=font,fill=255)
        draw.text((x,10),"      ROS NODES         ",font=font,fill=255)
        draw.text((x, 30),"Press button again... ",font=font,fill=255)
        draw.text((x, 40),".. to confirm ",font=font,fill=255)
        print(menu)
    if menu==60:
        print("Reiniciando os nodes, aguarde a musiquinha") 
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, 0),"    REBOTING                ",font=font,fill=255)
        draw.text((x,20),"    ROS NODES ...          ",font=font,fill=255)
        draw.text((x, 40),"Wait for the music        ",font=font,fill=255)
        os.system("sudo systemctl restart start_turtle.service")         
        menu=0
        time.sleep(5)

    return 

def limpa_tela(channel):
    # Draw the image buffer.
    disp.image(image)
    disp.display()
    time.sleep(0.5)
    return 


GPIO.add_event_detect(gpio_pin_up, GPIO.FALLING, callback = limpa_tela,bouncetime=300)
GPIO.add_event_detect(gpio_pin_down, GPIO.FALLING, callback = shutdown_robot,bouncetime=300)
GPIO.add_event_detect(gpio_pin_left, GPIO.FALLING, callback = stop_robot,bouncetime=300)
GPIO.add_event_detect(gpio_pin_right, GPIO.FALLING, callback = reboot_nodes,bouncetime=300)        

while True:
    limpa_tela(channel)
    if menu == 0:
        hostname = socket.gethostname() 
        draw.rectangle((0,0,width,height), outline=0, fill=0)
        draw.text((x, 0),"    {}            ".format(hostname),font=font,fill=255)
        draw.text((x, 20),"UP Nodes LF STOP            ",font=font,fill=255)
        draw.text((x, 40),"DN ShutDown ",font=font,fill=255)
        menu=0
    

