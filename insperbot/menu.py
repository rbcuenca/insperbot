# Este script é uma versão para ROS2 dos scripts originais
# em ROS1 feitos pelo Arnaldo Júnior e Lícia Sales.
# Nesta versão (Maio de 2025) estou montando um node ros para facilitar outros comandos.
# Feito por: Rogério Cuenca.

import rclpy
import RPi.GPIO as GPIO
import time as time
import socket
import os
import time
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
import subprocess
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String, Float64
from PIL import Image, ImageDraw, ImageFont


class Menu(Node):
    def __init__(self):
        super().__init__('menu')
        # Configurando e inicando o display
        self.RST = 24
        self.DC = 23
        self.SPI_PORT = 0
        self.SPI_DEVICE = 0
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=self.RST)
        self.disp.begin()
        self.width = self.disp.width
        self.height = self.disp.height
        # Configuração da imagem
        self.image = Image.new('1', (self.width, self.height))
        self.draw = ImageDraw.Draw(self.image)
        self.font = ImageFont.load_default()
        # Configuração dos GPIOs
        self.gpio_pin_down = 27
        self.gpio_pin_up = 17
        self.gpio_pin_left = 22
        self.gpio_pin_right = 23        
        # Estados do sistema
        self.confirmation_pending = False
        self.pending_action = None
        
        self.setup_gpio()
        self.initialize_display()
              
    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.gpio_pin_down, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpio_pin_up, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpio_pin_left, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.gpio_pin_right, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        GPIO.add_event_detect(self.gpio_pin_up, GPIO.FALLING, 
                            callback=self.handle_up, bouncetime=300)
        GPIO.add_event_detect(self.gpio_pin_down, GPIO.FALLING, 
                            callback=self.handle_down, bouncetime=300)
        GPIO.add_event_detect(self.gpio_pin_left, GPIO.FALLING, 
                            callback=self.handle_left, bouncetime=300)
        GPIO.add_event_detect(self.gpio_pin_right, GPIO.FALLING, 
                            callback=self.handle_right, bouncetime=300)

    def initialize_display(self):
        self.disp.clear()
        self.disp.display()
        self.show_main_screen()

    def clear_display(self):
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)

    def update_display(self):
        self.disp.image(self.image)
        self.disp.display()

    def show_main_screen(self):
        self.clear_display()
        hostname = socket.gethostname()
        ip_address = socket.gethostbyname(hostname)
        
        self.draw.text((0, 0), f"    {hostname}", font=self.font, fill=255)
        self.draw.text((0, 15), f"Senha:{hostname}", font=self.font, fill=255)
        self.draw.text((0, 30), "Status: Operacional", font=self.font, fill=255)
        self.draw.text((0, 45), "<- Res       Des ->", font=self.font, fill=255)
        self.update_display()

    def handle_up(self, channel):
        print('cima')

    def handle_down(self, channel):
        print('baixo')
        if not self.confirmation_pending:
            self.stop_robot()
            self.show_main_screen()

    def handle_left(self, channel):
        if not self.confirmation_pending:
            self.confirmation_pending = True
            self.pending_action = "REBOOT"
            self.show_confirmation_screen("Reiniciar nodes?", "Esq: Confirmar")
        elif self.pending_action == "REBOOT":
            self.execute_reboot()
            self.confirmation_pending = False
            self.pending_action = None
            self.show_main_screen()

    def handle_right(self, channel):
        print('direita')
        if not self.confirmation_pending:
            self.confirmation_pending = True
            self.pending_action = "SHUTDOWN"
            self.show_confirmation_screen("Desligar robo?", "Dir: Confirmar")
        elif self.pending_action == "SHUTDOWN":
            self.execute_shutdown()

    def show_confirmation_screen(self, message):
        self.clear_display()
        self.draw.text((0, 0), "Confirmar ação:", font=self.font, fill=255)
        self.draw.text((0, 20), message, font=self.font, fill=255)
        self.draw.text((0, 40), "-> Direita para confirmar", font=self.font, fill=255)
        self.update_display()

    def stop_robot(self):
        self.clear_display()
        self.draw.text((0, 0), "Parando robô...", font=self.font, fill=255)
        self.update_display()
        subprocess.run("ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'", shell=True)
        time.sleep(2)
        self.show_main_screen()

    def execute_reboot(self):
        self.clear_display()
        self.draw.text((0, 0), "Reiniciando nodes...", font=self.font, fill=255)
        self.update_display()
        os.system("sudo systemctl restart start_turtle.service")
        time.sleep(5)
        self.show_main_screen()

    def execute_shutdown(self):
        self.clear_display()
        self.draw.text((0, 0), "Desligando sistema...", font=self.font, fill=255)
        self.update_display()
        os.system("sudo shutdown now")

    def run(self):
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            GPIO.cleanup()
            self.disp.clear()
            self.disp.display()



def main(args=None):
    rclpy.init(args=args)
    menu= Menu()
    rclpy.spin(menu)
    menu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()