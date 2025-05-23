# Este script é uma versão para ROS2 dos scripts originais
# em ROS1 feitos pelo Arnaldo Júnior e Lícia Sales.
# Feito por: Rogério Cuenca.

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from std_msgs.msg import String, Float64
import RPi.GPIO as GPIO
import time as time

class Servo_Arm(Node):

    def __init__(self):
        super().__init__('servo_arm')

        self.subscriber = self.create_subscription(
            Float64,
            'joint2_position_controller/command',
            self.CommandCallback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.
        self.subscriber = self.create_subscription(
            Float64,
            'joint1_position_controller/command',
            self.CommandCallback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.
        
        # Seta o modo do  GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Seta o GPIO do servo motor
        self.pinMotorBraco = 12
        self.pinMotorGarra = 13

        # Seta a frequencia
        self.Frequency = 50

        # Seta o DutyCycle da posicao da Braco 
        # Braco Sobe
        self.BracoSobe = 10 
        # Braco no centro
        self.BracoCentro = 5
        # Braco recolhida
        self.BracoRecolhe = 2
        # garra fechada
        self.GarraFechada = 12
        # garra aberta
        self.GarraAberta = 6

        # Para o servo motor
        self.Stop = 0

        # Seta o pino do servo motor como saida
        GPIO.setup(self.pinMotorBraco, GPIO.OUT)
        GPIO.setup(self.pinMotorGarra, GPIO.OUT)

        # Ajusta a frequencia do PWM
        self.pwmMotorBraco = GPIO.PWM(self.pinMotorBraco, self.Frequency)
        self.pwmMotorGarra = GPIO.PWM(self.pinMotorGarra, self.Frequency)

        # Inicializa o PWM (servo parado)
        self.pwmMotorBraco.start(self.Stop)
        self.pwmMotorGarra.start(self.Stop)

    # Funcao stop servo motor
    def StopMotor(self):
        self.pwmMotorBraco.ChangeDutyCycle(self.Stop)
        self.pwmMotorGarra.ChangeDutyCycle(self.Stop)

    # Funcao posicao Braco
    def PosBraco(self, command):
        valor = float(command)
        commandBraco =5+ 2.67*valor + (0.444*valor**2)
        self.pwmMotorBraco.ChangeDutyCycle(commandBraco)

    # Funcao posicao Garra
    def PosGarra(self, command):
        # converte o valor recebido em radianos para entrada do PWM
        commandGarra= (6*command) + 12 
        self.pwmMotorGarra.ChangeDutyCycle(commandGarra)

    def CommandCallback(self, commandMessage):
        command = commandMessage.data

        self.get_logger().info('Posicao: '+str(command))
        self.PosGarra(command)
        time.sleep(0.5)
        self.StopMotor()
        time.sleep(0.5)
    
    def CommandCallback1(self, commandMessage):
        command = commandMessage.data

        self.get_logger().info('Posicao: '+str(command))
        self.PosBraco(command)
        time.sleep(0.5)
        self.StopMotor()
        time.sleep(0.5)



def main(args=None):
    rclpy.init(args=args)
    servo_arm= Servo_Arm()
    rclpy.spin(servo_arm)
    servo_arm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()