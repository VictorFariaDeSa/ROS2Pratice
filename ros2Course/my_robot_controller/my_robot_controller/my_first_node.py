#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):

    def __init__(self):
        super().__init__("first_node") #Aqui passamos o nome do Nó
        self.counter = 0
        self.create_timer(1.0,self.timer_callback) #Quando o nó for inciado, em intervalos de tempo igual ao primeiro parametro ele vai rodar a função do segundo parametro


    def timer_callback(self):
        self.get_logger().info(f"Hello {self.counter}")
        self.counter += 1
def main(args=None):
    rclpy.init(args=args) #Aqui estamos incializando a cominicação do ROS
    #O nó vai ser criado aqui dentro--> Utilizando OOP
    node = MyNode()
    rclpy.spin(node) #Isso faz com que o nó fique vivo (ele vai continuar a rodar ate que voce o mate, Com cntl+C por exemplo)
    #Como queremos rodar o no com as funcionalidades do ROS2 precisaremos instalar o nó através do setup.py

    rclpy.shutdown() #Última linha da função. Aqui paramos a comunicação do ROS e destruimos os nós







if __name__=='__main__': #Se eu quiser rodar o script direto do terminal eh preciso dessa linha
    main()