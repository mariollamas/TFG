#!/usr/bin/env python

import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 


class Twist2Wheel:

    #############################################################
    def __init__(self):
    #############################################################
        rospy.init_node('speed2wheel', anonymous=False)  # Inicio el nodo 
        rospy.loginfo("Start node speed2wheel")          # Muestro por pantalla
    
        self.L = rospy.get_param("~base_width", 0.468)   # Establezco el ancho del robot como un parametro (por defecto= 0.468 m el summit XL)
    
        self.pub_lwheel = rospy.Publisher('left_wheel', Float32, queue_size=10)     # Declara el publicador de vel rueda izq
        self.pub_rwheel = rospy.Publisher('right_wheel', Float32, queue_size=10)    # Declara el publicador de vel rueda dcha
        rospy.Subscriber('cmd_vel', Twist, self.Callback)                           # Se subscribe a cmd_vel para obtener la vel linear y ang
    
        self.vel_left = 0       # Vel izq
        self.vel_right = 0      # Vel dcha
          
    #############################################################
    def spinOnce(self):
    #############################################################  
        self.vel_right = 1.0 * self.v + self.w * self.L / 2     # Calcula las velocidades
        self.vel_left = 1.0 * self.v - self.w * self.L / 2
        rospy.loginfo("Wheel Vel: (%f, %f)", self.vel_left, self.vel_right)
        rospy.loginfo("v = %f; w= %f", self.v, self.w)
                
        self.pub_lwheel.publish(self.vel_left)          # Publicamos la velocidad
        self.pub_rwheel.publish(self.vel_right)

    #############################################################
    def Callback(self,msg):
    #############################################################
        #rospy.loginfo("-D- Callback: %s" % str(msg))
        self.v = msg.linear.x           # velocidad linear
        self.w = msg.angular.z          # velocidad angular
        self.spinOnce()                 # Llamo a la funcion que calcula vel
    
if __name__ == '__main__':      # Funcion Main, funcion principal
    """ main """
    try:
        WheelSpeed = Twist2Wheel()  # Creamos una variable de la clase creada (se ejecuta init)
        rospy.spin()                # Se queda recibiendo mensajes hasta que se mate el nodo
    except rospy.ROSInterruptException:     # Para asegurarnos que no continua ejecutandose despues de matar el nodo
        pass
