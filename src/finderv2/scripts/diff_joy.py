#!/usr/bin/env python
# -*- coding: utf8 -*-

import rospy
from finder.msg import TwoWheelInt16
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Imports de funciones matemáticas
from math import exp

class Finder_diff_joy:
    
    def __init__(self, node_name_override = 'finder_diff_joy'):
		
        # El nombre del node deberìa poder especificarse como override desde alguna
        # instancia superior, por si acaso, se establece un default razonable.
        # De todos modos, el nombre del nodo que se guarda "internamente" no puede
        # asumirse igual al proporcionado, por si se invocasen al mismo tiempo varios
        # nodos del mismo "tipo", pues ROS les anexa un número al final como ID
        rospy.init_node(node_name_override)
        self.nodename = rospy.get_name()
        # loginfo sirve como medio de DEBUGGING, y para guardar un registro de la
        # ejecución del programa. Por ejemplo, algón overnodo podría hacer uso del log
        # para arreglar errores
        rospy.loginfo("finder_diff_joy starting with name %s", self.nodename) 
        # QUizá algún parámetro superior especifique una frecuencia de salida de datos,
        # por default, si no se encuentra, se usa una de 20 Hz 
        self.rate = rospy.get_param("finder_diff_joy_rate", 20)
        # Conexión con Joystick, dos diccionarios especifican los botones de interés.
        # Estos datos TAMBIÉN podrían ser pedidos como parámetros. Finalmente, nos
        # suscribimos al nodo joy de TIPO Joy (son fijos, a menos que se invocaran
        # varios joysticks, pero la versión estándar de Joy de todos modos no
        # lo permite). El nodo Joy al publicar llama a la función joyCb
        self.axes_names = {'linear':5, 'angular':0}
        self.buttons_names = {'reverse':1}
        self.joySub = rospy.Subscriber("joy", Joy, self.joyCb)
        
        # Setting up the output data type
        self.wheelData = TwoWheelInt16()
        self.wheelData.wheel1 = 0
        self.wheelData.wheel2 = 0
        self.wheelPub  = rospy.Publisher("vdes", TwoWheelInt16) 
        
        # Otro topic en Arduino especifíca el "modo" de control. Si true, entonces
        # los datos publicados en vdes se pasan DIRECTAMENTE a las salidas PWM
        self.McPub = rospy.Publisher("mc", Bool)
        
        # Variables internas
        self.reverse = False
        self.mult_reverse = 0
        self.angular_rate = 0
        self.linear_rate = 0
        # lista de elementos de filtrado
        self.exp_term_angular_rate = 0
        self.exp_term_lin_prod = 0   
        self.last_lin_prod = 0
        self.last_angular_rate = 0               
        
    def joyCb(self, data):
		
        # data catch. Los métodos de callback deberían ser cortos y 
        # eficientes, además de imposibles de "bloquear"
        self.reverse = data.buttons[self.buttons_names['reverse']]
        self.angular_rate = data.axes[self.axes_names['angular']] / 2.
        self.linear_rate = (1 - data.axes[self.axes_names['linear']]) / 2.
        
    def update(self):
		
        # una variable registra el signo de la reversa
        if (self.reverse):
            self.mult_reverse = -1.
        else:
            self.mult_reverse = 1.
        
        self.lin_prod = self.mult_reverse * self.linear_rate
        
        #rospy.loginfo("%s lin_prod" % self.lin_prod) 
        #rospy.loginfo("%s ang_rate" % self.angular_rate)
        
        # Etapa de filtrado con decaimiento exponencial
        self.now_dif_lin_prod = self.last_lin_prod - self.lin_prod
        self.exp_term_lin_prod = (self.exp_term_lin_prod + self.now_dif_lin_prod) * exp(-3. * (1. / self.rate))
        self.out_lin_prod = self.lin_prod + self.exp_term_lin_prod
        self.last_lin_prod = self.lin_prod
        
        self.now_dif_angular_rate = self.last_angular_rate - self.angular_rate
        self.exp_term_angular_rate = (self.exp_term_angular_rate + self.now_dif_angular_rate) * exp(-3. * (1. / self.rate))
        self.out_angular_rate = self.angular_rate + self.exp_term_angular_rate
        self.last_angular_rate = self.angular_rate
        
        # Se determinan los valores finales de cada rueda
        self.m_right = (self.out_lin_prod + self.out_angular_rate / 2. + 1.5) * 85
        self.m_left = (self.out_lin_prod - self.out_angular_rate / 2. + 1.5) * 85
        
        rospy.loginfo("Finder m_right: %s m_left: %s", self.m_right, self.m_left) 
        
        self.wheelData.wheel1 = self.m_right
        self.wheelData.wheel2 = self.m_left
        
        self.wheelPub.publish(self.wheelData)
        
    def initNodes(self):
		
        # Nos aseguramos que el modo de control del uC (si los datos en vdes se entienden como
        # valores de PWM o de velocidad deseada) sea el correcto. Actualmente se manejan PWMs
        self.McPub.publish(False)
        
    def spin(self):
		
        # rospy.Rate y sleep permiten que el nodo pare su ejecución (cuidado, los subscriptores
        # NO están restringidos por el resto del nodo más allá de la función de callback) e 
        # intente ejecutarse cada tantos ms, dados por el rate declarado en el constructor de la
        # clase. Ahorra recursos
        self.initNodes()
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

# Invocación del constructor. Básicamente, si este archivo fuera a ejecutarse como "top level", el
# intérprete le asignaría el "nombre" (protegido) __main__ y ejecutaría las siguientes lineas como
# LO PRIMERO en el archivo (salvo imports y declaraciones)    
if __name__ == '__main__':
    """ main """
    finder_diff_joy = Finder_diff_joy()
    finder_diff_joy.spin()
    

