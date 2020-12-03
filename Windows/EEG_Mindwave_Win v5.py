# -*- coding: cp1252 -*-
############################################################################
#Este software fue desarrollado en el laboratorio de percepcion visual y robotica
#PVR del Instituto Tecnológico de Chihuahua
#Ing. Carlos Eduardo Cañedo Figueroa  "eduarcan_1711@hotmail.com"
#27/Febrero/2017
############################################################################
#Para que este software funcione es necesario tener instaladas las librerias
#numpy, scipy, pandas, pybluez, matplotlib y espeak
#de manera externa se debe instalar la libreria peakutils
#que se encuentra dentro de los archivos de este software
import time
import bluetooth
import re
import struct
import collections
import sys
import os
import matplotlib.pyplot as plt
from scipy import signal
from scipy import stats # importando scipy.stats
import pandas as pd # importando pandas
import scipy
import csv
import sys
from math import exp
from scipy.stats import logistic
import numpy as np 
import unittest
import peakutils
import os
from numpy.testing import assert_array_almost_equal

##################################################
#De aqui en adelante todo el codigo esta enfocado a la conexión
#del dispositivo BCI MindWavemobil y la lectura de datos.
class MindwaveMobileRawReader:
    START_OF_PACKET_BYTE = 0xaa;
    def __init__(self):
        self._buffer = [];
        self._bufferPosition = 0;
        
    def connectToMindWaveMobile(self, mac):
        # Conexión bluetooth RFCOMM
        self.mindwaveMobileSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        mindwaveMobileAddress = mac;
        
        while(True):
            try:
                self.mindwaveMobileSocket.connect((mindwaveMobileAddress, 1))
                return;
            except bluetooth.btcommon.BluetoothError as error:
                #Esto puede ocurrir si el dispositivo fue detectado pero no se pudo realizar la conexión
                #La falla principal es que el dispositivo BCI este muy retirado,
                #que la bateria se este agotando o bien que el bluetooth no este
                #funcionando de manera correcta.
                print "Error en la conexión ", error, "; Reinicia dispositivos..."
                print "Reiniciando sistema"
                #os.system("espeak -ves -s300 'Reiniciar sistema completo y revisar baterias'") #Solo para sistemas con espeak instalado de lo contrario comentar
                time.sleep(15)


        
    def _readMoreBytesIntoBuffer(self, amountOfBytes):
        newBytes = self._readBytesFromMindwaveMobile(amountOfBytes)
        self._buffer += newBytes
    
    def _readBytesFromMindwaveMobile(self, amountOfBytes):
        missingBytes = amountOfBytes
        receivedBytes = ""
        # Algunas veces el puerto de salida no recibe los paquetes correctos
        #es por eso que esta parte del codigo revisa si el paquete es correcto
        while(missingBytes > 0):
            receivedBytes += self.mindwaveMobileSocket.recv(missingBytes)
            missingBytes = amountOfBytes - len(receivedBytes)
        return receivedBytes;

    def peekByte(self):
        self._ensureMoreBytesCanBeRead();
        return ord(self._buffer[self._bufferPosition])

    def getByte(self):
        self._ensureMoreBytesCanBeRead(100);
        return self._getNextByte();
    
    def  _ensureMoreBytesCanBeRead(self, amountOfBytes):
        if (self._bufferSize() <= self._bufferPosition + amountOfBytes):
            self._readMoreBytesIntoBuffer(amountOfBytes)
    
    def _getNextByte(self):
        nextByte = ord(self._buffer[self._bufferPosition]);
        self._bufferPosition += 1;
        return nextByte;

    def getBytes(self, amountOfBytes):
        self._ensureMoreBytesCanBeRead(amountOfBytes);
        return self._getNextBytes(amountOfBytes);
    
    def _getNextBytes(self, amountOfBytes):
        nextBytes = map(ord, self._buffer[self._bufferPosition: self._bufferPosition + amountOfBytes])
        self._bufferPosition += amountOfBytes
        return nextBytes
    
    def clearAlreadyReadBuffer(self):
        self._buffer = self._buffer[self._bufferPosition : ]
        self._bufferPosition = 0;
    
    def _bufferSize(self):
        return len(self._buffer);
    
#------------------------------------------------------------------------------ 
#Creacion de Datapoints, usados para el resto del procesamiento
class DataPoint:
    def __init__(self, dataValueBytes):
        self._dataValueBytes = dataValueBytes

class RawDataPoint(DataPoint):
    def __init__(self, dataValueBytes):
        DataPoint.__init__(self, dataValueBytes)
        self.rawValue = self._readRawValue()

    def _readRawValue(self):
        firstByte = self._dataValueBytes[0]
        secondByte = self._dataValueBytes[1]
        # Se evalua que todo este correcto..
        # se puede revisar en http://stackoverflow.com/questions/5994307/bitwise-operations-in-python
        rawValue = firstByte * 256 + secondByte;
        if rawValue >= 32768:
            rawValue -= 65536
        return rawValue # Dato decodificado del dispositivo BCI

    def __str__(self):
        return str(self.rawValue)

#------------------------------------------------------------------------------ 
EXTENDED_CODE_BYTE = 0x55
class MindwavePacketPayloadParser:
    
    def __init__(self, payloadBytes):
        self._payloadBytes = payloadBytes
        self._payloadIndex = 0
        
    def parseDataPoints(self):
        dataPoints = []
        while (not self._atEndOfPayloadBytes()):
            dataPoint = self._parseOneDataPoint()
            dataPoints.append(dataPoint)
        return dataPoints
        
    def _atEndOfPayloadBytes(self):
        return self._payloadIndex == len(self._payloadBytes)
    
    def _parseOneDataPoint(self):
        dataRowCode = self._extractDataRowCode();
        dataRowValueBytes = self._extractDataRowValueBytes(dataRowCode)
        return self._createDataPoint(dataRowCode, dataRowValueBytes)
    
    def _extractDataRowCode(self):
        return self._ignoreExtendedCodeBytesAndGetRowCode()
        
    def _ignoreExtendedCodeBytesAndGetRowCode(self):
        # EXTENDED_CODE_BYTES Utilizado de acuerdo a 
        # http://wearcam.org/ece516/mindset_communications_protocol.pdf
        # (Agosto 2012)
        byte = self._getNextByte()
        while (byte == EXTENDED_CODE_BYTE):
            byte = self._getNextByte()
        dataRowCode = byte
        return dataRowCode
       
    def _getNextByte(self):
        nextByte = self._payloadBytes[self._payloadIndex]
        self._payloadIndex += 1
        return nextByte
    
    def _getNextBytes(self, amountOfBytes):
        nextBytes = self._payloadBytes[self._payloadIndex : self._payloadIndex + amountOfBytes]
        self._payloadIndex += amountOfBytes
        return nextBytes
    
    def _extractDataRowValueBytes(self, dataRowCode):
        lengthOfValueBytes = self._extractLengthOfValueBytes(dataRowCode)
        dataRowValueBytes = self._getNextBytes(lengthOfValueBytes)
        return dataRowValueBytes
       
    def _extractLengthOfValueBytes(self, dataRowCode):
        dataRowHasLengthByte = dataRowCode > 0x7f
        if (dataRowHasLengthByte):
            return self._getNextByte()
        else:
            return 1
        
    def _createDataPoint(self, dataRowCode, dataRowValueBytes):
        if (dataRowCode == 0x80):
            return RawDataPoint(dataRowValueBytes)

            assert False 
#------------------------------------------------------------------------------       
Err=0;
class MindwaveDataPointReader:
    def __init__(self):
        self._mindwaveMobileRawReader = MindwaveMobileRawReader()
        self._dataPointQueue = collections.deque()

    def start(self, mac):
        self._mindwaveMobileRawReader.connectToMindWaveMobile(mac)
        
    def readNextDataPoint(self):
        if (not self._moreDataPointsInQueue()):
            self._putNextDataPointsInQueue()
        return self._getDataPointFromQueue()

    def _moreDataPointsInQueue(self):
        return len(self._dataPointQueue) > 0
    
    def _getDataPointFromQueue(self):
        return self._dataPointQueue.pop();
    
    def _putNextDataPointsInQueue(self):
        dataPoints = self._readDataPointsFromOnePacket()
        self._dataPointQueue.extend(dataPoints)
    
    def _readDataPointsFromOnePacket(self):
        self._goToStartOfNextPacket()
        payloadBytes, checkSum = self._readOnePacket()
        if (not self._checkSumIsOk(payloadBytes, checkSum)):
            print "Paquete recibido incorrecto o incompleto..."
            Err=Err+1;
            if Err>10:
                print "Realiza un cambio de baterias"
                
            return self._readDataPointsFromOnePacket();
        else:
            dataPoints = self._readDataPointsFromPayload(payloadBytes)
        self._mindwaveMobileRawReader.clearAlreadyReadBuffer()
        return dataPoints;
        
    def _goToStartOfNextPacket(self):
        while(True):
            byte = self._mindwaveMobileRawReader.getByte()
            if (byte == MindwaveMobileRawReader.START_OF_PACKET_BYTE):  # Se necesitan dos de estos bytes para comenzar.
                byte = self._mindwaveMobileRawReader.getByte()
                if (byte == MindwaveMobileRawReader.START_OF_PACKET_BYTE):
                    # Ahora se puede comenzar..
                    return;

    def _readOnePacket(self):
            payloadLength = self._readPayloadLength();
            payloadBytes, checkSum = self._readPacket(payloadLength);
            return payloadBytes, checkSum
    
    def _readPayloadLength(self):
        payloadLength = self._mindwaveMobileRawReader.getByte()
        return payloadLength

    def _readPacket(self, payloadLength):
        payloadBytes = self._mindwaveMobileRawReader.getBytes(payloadLength)
        checkSum = self._mindwaveMobileRawReader.getByte()
        return payloadBytes, checkSum

    def _checkSumIsOk(self, payloadBytes, checkSum):
        sumOfPayload = sum(payloadBytes)
        lastEightBits = sumOfPayload % 256
        invertedLastEightBits = self._computeOnesComplement(lastEightBits)
        return invertedLastEightBits == checkSum;
    
    def _computeOnesComplement(self, lastEightBits):
        return ~lastEightBits + 256
        
    def _readDataPointsFromPayload(self, payloadBytes):
        payloadParser = MindwavePacketPayloadParser(payloadBytes)
        return payloadParser.parseDataPoints();
#------------------------------------------------------------------------------ 
##################################################
# De aqui en adelante inicia el menu principal de la aplicación
#se mandan llamar las clases de conexión del dispositivo BCI
#se declaran las variables necesarias
#se desarrolla sistema de analisis de señales.
###################################################
#Declaración de variables necesarias para el sistema
evento=list()
detectado=[]
b,a=scipy.signal.butter(1,0.016,btype='low')
ev=0
c=0
d=0  #Elementos en el buffer
dr=0
con=0 #Contador de iteraciones
paso=1022 #Tamaño de una ventana (2 segundos por ventana)
vector=list(range(1023))
zero=list(range(511))
####################################################
#Estas funciones son opcionales
#se pueden utulizar para guardar sesiones y adquirir muestras
#de algun estudio realizado
#Solo se debe descomentar todo este bloque y buscar dentro del resto del codigo de
#procesamiento las funciones de escritura de datos en archivos CSV indicadas en el codigo
#las cuales tambien se deben descomentar.

#archivo=open('datos_señal_Jose_prueba_1.csv','w')
#caracteristicas=open('caracteristicas_Jose_prueba_1.csv','w')

#arc_write=csv.writer(archivo, delimiter=',',lineterminator='\n')
#char_write=csv.writer(caracteristicas, delimiter=',',lineterminator='\n')

###################################################
#Correr en Windows dado que el dispositivo se encuentra
#emparejado no es necesario estar realizando una busqueda continua

##mac = '20:68:9D:79:DC:D1'
mac='98:07:2D:7F:F8:76'
mindwaveDataPointReader = MindwaveDataPointReader()
mindwaveDataPointReader.start(mac)

############################################################
#Correr forzosamente en linux, tambien corre en windows.
#ya que el dispositivo no se empareja una vez que
#el sistema es reiniciado, se debe realizar una busqueda de
#dispositivos disponibles cada vez que el sistema de inicia.
##ejecutar=1
##name=''
##while(ejecutar==1):
##            #os.system("espeak -ves -s300 'Buscando dispositivo E E G'") #Solo para sistemas con espeak instalado de lo contrario comentar
##            nearby_devices = bluetooth.discover_devices(lookup_names=True)
##            print("found %d devices" % len(nearby_devices))
##            for addr, name in nearby_devices:
##                print("  %s - %s" % (addr, name))
##                if name=='MindWave Mobile':
##                        mac = addr
##                        print (mac)
##                        mindwaveDataPointReader = MindwaveDataPointReader()
##                        mindwaveDataPointReader.start(mac)
##                        ejecutar=2
############################################################################################################
# De aqui en adelante se indican las variables y los parametros de las redes neuronales
#que serviran para la clasificación de eventos detectados
#La estructura de las redes neuronales es la siguiente.

#                /------------------0-----------TanSig----------\
#                /------------------0-----------TanSig----------\
#   3 entradas 0 {4 neuronas capa intermedia                    } 0-> salida lineal
#                \------------------0-----------TanSig----------/
#                \------------------0-----------TanSig----------/
#
#############################################################################################################
# Parametros para red neuronal expresión Si
iw1_S=[-1.28525550398776,-1.67242177113707,-7.30799418111909]
iw2_S=[49.4231807126897,0.370863506611682,-0.720120973934198]
iw3_S=[-172.062050125205,-6.46288483659798,80.4899481420968]
iw4_S=[-8.97756267837256,-0.419566775499572,0.375861061766228]

bias1_S=[7.35162384777232,8.70946069871787,-8.93152764907367,-1.50294485378840]

lw_S=[-1.70128604562246,-1.00070065456328,-1.00004796997483,-0.000657626688677838]

bias2_S=0.701284769605565

###################################################################################################################

###################################################################################################################
#Paramentros para red neuronal expresion No
iw1_N=[5.49343667811109,-0.227085695802986,-14.9324949614070]
iw2_N=[5.30104255542695,-0.227582282685663,-12.6714079983854]
iw3_N=[-7.24135149638501,-3.15981599827580,1.10041345696888]
iw4_N=[-0.479277755494529,1.54334608611205,0.813275088332283]

bias1_N=[-11.8955006698992,-10.2164533356199,-5.74837090910445,-2.55548577703115]

lw_N=[5.84131051833710,-5.93041310635018,-0.167660482205745,-0.00440717577768775]

bias2_N=-1.24859883027606

###################################################################################################################
###################################################################################################################
#Paramentros para red neuronal expresion No sé
iw1_NS=[-1.53939334775265,-1.26945990719291,0.955739193641885]
iw2_NS=[3.27964933065322,-7.18538595984150,1.44460276483839]
iw3_NS=[-5.41455192743479,1.28059652933842,16.2290713701355]
iw4_NS=[-0.236747471607390,-4.89355338288698,-8.10782508750142]

bias1_NS=[2.91924008465347,-5.71414078272102,6.64543513923034,-0.672806584619684]

lw_NS=[-0.325688004044815,-2.11464356664577,1.05005233622680,1.09285291740218]

bias2_NS=-2.83659339212122

###################################################################################################################
###################################################################################################################
#Paramentros para red neuronal expresion Más o menos
iw1_MS=[-0.397379077439344,1.04823232540308,-0.800529935765565]
iw2_MS=[-15.2026139866494,0.216882838418264,-3.10578842278070]
iw3_MS=[-11.1070774923539,0.109607301994530,-1.73485164689520]
iw4_MS=[-2.02241945377948,-1.32332273671208,2.99457135195215]

bias1_MS=[3.62848352775187,-8.21874102548877,-6.14739146774925,0.864290308944766]

lw_MS=[0.0240012100697603,3.57619404955975,-3.59103972744147,0.0122772200886582]

bias2_MS=-1.03200223979192

###################################################################################################################
###################################################################################################################
#Paramentros para red neuronal Ruido
iw1_R=[14.9802519593616,0.179254562633311,-5.74850224384566]
iw2_R=[-16.2126034715443,0.627875164235056,0.310789335976796]
iw3_R=[4.27848071114461,5.20957898933157,2.47663853150605]
iw4_R=[0.0148852271930111,1.87975192555761,0.308094306524793]

bias1_R=[5.27997693590817,-2.33562890894019,1.67861574182089,-3.06042191527905]

lw_R=[-0.990068225850420,-0.961361515472697,0.0631925667468894,0.644646901455215]

bias2_R=1.67706868034964

###################################################################################################################


###################################################################################################
def red_neuronal_si(picos,energia,distancia):
###
### Normalización de datos de entrada
    ymax_S=1.0;
    ymin_S=-1.0;
    xminima_S=[1.0,71.312255629999996,0.0]
    xmaxima_S=[12.0,989.20772260000001,2028.0]

    
    y_1_S=picos-xminima_S[0];
    y_2_S=energia-xminima_S[1];
    y_3_S=distancia-xminima_S[2];

    k_1_S=((ymax_S-ymin_S)/(xmaxima_S[0]-xminima_S[0])*y_1_S);
    k_2_S=((ymax_S-ymin_S)/(xmaxima_S[1]-xminima_S[1])*y_2_S);
    k_3_S=((ymax_S-ymin_S)/(xmaxima_S[2]-xminima_S[2])*y_3_S);

    x_1_S=k_1_S+ymin_S;
    x_2_S=k_2_S+ymin_S;
    x_3_S=k_3_S+ymin_S;
###
### Red neuronal Si
    net1_S=[x_1_S*iw1_S[0],x_2_S*iw1_S[1],x_3_S*iw1_S[2]];
    net2_S=[x_1_S*iw2_S[0],x_2_S*iw2_S[1],x_3_S*iw2_S[2]];
    net3_S=[x_1_S*iw3_S[0],x_2_S*iw3_S[1],x_3_S*iw3_S[2]];
    net4_S=[x_1_S*iw4_S[0],x_2_S*iw4_S[1],x_3_S*iw4_S[2]];

    n1_S=(sum(net1_S))+bias1_S[0];
    n2_S=(sum(net2_S))+bias1_S[1];
    n3_S=(sum(net3_S))+bias1_S[2];
    n4_S=(sum(net4_S))+bias1_S[3];


#Funcion tangencial sigmoidal
    f1_S=2/(1+np.exp(-2*n1_S))-1;
    f2_S=2/(1+np.exp(-2*n2_S))-1;
    f3_S=2/(1+np.exp(-2*n3_S))-1;
    f4_S=2/(1+np.exp(-2*n4_S))-1;
    
#Sumatoria de todas las funciones de la capa intermedia por los pesos de salida.
    salida_S=((f1_S*lw_S[0])+(f2_S*lw_S[1])+(f3_S*lw_S[2])+(f4_S*lw_S[3]));
    sumatoria_S=salida_S+bias2_S;
    
###
### MAPMAXMIN inverso para la salida
    xmi_S=0.0;
    xma_S=+1.0;
    ymi_S=-1.0;
    yma_S=+1.0;

    y_i_S=sumatoria_S+(-ymi_S);

    k_i_S=((xma_S-xmi_S)/(yma_S-ymi_S))*(y_i_S);
    #print(k_i_S)
    return k_i_S;


#######################################################################################################
def red_neuronal_no(picos,energia,distancia):
###
### Normalización de datos de entrada
    ymax_N=1.0;
    ymin_N=-1.0;
    xminima_N=[1.0,71.312255629999996,0.0]
    xmaxima_N=[12.0,989.20772260000001,2028.0]

    
    y_1_N=picos-xminima_N[0];
    y_2_N=energia-xminima_N[1];
    y_3_N=distancia-xminima_N[2];

    k_1_N=((ymax_N-ymin_N)/(xmaxima_N[0]-xminima_N[0])*y_1_N);
    k_2_N=((ymax_N-ymin_N)/(xmaxima_N[1]-xminima_N[1])*y_2_N);
    k_3_N=((ymax_N-ymin_N)/(xmaxima_N[2]-xminima_N[2])*y_3_N);

    x_1_N=k_1_N+ymin_N;
    x_2_N=k_2_N+ymin_N;
    x_3_N=k_3_N+ymin_N;
###
### Red neuronal No
    net1_N=[x_1_N*iw1_N[0],x_2_N*iw1_N[1],x_3_N*iw1_N[2]];
    net2_N=[x_1_N*iw2_N[0],x_2_N*iw2_N[1],x_3_N*iw2_N[2]];
    net3_N=[x_1_N*iw3_N[0],x_2_N*iw3_N[1],x_3_N*iw3_N[2]];
    net4_N=[x_1_N*iw4_N[0],x_2_N*iw4_N[1],x_3_N*iw4_N[2]];

    n1_N=(sum(net1_N))+bias1_N[0];
    n2_N=(sum(net2_N))+bias1_N[1];
    n3_N=(sum(net3_N))+bias1_N[2];
    n4_N=(sum(net4_N))+bias1_N[3];


#Funcion tangencial sigmoidal
    f1_N=2/(1+np.exp(-2*n1_N))-1;
    f2_N=2/(1+np.exp(-2*n2_N))-1;
    f3_N=2/(1+np.exp(-2*n3_N))-1;
    f4_N=2/(1+np.exp(-2*n4_N))-1;
    
#Sumatoria de todas las funciones de la capa intermedia por los pesos de salida.
    salida_N=((f1_N*lw_N[0])+(f2_N*lw_N[1])+(f3_N*lw_N[2])+(f4_N*lw_N[3]));
    sumatoria_N=salida_N+bias2_N;
    
###
### MAPMAXMIN inverso para la salida
    xmi_N=0;
    xma_N=+1.0;
    ymi_N=-1.0;
    yma_N=+1.0;

    y_i_N=sumatoria_N+(-ymi_N);

    k_i_N=((xma_N-xmi_N)/(yma_N-ymi_N))*(y_i_N);
    #print(k_i_N)
    return k_i_N;

#######################################################################################################
def red_neuronal_nose(picos,energia,distancia):
###
### Normalización de datos de entrada
    ymax_NS=1.0;
    ymin_NS=-1.0;
    xminima_NS=[1.0,71.312255629999996,0.0]
    xmaxima_NS=[12.0,989.20772260000001,2028.0]

    
    y_1_NS=picos-xminima_NS[0];
    y_2_NS=energia-xminima_NS[1];
    y_3_NS=distancia-xminima_NS[2];

    k_1_NS=((ymax_NS-ymin_NS)/(xmaxima_NS[0]-xminima_NS[0])*y_1_NS);
    k_2_NS=((ymax_NS-ymin_NS)/(xmaxima_NS[1]-xminima_NS[1])*y_2_NS);
    k_3_NS=((ymax_NS-ymin_NS)/(xmaxima_NS[2]-xminima_NS[2])*y_3_NS);

    x_1_NS=k_1_NS+ymin_NS;
    x_2_NS=k_2_NS+ymin_NS;
    x_3_NS=k_3_NS+ymin_NS;
###
### Red neuronal No se
    net1_NS=[x_1_NS*iw1_NS[0],x_2_NS*iw1_NS[1],x_3_NS*iw1_NS[2]];
    net2_NS=[x_1_NS*iw2_NS[0],x_2_NS*iw2_NS[1],x_3_NS*iw2_NS[2]];
    net3_NS=[x_1_NS*iw3_NS[0],x_2_NS*iw3_NS[1],x_3_NS*iw3_NS[2]];
    net4_NS=[x_1_NS*iw4_NS[0],x_2_NS*iw4_NS[1],x_3_NS*iw4_NS[2]];

    n1_NS=(sum(net1_NS))+bias1_NS[0];
    n2_NS=(sum(net2_NS))+bias1_NS[1];
    n3_NS=(sum(net3_NS))+bias1_NS[2];
    n4_NS=(sum(net4_NS))+bias1_NS[3];


#Funcion tangencial sigmoidal
    f1_NS=2/(1+np.exp(-2*n1_NS))-1;
    f2_NS=2/(1+np.exp(-2*n2_NS))-1;
    f3_NS=2/(1+np.exp(-2*n3_NS))-1;
    f4_NS=2/(1+np.exp(-2*n4_NS))-1;
    
#Sumatoria de todas las funciones de la capa intermedia por los pesos de salida.
    salida_NS=((f1_NS*lw_NS[0])+(f2_NS*lw_NS[1])+(f3_NS*lw_NS[2])+(f4_NS*lw_NS[3]));
    sumatoria_NS=salida_NS+bias2_NS;
    
###
### MAPMAXMIN inverso para la salida
    xmi_NS=0;
    xma_NS=+1.0;
    ymi_NS=-1.0;
    yma_NS=+1.0;

    y_i_NS=sumatoria_NS+(-ymi_NS);

    k_i_NS=((xma_NS-xmi_NS)/(yma_NS-ymi_NS))*(y_i_NS);
    #print(k_i_N)
    return k_i_NS;
#######################################################################################################
def red_neuronal_maso(picos,energia,distancia):
#
# Normalización de datos de entrada
    ymax_MS=1.0;
    ymin_MS=-1.0;
    xminima_MS=[1.0,71.312255629999996,0.0]
    xmaxima_MS=[12.0,989.20772260000001,2028.0]

    
    y_1_MS=picos-xminima_MS[0];
    y_2_MS=energia-xminima_MS[1];
    y_3_MS=distancia-xminima_MS[2];

    k_1_MS=((ymax_MS-ymin_MS)/(xmaxima_MS[0]-xminima_MS[0])*y_1_MS);
    k_2_MS=((ymax_MS-ymin_MS)/(xmaxima_MS[1]-xminima_MS[1])*y_2_MS);
    k_3_MS=((ymax_MS-ymin_MS)/(xmaxima_MS[2]-xminima_MS[2])*y_3_MS);

    x_1_MS=k_1_MS+ymin_MS;
    x_2_MS=k_2_MS+ymin_MS;
    x_3_MS=k_3_MS+ymin_MS;
#
# Red neuronal MAS O MENOS
    net1_MS=[x_1_MS*iw1_MS[0],x_2_MS*iw1_MS[1],x_3_MS*iw1_MS[2]];
    net2_MS=[x_1_MS*iw2_MS[0],x_2_MS*iw2_MS[1],x_3_MS*iw2_MS[2]];
    net3_MS=[x_1_MS*iw3_MS[0],x_2_MS*iw3_MS[1],x_3_MS*iw3_MS[2]];
    net4_MS=[x_1_MS*iw4_MS[0],x_2_MS*iw4_MS[1],x_3_MS*iw4_MS[2]];

    n1_MS=(sum(net1_MS))+bias1_MS[0];
    n2_MS=(sum(net2_MS))+bias1_MS[1];
    n3_MS=(sum(net3_MS))+bias1_MS[2];
    n4_MS=(sum(net4_MS))+bias1_MS[3];


#Funcion tangencial sigmoidal
    f1_MS=2/(1+np.exp(-2*n1_MS))-1;
    f2_MS=2/(1+np.exp(-2*n2_MS))-1;
    f3_MS=2/(1+np.exp(-2*n3_MS))-1;
    f4_MS=2/(1+np.exp(-2*n4_MS))-1;
    
#Sumatoria de todas las funciones de la capa intermedia por los pesos de salida.
    salida_MS=((f1_MS*lw_MS[0])+(f2_MS*lw_MS[1])+(f3_MS*lw_MS[2])+(f4_MS*lw_MS[3]));
    sumatoria_MS=salida_MS+bias2_MS;
    
#
# MAPMAXMIN inverso para la salida
    xmi_MS=0;
    xma_MS=+1.0;
    ymi_MS=-1.0;
    yma_MS=+1.0;

    y_i_MS=sumatoria_MS+(-ymi_MS);

    k_i_MS=((xma_MS-xmi_MS)/(yma_MS-ymi_MS))*(y_i_MS);
    #print(k_i_MS)
    return k_i_MS;
#########################################################################################################
def red_neuronal_ruido(picos,energia,distancia):

# Normalización de datos de entrada
    ymax_R=1.0;
    ymin_R=-1.0;
    xminima_R=[1.0,71.312255629999996,0.0]
    xmaxima_R=[12.0,989.20772260000001,2028.0]

    
    y_1_R=picos-xminima_R[0];
    y_2_R=energia-xminima_R[1];
    y_3_R=distancia-xminima_R[2];

    k_1_R=((ymax_R-ymin_R)/(xmaxima_R[0]-xminima_R[0])*y_1_R);
    k_2_R=((ymax_R-ymin_R)/(xmaxima_R[1]-xminima_R[1])*y_2_R);
    k_3_R=((ymax_R-ymin_R)/(xmaxima_R[2]-xminima_R[2])*y_3_R);

    x_1_R=k_1_R+ymin_R;
    x_2_R=k_2_R+ymin_R;
    x_3_R=k_3_R+ymin_R;
#
# Red neuronal MAS O MENOS
    net1_R=[x_1_R*iw1_R[0],x_2_R*iw1_R[1],x_3_R*iw1_R[2]];
    net2_R=[x_1_R*iw2_R[0],x_2_R*iw2_R[1],x_3_R*iw2_R[2]];
    net3_R=[x_1_R*iw3_R[0],x_2_R*iw3_R[1],x_3_R*iw3_R[2]];
    net4_R=[x_1_R*iw4_R[0],x_2_R*iw4_R[1],x_3_R*iw4_R[2]];

    n1_R=(sum(net1_R))+bias1_R[0];
    n2_R=(sum(net2_R))+bias1_R[1];
    n3_R=(sum(net3_R))+bias1_R[2];
    n4_R=(sum(net4_R))+bias1_R[3];


#Funcion tangencial sigmoidal
    f1_R=2/(1+np.exp(-2*n1_R))-1;
    f2_R=2/(1+np.exp(-2*n2_R))-1;
    f3_R=2/(1+np.exp(-2*n3_R))-1;
    f4_R=2/(1+np.exp(-2*n4_R))-1;
    
#Sumatoria de todas las funciones de la capa intermedia por los pesos de salida.
    salida_R=((f1_R*lw_R[0])+(f2_R*lw_R[1])+(f3_R*lw_R[2])+(f4_R*lw_R[3]));
    sumatoria_R=salida_R+bias2_R;
    
#
# MAPMAXMIN inverso para la salida
    xmi_R=0;
    xma_R=+1.0;
    ymi_R=-1.0;
    yma_R=+1.0;

    y_i_R=sumatoria_R+(-ymi_R);

    k_i_R=((xma_R-xmi_R)/(yma_R-ymi_R))*(y_i_R);
    
    return k_i_R;

##################################################################################
##################################################################################
#Algoritmo de analisis y reconocimiento de señales EEG

while True:
    try:
        con=con+1
        dataPoint = mindwaveDataPointReader.readNextDataPoint()
        if con==1:
            print "Enlace establecido"
            #os.system("espeak -ves -s300 'Enlace establecido'") #Solo para sistemas con espeak instalado de lo contrario comentar
        if (dataPoint.__class__ is RawDataPoint): #Lectura de datos a travez de NeuroPi
            a=str(dataPoint)
            Num=int(a)
            #arc_write.writerow([Num])    #Descomentar para guardar todos los datos la mindwave
            
        d=d+1                       #Numero de elementos recibidos
        if d>paso:                  #Si el numero de elementos es mayor a 1022 (Numero de datos por ventana)
            ###########################
            #Descomentar para ver datos por ventana
            plt.figure(1)
            plt.cla()
            plt.clf()
            plt.plot(vector)
            plt.pause(0.01);
            
            plt.ylim([-2000, 2000])
            ###########################
            varianza=np.var(vector)  #Analisis de varianza de los elementos en la ventana
            d=511                    #Indicador de elementos nuevos una vez lleno el buffer

            #Si la varianza pasa el umbral de 3000 se comennzará a crear un vector con elementos correspondiente a un evento
            if varianza>3000:        
                c=c+1;                             #Número de ventanas correspondiente a eventos
                dr=dr+paso+1;
                evento[dr-paso:dr]=vector[d:paso]  #Vector de datos correspondientes a eventos
                
            else:
                if c>1 and len(evento)>paso:      #Se inicia el analisis del vector de evento
                    ev=ev+1
                    
                    ###################################################################
                    b,a=scipy.signal.butter(1,0.016,btype='low')
                    fil=scipy.signal.lfilter(b,a,evento)           #Filtro de la señal detectada como evento con un filtro butter pasa bajas Fc=0.016 Hz
                    arr_fil=np.asarray(fil)
                    ###################################################################
                    #Mapminmax Normalización
                    #norm=arr_fil/max(arr_fil)                      #Normalización de la señal de evento

                    ###################################
                    #Normalización por media y desvaicion std
                    media=(sum(arr_fil))/(len(arr_fil)-1)
                    desviacion=np.std(arr_fil)
                    norm_a=(arr_fil-media)/desviacion

                    #######################################
                    #Normalización por softmax
                    e_x = np.exp((norm_a - media)/desviacion)
                    norm_b=(1+e_x)/(1-e_x)

                    ################################
                    #Normalizacion mapminmax
                    ymax_n=1;
                    ymin_n=-1;
                    xmin_norm=min(arr_fil);
                    xmax_norm=max(arr_fil);
                    ynorm=arr_fil-xmin_norm;
                    k_norm=((ymax_n-ymin_n)/(xmax_norm-xmin_norm)*ynorm);
                    norm=k_norm+ymin_n;

                    #norm=norm_n*-1



                    arr_norm=np.asarray(norm)

                    #########################
                    
                    # En caso de querer observar las señales correspondientes a eventos descomentar
                    #
                    f,(ax1,ax2)=plt.subplots(2,1)
                    ax1.plot(arr_norm)
                    ax2.plot(evento)
                    plt.pause(0.01);
                    #
                    
                    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#
                    #Extracción de las 3 caracteristicas para clasificar las señales de evento

                    energia = sum(arr_norm*arr_norm)               #Calculo de la energia de la señal

                    P_F=peakutils.peak.indexes(arr_norm, thres=0.8/max(arr_norm), min_dist=150)  #detección de picos en la señal de evento

                    #Distancia entre los picos encontrados en el segundo 3 y 4
                    #En caso de no encontrar picos en esas posiciones su valor sera 0                    
                    try:
                        dist=P_F[3]-P_F[2]
                    except:
                        dist=0

                    picos=len(P_F)                          #Numero de picos

                    #char_write.writerow([picos,energia,dist])  #Descomentar para guardar caracteristcas de las señales
                    print(picos,energia,dist)
                    #$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$#

                    #Analisis de las caracteristicas a través de las redes neuronales
                    P_si=red_neuronal_si(picos,energia,dist)
                    #print(P_si)
                    
                    P_no=red_neuronal_no(picos,energia,dist)
                    #print(P_no)

                    P_nose=red_neuronal_nose(picos,energia,dist)
                    #print(P_nose)

                    P_maso=red_neuronal_maso(picos,energia,dist)
                    #print(P_maso)

                    #P_ruido=red_neuronal_ruido(picos,energia,dist)
                    #print(P_ruido)
                    #################################################################
                    #Analisis de los resultados de las redes neuronales
                    #Bajo el siguiente esquema
                    #
                    #        / Si          if P_si    >  {P_no, P_maso, P_nose, P_ruido}  +0.2
                    #        / No          if P_no    >  {P_si, P_maso, P_nose, P_ruido}  +0.2
                    # Clase={- No se       if P_nose  >  {P_no, P_maso, P_si,   P_ruido}  +0.2
                    #        \ Mas o menos if P_maso  >  {P_no, P_si,   P_nose, P_ruido}  +0.2
                    #        \ Ruido       if P_ruido >  {P_no, P_maso, P_nose, P_si}     +0.2
                    #
                    M_g=0.2;
                    V_g=0.6
                    if(P_si>V_g):
                        if(P_si>M_g+P_no and P_si>M_g+P_nose and P_si>M_g+P_maso):
#                        if(P_si>M_g+P_no and P_si>M_g+P_nose and P_si>M_g+P_maso and P_si>M_g+P_ruido):
                            print("expresion Si detectada")
                            #os.system("espeak -ves -s300 'SI!'") #Solo para sistemas con espeak instalado de lo contrario comentar

                    
                    if(P_no>V_g):
                        if(P_no>M_g+P_si and P_no>M_g+P_nose and P_no>M_g+P_maso):
#                        if(P_no>M_g+P_si and P_no>M_g+P_nose and P_no>M_g+P_maso and P_no>M_g+P_ruido):
                            print("expresion No detectada")
                            #os.system("espeak -ves -s300 'NO!'") #Solo para sistemas con espeak instalado de lo contrario comentar

                    if(P_nose>V_g):
                        if(P_nose>M_g+P_si and P_nose>M_g+P_no and P_nose>M_g+P_maso):
#                        if(P_nose>M_g+P_si and P_nose>M_g+P_no and P_nose>M_g+P_maso and P_nose>M_g+P_ruido):
                            print("expresion No Se detectada")
                            #os.system("espeak -ves -s300 'No Se!'") #Solo para sistemas con espeak instalado de lo contrario comentar 

                    if(P_maso>V_g):
                        if(P_maso>M_g+P_si and P_maso>M_g+P_nose and P_maso>M_g+P_si):
#                        if(P_maso>M_g+P_si and P_maso>M_g+P_nose and P_maso>M_g+P_si and P_maso>M_g+P_ruido):
                            print("expresion Mas o menos detectada")
                            #os.system("espeak -ves -s300 'Mas o menos!'") #Solo para sistemas con espeak instalado de lo contrario comentar
                    if(P_no<V_g and P_si<V_g and P_nose<V_g and P_maso<V_g):
                        print("Ruido detectado")
                    #if(P_ruido>V_g):
                    #    if(P_ruido>M_g+P_si and P_ruido>M_g+P_nose and P_ruido>M_g+P_maso and P_ruido>M_g+P_no):
                    #        print("Ruido detectado")

                    ############################################################################
                    # Se vacian los registros para el siguiente analisis
                    fil=list()
                    norm=list()
                    energia=0
                    dr=0 
                    evento=list()
            
            vector[0:d]=vector[d-1:paso-1]
            
        else:
            vector[d]=Num
    except KeyboardInterrupt:
        
        #archivo.close()           #Descomentar si se quiere guardar datos
        #caracteristicas.close()   #descoemntar si se quieren guardar caracteristicas de los eventos detectados
        
        print("Cerrrar todas las ventanas")
        plt.show()
        print("Exiting")
        break


