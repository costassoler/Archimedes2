import socket
import os


while True:
    try:
        L=1
        R=2
        V=3
        text_file = open("tex.txt","w")
        Eth0Status = os.popen("sudo ifconfig eth0").read()
        text_file.write("weehaw")
        text_file.close
        
        print(Eth0Status)
        if ('RUNNING' in Eth0Status):
            L=R=V=0
        print(L,R,V)
        
    except:
        continue
