
from machine import UART
import time
import sys
import select

uart0 = UART(id=0,baudrate=9600,rx=Pin(17),tx=Pin(16),bits=8,parity=None,stop=1)

poll_uart0 = select.poll()
poll_uart0.register(uart0,select.POLLIN)

opMode = 'standby'
transmitDelayMs = 3000;
msgCnt = 0

while True
