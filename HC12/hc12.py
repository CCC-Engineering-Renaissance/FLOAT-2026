
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

while True:
    if(uart0.any()):
        time.sleep(1)
        msg_recieved = uart0.readline().decode('utf-8','ignore').strip()
        if(msg_recieved == 'standby'):
            op_mode = 'standby'
            uart0.write("ACK\n")
        elif(msg_recieved == 'transmit'):
            op_mode = 'transmit'
            uart0.write("ACK\n")
            transmit_delay_start_time_ms = time.ticks_ms()
        else:
            uart0.write("NAK\n")
    delta_time_ms = time.ticks_diff(time.ticks_ms(),transmit_delay_start_time_ms)
    if ((op_mode == 'transmit') and (delta_time_ms >= transmit_delay_ms)):
        msg_cnt+=1
        output_msg_str = "Hello " + str(msg_cnt)
        output_msg_str_encoded = output_msg_str.encode('utf-8','ignore')
        uart0.write(output_msg_str_encoded)
        transmit_delay_start_time_ms = time.ticks_ms()
