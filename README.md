# M253BSP_CAN_Bus_off_Recovery
 M253BSP_CAN_Bus_off_Recovery


update @ 2023/10/13

1. bus off recovery flow ( ex : CAN bus high and low signal short )

- regular polling : send TX SID : 0x333 and XID : 0x444 when 200 ms

![image](https://github.com/released/M253BSP_CAN_Bus_off_Recovery/blob/main/log_regular.jpg)	


2. if entry bus off state (BO interrupt) , attempt to send TX SID : 0x555 , per 50 ms , until 1 sec

![image](https://github.com/released/M253BSP_CAN_Bus_off_Recovery/blob/main/log_slow_resp.jpg)	


3. if bus off state continue after 1 sec , attempt to send TX SID : 0x666 , per 1 sec , until bus off state recover successful

![image](https://github.com/released/M253BSP_CAN_Bus_off_Recovery/blob/main/log_fast_resp_recover.jpg)	


