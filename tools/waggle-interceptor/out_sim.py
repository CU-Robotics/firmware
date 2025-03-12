import time
import random
a = '''
(base) zhixing-li@Zhixings-MacBook-Air firmware % make upload
We've detected you are using a Mac! Consult God if this breaks.
Memory region         Used Size  Region Size  %age Used
            ITCM:      132124 B       512 KB     25.20%
            DTCM:       64992 B       512 KB     12.40%
             RAM:       60768 B       512 KB     11.59%
           FLASH:        161 KB      7936 KB      2.03%
            ERAM:          0 GB        16 MB      0.00%
[Constructing firmware.hex]
[Uploading] - If this fails, press the button on the teensy and re-run make upload
      upload@15923190-Teensy  Uploading to board '15923190-Teensy' (Teensy 4.1)
      upload@15923190-Teensy  Triggering board reboot
      upload@15923190-Teensy  Firmware: firmware.hex
      upload@15923190-Teensy  Flash usage: 161 kiB (2.0%)
      upload@15923190-Teensy  Uploading... 100%
      upload@15923190-Teensy  Sending reset command (with RTC)
Monitoring Teensy on /dev/cu.usbmodem159231901
Attempting to open teensy at /dev/cu.usbmodem159231901
TEENSY SERIAL START
.:^!?!^.
           .:~!?JYYYJ?7?Y5Y7!!.
         :?5YJ?!~:.      ^777YP?.
         5G~                  ~YP?:
         7P5555Y:               ^YP?:....
        ~55J7~^.   ..    .        ^JYYYYYYYYYJJ!.
        YG^     !Y5555J:^PJ    Y5:      ...::^5G^
       :GY    .YG?^..^~ ~GY    5G^ ^!~~^^^!!~7G?
 .!JYYY5G!    7BJ       ~GY    5G^ ~??JJJY555GP!
^55!^:.^~.    ^PP~   .: ^GP:  ^PP:           :7PY.
YG^            :JP5YY55: ~YP55PY^              ~GJ
?G~      .?7~:   .^~~^.    .^:.                :G5
.5P^     7BYJ5YJ7^.                          .~5P^
 .JPJ!~!JP?  .:~?PP^            .:.    .^!JYY5Y!.
   :!???!:       5P.         .!Y5YYYJ?Y5Y?!^:.
                 7G7        7GY!. .:~!^.
                  JG!      :G5
                   7PY!^^~?PY:
                    .!JJJJ?^

FW Ver. 2.1.0

Last Built: Feb 25 2025 at 20:53:53
Random Num: 313ab95b

1
3
Calibrating IMU...
ll_ddot: 0.00
lr_ddot: 0.00
theta_ll: -13.462652
theta_lr: -12.353183
theta_ll_dot: -1.266117
theta_lr_dot: -1.266117
jl: 0.05 0.27
-0.05 0.31
jr: 0.05 0.29
-0.05 0.31
pitch: 0.192226, roll: 0.007845, yaw: 0.499071
torque_fr: 14.00
torque_fl: -14.00
torque_bl: 4.11
torque_br: -5.75
torque_wl: -2.84
torque_wr: 2.82
SAFTYON
speed_wl 0.00
speed_wr 0.00
s: 0.00
wheel_speed_filtered: 0.00
imu_s: 0.00
imu_speed: 0.00
b_accel: 0.00
leglength ll: 0.116824
leglength lr: 0.119738
leglength_dot ll: 0.000000
leglength_dot lr: 0.000000
ll_ddot: 0.00
lr_ddot: 0.00
theta_ll: -13.463951
theta_lr: -12.354483
theta_ll_dot: -1.299443
theta_lr_dot: -1.299443
jl: 0.05 0.27
-0.05 0.31
jr: 0.05 0.29
-0.05 0.31
pitch: 0.192248, roll: 0.007837, yaw: 0.499062
torque_fr: 14.00
torque_fl: -14.00
torque_bl: 4.47
torque_br: -5.38
torque_wl: -2.83
torque_wr: 2.82
SAFTYON
speed_wl 0.00
speed_wr 0.00
s: 0.00
wheel_speed_filtered: 0.00
imu_s: 0.00
imu_speed: 0.00
b_accel: 0.00
leglength ll: 0.116824
leglength lr: 0.119738
leglength_dot ll: 0.000000
leglength_dot lr: 0.000000
ll_ddot: 0.00
lr_ddot: 0.00
theta_ll: -13.464929
theta_lr: -12.355460
theta_ll_dot: -0.972707
waggle graph theta_lr_dot: -0.972707
jl: 0.05 0.27
-0.05 0.31
jr: 0.05 0.29
-0.05 0.31
pitch: 0.192265, roll: 0.007819, yaw: 0.499065
waggle graph torque_fr: 14.00
waggle graph torque_fl: -14.00
torque_bl: 4.64
waggle graph torque_br: -5.39
torque_wl: -2.83
torque_wr: 2.81
SAFTYON
speed_wl 0.00
speed_wr 0.00
s: 0.00
wheel_speed_filtered: 0.00
imu_s: 0.00
imu_speed: 0.00
b_accel: 0.00
leglength ll: 0.116824
leglength lr: 0.119738
leglength_dot ll: 0.000000
leglength_dot lr: 0.000000
ll_ddot: 0.00
lr_ddot: 0.00
theta_ll: -13.465753
theta_lr: -12.356284
theta_ll_dot: -0.825543
theta_lr_dot: -0.825543
jl: 0.05 0.27
-0.05 0.31
jr: 0.05 0.29
-0.05 0.31
pitch: 0.192280, roll: 0.007814, yaw: 0.499068
torque_fr: 14.00
torque_fl: -14.00
torque_bl: 4.64
waggle graph torque_br: -5.47
torque_wl: -2.83
torque_wr: 2.80
SAFTYON
speed_wl 0.00
speed_wr 0.00
s: 0.00
wheel_speed_filtered: 0.
'''

print(a)
n = 1
while True:
    n += random.random()-0.5
    print(f'waggle graph torque_br {n}')
