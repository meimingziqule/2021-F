THRESHOLD = (21, 0, -77, 5, -110, 127)
import sensor, image, time, pyb,math,lcd
from pyb import UART, LED,Pin, Timer
import math
#rho_pid = PID(p=0.37, i=0)
#theta_pid = PID(p=0.001, i=0)
LED(1).on()
LED(2).on()
LED(3).on()
uart = UART(3,19200)
sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
lcd.init(freq=15000000)		
while(True):
	clock.tick()
	img = sensor.snapshot().binary([THRESHOLD])
	line = img.get_regression([(100,100)], robust = True)
#	计算出的rho_err接近0，表示直线中心与图像中心对齐。
#   计算出的theta_err接近0，表示直线角度与图像水平对齐。
# #rho_err 其实就是判断直线与画面中心的误差，
	if (line):
		rho_err = abs(line.rho())-img.width()/2
		if line.theta()>90:
			theta_err = line.theta()-180
		else:
			theta_err = line.theta()
		img.draw_line(line.line(), color = 127)
		print(rho_err,line.magnitude(),rho_err)
		#用于表示检测到的直线的强度或置信度的值
		if line.magnitude()>8:
			#rho_output = rho_pid.get_pid(rho_err,1)
			#theta_output = theta_pid.get_pid(theta_err,1)
			#output = rho_output + theta_output
			#if(output<0):
			#	output = abs(output) + 100
			#OUTPUT = str(round(output))
			#uart.write(OUTPUT)
			#uart.write('\r\n')
			#print(OUTPUT)
			OUTPUT = str(round(theta_err))
			uart.write(OUTPUT)
			uart.write('\r\n')
			print(OUTPUT)
			lcd.display(img)