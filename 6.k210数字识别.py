import sensor, image, lcd, time, math
import KPU as kpu
import gc, sys
from machine import UART, Timer
from fpioa_manager import fm

red_thresholds = (29, 97, 14, 127, -128, 127)
input_size = (224, 224)

labels = ["9", "1", "4", "2", "3", "8", "5", "6", "7"]
anchors = [0.4192, 0.3702, 0.5744, 1.6689, 0.6932, 0.6054, 1.0054, 0.9615, 2.1672, 1.6683]

num = 0
condi_flag = ""
begin_flag = 1
red_flag = 0

fm.register(18, fm.fpioa.UART1_RX, force=True)
fm.register(19, fm.fpioa.UART1_TX, force=True)
uart = UART(UART.UART1, 115200, 8, 1, 0, timeout=1000, read_buf_len=4096)
frame_head = '#'
frame_tail = ';'


condi_flag = ""#位置标志位
#出现最多的两个数字
num1=0
num2=0

roi_1 = (0,0,300,200)

task_flag = 'a'
#拼接'#0l4;'
send_num_flag1 =None
send_num_flag2 =None
#拼接'#0l4;'
send_num_flag = None

#发送目标显示
task1_num_show = None
task2_num_show =None

num1_center_x = None
num1_center_x = None
num2_center_x = None
num2_center_x = None

task2_3_num_flag = None

task1_num_list  = []
task2_num_list  = []
lcd.init(freq=15000000)
#找两个数字
def find_most_two_num(list):  
    key_list = []
    value_list = []
    count_dict = {}
    for item in list:#遍历列表
        if item in count_dict:#如果元素已经存在于字典中，则增加计数
            count_dict[item] += 1#增加计数
        else:#如果元素不存在于字典中，则初始化计数为1
            count_dict[item] = 1#初始化计数为1’
    new_count_dict = {k: v for k, v in sorted(count_dict.items(), key=lambda item: item[1], reverse=True)}
    for k,v in new_count_dict.items():
        key_list.append(k)
       # value_list.append(v)
    count_dict.clear()
    return key_list[0],key_list[1]
'''
from collections import Counter

def find_most_two_num(input_list):
    """
    Find the two most common elements in a list.
    
    Parameters:
    input_list (list): The list of elements to be analyzed.
    
    Returns:
    tuple: A tuple containing the two most common elements, or (None, None) if less than two elements exist.
    """
    # 检查输入是否为列表类型
    if not isinstance(input_list, list):
        raise ValueError("Input must be a list.")
    
    # 使用Counter来计算元素的频率，并直接获取最频繁的两个元素
    counter = Counter(input_list)
    most_common = counter.most_common(2)
    
    # 如果列表为空或只有一个元素，most_common将少于两个元素，此时返回(None, None)
    if len(most_common) < 2:
        return None, None
    else:
        # 返回出现次数最多的两个元素
        return most_common[0][0], most_common[1][0]
'''



######找出列表中出现次数最多的元素
def find_most_common(list):  
    count_dict = {}
    for item in list:######遍历列表
        if item in count_dict:######如果元素已经存在于字典中，则增加计数
            count_dict[item] += 1######增加计数
        else:######如果元素不存在于字典中，则初始化计数为1
            count_dict[item] = 1######初始化计数为1
    max_count = max(count_dict.values())######找到最大计数
    most_frequent_values = [k for k, v in count_dict.items() if v == max_count]######找到所有最大计数的元素
    if len(most_frequent_values) == 1:
        return most_frequent_values[0]
    else:
        list.clear()  ###### 出现最大元素个数相同，则清空传入的列表
        return None
    
#获取数字识别结果           (这里为什么不用)
def find_most_commom_value(img,objects,num):
    if objects:
        num_list = []
        for obj in objects:
            pos = obj.rect()
            img.draw_rectangle(pos)
            img.draw_string(pos[0], pos[1], "%s : %.2f" %(labels[obj.classid()], obj.value()), scale=2, color=(255, 0, 0))
            print("%s : %.2f" %(labels[obj.classid()], obj.value()))
            num_list.append(labels[obj.classid()])#添加预测结果到列表
            num += 1
            print(num)
            if num == 10:
                print("num ==10")
                num = 0
                most_commom_value = find_most_common(list)


#位置检测
def find_num_condi(center_x,center_y,red_x):
    if center_x > red_x:
        return "#0r"
    if center_x < red_x:
        return "#0l" 



def find_red_max_blob(img,red_threshold):  #寻找红色色块  返回其外接矩形框坐标
    blobs = img.find_blobs([red_threshold])
    if blobs:
        max_blob = max(blobs, key=lambda b: b.pixels()) # 找到最大的色块
        img.draw_rectangle(max_blob.rect(), color=(255, 0, 0)) # 在图像上画出最大色块的矩形
        print("最大色块位置：", max_blob.cx(), max_blob.cy()) # 输出最大色块中心坐标到串口
        return max_blob.cx()
        # 如果需要通过串口发送数据，请取消下一行注释
        # uart.write('X: {} Y: {}\n'.format(max_blob.cx(), max_blob.cy()).encode())

#task_flag == a
def main(anchors = None,labels = None, model_addr="/sd/demo.kmodel", sensor_window=input_size, lcd_rotation=0, sensor_hmirror=False, sensor_vflip=False):
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA) #320×240
    #sensor.set_windowing(sensor_window)
    sensor.set_hmirror(sensor_hmirror)
    sensor.set_vflip(sensor_vflip)
    sensor.set_hmirror(True)
    sensor.set_vflip(True)
    sensor.run(1)
    sensor.skip_frames(n=10)
    global num,begin_flag,task_flag,red_thresholds,red_flag,condi_flag,num1,num2,send_num_flag1,send_num_flag2,send_num_flag,task2_num_list,task1_num_list,task2_3_num_flag,
    
    
    task = None
    task = kpu.load(model_addr)
    
    while(True):
        img = sensor.snapshot()
        img.copy(roi = (48,16,224,224),copy_to_fb = img)
        img.pix_to_ai()    
        data = uart.read()
        if data:
            data_decoded = data.decode('utf-8')
            if data_decoded[0] == frame_head and data_decoded[2] == frame_tail:
                task_flag = data_decoded[1]
                if task_flag == 'a':    
                    num = 0
                print(task_flag)
        #task1
        if task_flag == 'a':      
            kpu.init_yolo2(task, 0.5, 0.3, 5, anchors) # threshold:[0,1], nms_value: [0, 1]
            img.draw_string(0, 0, "task1", color=(255, 0, 0), scale=2)
            #print("正在执行task1")
            objects = kpu.run_yolo2(task, img)
            if objects:
                for obj in objects:
                    pos = obj.rect()
                    img.draw_rectangle(pos)
                    img.draw_string(pos[0], pos[1], "%s : %.2f" %(labels[obj.classid()], obj.value()), scale=2, color=(255, 255, 255))
                    print("%s : %.2f" %(labels[obj.classid()], obj.value()))
                    task1_num_list.append(labels[obj.classid()])#添加预测结果到列表
                    num +=1
                    print(num)
                    img.draw_string(0, 100, "num:%d"%(num) , scale=2, color=(255, 255, 255))
                    if num == 10:
                        print("num == 10")
                        print("list:",task1_num_list)
                        img.draw_string(0, 100, "num:%d"%(num) , scale=2, color=(255, 255, 255))
                        num = 0
                        most_commom_value = find_most_common(task1_num_list)
                        task1_num_list.clear()
                        if most_commom_value != None:#判断是否返回最大值
                            if begin_flag == 1:
                                begin_flag = 0#区分第一次
                                uart.write("#0b"+str(most_commom_value)+';')
                                task1_num_show = most_commom_value
                                if task1_num_show is not None:
                                    img.draw_string(0, 200, task1_num_show , scale=2, color=(255, 255, 255))#打印识别结果
                                print("#0b")
                                print("begin")                                                                                                                       
                            else:    
                                uart.write("#0c"+str(most_commom_value)+';')
                                img.draw_string(0, 200, "finish" , scale=2, color=(255, 255, 255))
                                print("most_commom_value:",most_commom_value)
                                print("#0c"+str(most_commom_value))
                        else:
                            print("no most_commom_value")
                            img.draw_string(0, 150, "none_finish" , scale=2, color=(255, 255, 255))
                    '''#K210找十字路口(已调)
                    red_blobs_crossroad = img.find_blobs([(0, 100, 12, 127, -124, 117)],roi=(0,80,500,30),x_stride=20, y_stride=20, pixels_threshold=3000)
                    if not red_blobs_crossroad:
                        print("none_crossroad;")
                    else:
                        for blob in red_blobs_crossroad:
                            img.draw_rectangle(blob.rect())
                            img.draw_cross(blob.cx(), blob.cy())
                            uart.write('#b;')#k210识别到十字路口
                            img.draw_string(0, 0, "crossroad", color=(255, 0, 0), scale=2)
                            #print("#0c;")               ''' 
            
        #task2    
        elif task_flag == 'b': 
            kpu.init_yolo2(task, 0.4, 0.4, 5, anchors) # threshold:[0,1], nms_value: [0, 1]
            img.draw_string(0, 0, "task2", color=(255, 0, 0), scale=2)
            #print("正在执行task2")
            objects = kpu.run_yolo2(task, img)
            if objects:
                for obj in objects:
                    red_x = find_red_max_blob(img,red_thresholds) #识别最大红色色块  (是否需要roi)
                    pos = obj.rect()
                    img.draw_rectangle(pos)
                    img.draw_string(pos[0], pos[1], "%s : %.2f" %(labels[obj.classid()], obj.value()), scale=2, color=(255, 255, 255))
                    center_x = pos[0] + pos[2] / 2
                    center_y = pos[1] + pos[3] / 2
                    #保护逻辑
                    if labels[obj.classid()] == 1  or labels[obj.classid()] == 2:
                        continue
                    
                    print("%s : %.2f" %(labels[obj.classid()], obj.value()))
                    task2_num_list.append(labels[obj.classid()])#添加预测结果到列表
                    num +=1
                    print(num)
                    img.draw_string(0, 100, "num:%d"%(num) , scale=2, color=(255, 255, 255))
                    #识别
                    if num >= 20:
                        print("num == 20")
                        print(task2_num_list)
                        img.draw_string(0, 100, "num:%d"%(num) , scale=2, color=(255, 255, 255))
                        num = 0
                        num1,num2 = find_most_two_num(task2_num_list)#返回出现频率最高的两个数
                        task2_num_list.clear()
                        print("num1:%d,num2:%d")

                    if labels[obj.classid()] == num1:
                        num1_center_x = center_x
                        num1_center_y = center_y
                    if labels[obj.classid()] == num2:
                        num2_center_x = center_x
                        num2_center_y = center_y
                    if num1_center_x is not None and num1_center_y is not None and num2_center_x is not None and num2_center_y is not None:
                        if num1_center_x < num2_center_x:
                            task2_3_num_flag = f"#0l{num1}r{num2};"
                        else:
                            task2_3_num_flag = f"#0r{num1}l{num2};"
                        uart.write(task2_3_num_flag)
                        print("task2_3_num_flag:%s" % task2_3_num_flag)
                        '''if num1_center_x < num2_center_x:
                            task2_3_num_flag1 = str("#0l") + str(num1)#  #0l3
                            task2_3_num_flag2 = str("#0r") + str(num2)#  #0r4
                        else:
                            task2_3_num_flag1 = str("#0r") + str(num1)#  #0r3
                            task2_3_num_flag2 = str("#0l") + str(num2)#  #0l4
                        task2_3_num_flag = task2_3_num_flag1+ task2_3_num_flag2
                        task2_3_num_flag  = task2_3_num_flag[:4]+task2_3_num_flag[6:]+';'  # #0l3r4;
                        uart.write(task2_3_num_flag)
                        print("task2_3_num_flag:%s"%task2_3_num_flag)           '''
                        
        lcd.display(img)


if __name__ == "__main__":
    
        # main(anchors = anchors, labels=labels, model_addr=0x300000, lcd_rotation=0)
        main(anchors = anchors, labels=labels, model_addr="/sd/demo.kmodel")