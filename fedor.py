import socket
import time
import cv2
import numpy as np
import lidar


robot = 10099      # Порт управления роботом
lunohod = 11099    # Порт управления луноходом


# Сбросить сцену (робот возвращается в исходную позицию)
def scene_reset():
    connection = connect(10000, socket.SOCK_DGRAM)
    result = send_command(connection, 'SCENE:RESET')
    connection.close()

    return result


# Установить скорость движения лунохода
# Параметры:
#    velocity - скорость вращения моторов
#    seconds - время, в течении которого будет действовать указанная скорость,
#        если 0, то скорость будет действовать до следующего вызова lunohod_move или остановки моторов
def lunohod_move(velocity, seconds=0):
    motors_velset(
        lunohod,
        [
            ['R.WheelF', velocity],
            ['R.WheelB', velocity],
            ['L.WheelF', velocity],
            ['L.WheelB', velocity]
        ]
    )

    if seconds > 0:
        time.sleep(seconds)
        motors_velset(
            lunohod,
            [
                ['R.WheelF', 0],
                ['R.WheelB', 0],
                ['L.WheelF', 0],
                ['L.WheelB', 0]
            ]
        )


# Установить скорость вращения лунохода вокруг вертикальной оси (Z)
# Параметры:
#    velocity - скорость, с которой будут вращаться колёса
#    seconds - время, в течении которого луноход будет поворачиваться
def lunohod_rotate(velocity, seconds=0):
    motors_velset(
        lunohod,
        [
            ['R.WheelF', velocity],
            ['R.WheelB', velocity],
            ['L.WheelF', -velocity],
            ['L.WheelB', -velocity]
        ]
    )

    if seconds > 0:
        time.sleep(seconds)
        motors_velset(
            lunohod,
            [
                ['R.WheelF', 0],
                ['R.WheelB', 0],
                ['L.WheelF', 0],
                ['L.WheelB', 0]
            ]
        )


# Выполнить команду
# Параметры:
#    port - порт, по которому необходимо отправить команду
#    command - команда
# Возаращаемое значение - список, состоящий из двух элементов:
#     0 - код ответа
#     1 - содержание полученного ответа
def exec_command(port, command):
    connection = connect(port)
    send_command(connection, command)

    response = connection.recv(4096)

    code = response[:1]
    body = response[1:]

    connection.close()

    return [code, body]


# Получить изображение с камеры (левой)
def get_capture():
    return cv2.VideoCapture('http://127.0.0.1:800')


# Отправить команду
# Параметры:
#    сonnection - подключение (объект класса socket.socket), через которое отправляется команда
#    command - команда
# Возвращаемое значение - количество отправленных байт
def send_command(connection, command):
    command += '\x0D\x0A'
    bytes_sent = connection.send(str2bytes(command))

    return bytes_sent


# Установить соединение
# Параметры:
#    port - порт, по которому необходимо установить соединение (адрес всегда 127.0.0.1)
#    socket_type - тип сокета, по умолчанию SOCK_STREAM
# Возвращаемое значение
#     Сокет (объект класса socket.socket)
def connect(port, socket_type=socket.SOCK_STREAM):
    soc = socket.socket(socket.AF_INET, socket_type)
    soc.connect(("127.0.0.1", port))

    return soc


def str2bytes(str):
    return bytes(str, 'windows-1251')


def bytes2str(data):
    return data.decode('windows-1251')


# Установить скорость вращения одного мотора
# Параметры:
#    port - порт для подключения к блоку управления
#    motor_id - идентификатор мотора
#    value - скорость вращения мотора
# Возвращаемое значение - ответ блока управления
def motor_velset(port, motor_id, value):
    command = 'ROBOT:MOTORS:' + motor_id + ':VELSET:' + str(value)
    response = exec_command(port, command)

    return response


# Получить угол, на который установлен указанный мотор
# Параметры:
#    port - порт для подключения к блоку управления
#    motor_id - идентификатор мотора
# Возвращаемое значение - угол, на который установлен указанный мотор (float)
def motor_posget(port, motor_id):
    command = 'ROBOT:MOTORS:' + motor_id + ':POSGET'
    response = exec_command(port, command)

    data = bytes2str(response[1]).split(';')

    return float(data[0])


# Установить угол для мотора
# Параметры:
#    port - номер порта для подключения к блоку управления
#    motor_id - идентификатор мотора
#    value - значение угла
# Возвращаемое значение - ответ блока управления
def motor_posset(port, motor_id, value):
    command = 'ROBOT:MOTORS:' + motor_id + ':POSSET:' + str(value)
    response = exec_command(port, command)

    return response


# Установить скорость вращения моторов
# Параметры:
#    port - номер порта для подключения к блоку управления
#    motors - cписок, содержащий идентификаторы моторов и скорость их вращения
# Возвращаемое значение -
def motors_velset(port, motors):
    return motors_set(port, 'VELSET', motors)


# Установить моторы в указанную позицию
# Параметры:
#    port - номер порта для подключения к блоку управления
#    motors - список моторов и углов, на который каждый из моторов должен быть установлен
def motors_posset(port, motors):
    return motors_set(port, 'POSSET', motors)


# Установить режим работы моторов
# Параметры:
#    port - номер порта для подключения к блоку управления
#    mode - режим работы моторов
#    motors - cписок, содержащий идентификаторы моторов и
#        значения для указанного в параметре mode режима работы мотора
def motors_set(port, mode, motors):
    command = 'ROBOT:MOTORS:'
    motors_list = ''
    vel_list = ''
    
    for motor in motors:
        motors_list += motor[0] + ';'
        vel_list += str(motor[1]) + ';'

    command += motors_list + ':' + mode + ':' + vel_list

    return exec_command(port, command)


# Получить момент (ток) выбранного привода
# Параметры
#    port - порт для подключения к блоку управления
#    motor_id - идентификатор мотора
def motor_torqget(port, motor_id):
    command = 'ROBOT:MOTORS:' + motor_id + ':TORQGET'
    response = exec_command(port, command)

    data = bytes2str(response[1]).split(';')

    return float(data[0])    


def find_obg(mot, mot_stop, sila, speed):
    motor_velset(robot, mot, speed)
    while True:
        if abs(motor_torqget(robot, mot_stop)) > sila:
            motor_velset(robot, mot, 0)
            break


def take_obg():
    motor_posset(robot, 'R.Finger.Index', 90)
    motor_posset(robot, 'R.Finger.Middle', 90)
    motor_posset(robot, 'R.Finger.Ring', 90)
    motor_posset(robot, 'R.Finger.Little', 90)

    time.sleep(3)
    motor_posset(robot, 'R.Finger.ThumbS', 0)
    time.sleep(3)
    motor_posset(robot, 'R.Finger.Thumb', 0)


# Установить угол поворота торса робота
# Параметры
#    target - угол, на который должен повернуться торс робота
# Возвращаемое значение - угол на который повёрнут торс робота
def set_robot_torsor(target):
    motor_id = 'TorsoR'
    current = motor_posget(robot, motor_id)

    if int(abs(target - current)) != 0:
        motor_posset(robot, motor_id, target)
        current = motor_posget(robot, motor_id)

        while int(abs(target - current)) > 0:
            current = motor_posget(robot, motor_id)
            print('diff:', abs(target - current))

    return current


# Проследовать от кнопки в ангар
def go_to():
    lunohod_move(-100,7)
    motor_posset(robot, 'HeadR', -90)
    motor_posset(robot, 'TorsoR', 0)
    wait_motor_posset(robot, 'L.ShoulderF', 0)
    motor_posset(robot, 'L.Elbow', 90)
    
    center = [400, 300]
    lunohod_rotate(50)
    while True:
        btn = get_button()
        if btn[0] > 0 and abs(btn[0] - center[0]) <= 5  :
            lunohod_rotate(0)
            break

    treangle(20)
    treangle(10)
    treangle(5)
    treangle(1)

    motor_posset(robot, 'HeadR', 0)
    treangle(1)
    
    lunohod_move(-100)
    motor_posset(robot, 'HeadR', -90)
    dist = lidar.get_distance()
    while True: 
        now_dist = lidar.get_distance()
        print(dist, now_dist)
        if now_dist - dist > 0.5:
            lunohod_move(-100, 5)
            print('uuu')
            break
    
    kak = exec_command(robot, 'ROBOT:SENSORS:IMU:GyroZ')[1].decode('utf-8')
    need_grad = float(kak.split(';')[0].replace(',', '.')) 
    set_grad(need_grad - 90)

    lunohod_move(-100)
    time.sleep(10)
    dist = lidar.get_distance()
    while True:
        now_dist = lidar.get_distance()
        print(dist, now_dist)
        if dist - now_dist < 0.5:
            lunohod_move(0)
            treangle(1)
            lunohod_move(-100)
            print('iii')
            break
        
    while True:
        now_dist = lidar.get_distance()
        print(dist, now_dist)
        if  now_dist - dist > 0.5:
            lunohod_move(0)
            print('u aa')
            break
        
    lunohod_move(100, 10)
        
    kak = exec_command(robot, 'ROBOT:SENSORS:IMU:GyroZ')[1].decode('utf-8')
    need_grad = float(kak.split(';')[0].replace(',', '.')) 
    set_grad(need_grad + 90)

    lunohod_move(100, 100)


# Подъехать к позиции, из которой видна кнопка открытия ворот ангара
def find_pos():
    center = [400, 300]
    
    lunohod_move(500)
    motor_posset(robot, 'HeadR', -90)
    target = center[0] + 40
    
    while True:
        btn = get_button()
        if btn[0] > target - 50 :
            lunohod_move(50)

        if target - btn[0] < 1 :
            lunohod_move(0)
            break

    motor_posset(robot, 'HeadR', 0)
    set_robot_torsor(0)
    lunohod_rotate(-50)
    while True:
        btn = get_button()
        if btn[0] > 0 and abs(btn[0] - center[0]) <= 2  :
            lunohod_rotate(0)
            print('wow')
            break


# Повернуть луноход вокруг вертикальной оси вращения таким образом,
# чтобы кнопка была в центре изображения, получаемого с камеры глаза
def rotate_to_button_R():
    center = [400, 300]
    
    LShoulderF = motor_posget(robot, 'L.ShoulderF')
    wait_motor_posset(robot, 'L.ShoulderF', 0)
    
    btn = get_button()

    if btn[0] > center[0] or btn[0] == 0:
        lunohod_rotate(-10)
    else:
        lunohod_rotate(10)
    
    while True:
        btn = get_button()
        #print(btn)

        if btn[0] == 0:
            lunohod_move(-10, 10)
            
        elif btn[0] > 0 and abs(btn[0] - center[0]) <= 5  :
            lunohod_rotate(0)
            break
        else:
            if btn[0] > center[0] or btn[0] == 0:
                lunohod_rotate(-10)
            else:
                lunohod_rotate(10)
    
    wait_motor_posset(robot, 'L.ShoulderF', LShoulderF)


# Выставить левую руку на угол, соответствующий положению кнопки
def rotate_to_button_F():
    print('rotate_to_button_F():')

    center = [400, 300]
    posHead = motor_posget(robot, 'HeadF')
    posNeck = motor_posget(robot, 'Neck')
    wait_motor_posset(robot, 'L.ShoulderF', 0)
    
    while True:
        btn = get_button()

        if btn[0] == 0:
            lunohod_move(-10, 5)

        if abs(btn[1] - center[1]) > 20:
            if btn[1] > center[1] :
                posHead += 0.5
                posNeck += 0.5
            else:
                posHead -= 0.5
                posNeck -= 0.5
                
            motor_posset(robot, 'HeadF', posHead)
            motor_posset(robot, 'Neck', posNeck)
            time.sleep(1)
            
        else:
            break

    posShoulder = (posHead+posNeck)/2 - 89
    print(
        'posHead:', posHead,
        'posNeck:', posNeck,
        '(posHead+posNeck)/2 - 89', posShoulder
    )
          
    if abs(posShoulder) < 70:
        posShoulder = -83
     
    #motor_posset(robot, 'L.ShoulderF', posShoulder)
    wait_motor_posset(robot, 'L.ShoulderF', posShoulder)
    
    motor_posset(robot, 'HeadF', 0)
    motor_posset(robot, 'Neck', 0)
     

# Получить координаты кнопки на изображении, получаемом левым глазом
# Возвращаемое значение - список значений:
#     [0] - координата по оси X,
#     [1] - координата по оси Y
def get_button():
    cap = get_capture()
    
    ret, img = cap.read()
    img = cv2.resize(img, (800, 600), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #hsv = cv2.blur(hsv, (5, 5))
    lo_green = np.array((50, 128, 80), np.uint8)
    hi_green = np.array((70, 255, 180), np.uint8)
   
    mask = cv2.inRange(hsv, lo_green, hi_green)
    #cv2.imshow('button', mask)
    contours_info = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contours = contours_info[0]
    #print(contours)

    if isinstance(contours, list) and len(contours) > 0:
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        (x, y, w, h) = cv2.boundingRect(contours[0])
    else:
        x, y, w, h = 0, 0, 0, 0

    return [int(x+w), int(y+h/2)]


# Подъехать к стене на определённое расстояние
# Параметры
#    ShoulderF - позиция (угол), на которую должен быть выставлен мотор ShoulderF
#    distance - расстояние, на которое необходимо подъехать к стене
#    adjust - надо ли следить за положение кнопки и докручивать луноход таким образом,
#        чтобы он ехал ровно на кнопку
#    try_poss -
def goto_wall(ShoulderF=-90, distance=0.65, adjust=False, try_poss=0):
    motor_posset(robot, 'L.Finger.Index', -30)
    time.sleep(10)
    lunohod_move(10)
    motor_posset(robot, 'L.Elbow', 0)

    if not wait_motor_posset(robot, 'L.ShoulderF', ShoulderF):
        lunohod_move(-30, 15)
        return False
        
    motor_posset(robot, 'L.Finger.Middle', -30)
    motor_posset(robot, 'L.Finger.Ring', -30)
    motor_posset(robot, 'L.Finger.Little', -30)
    motor_posset(robot, 'L.Finger.Thumb', -30)

    while True:
        now_poss = exec_command(robot, 'ROBOT:SENSORS:IMU:GyroZ')[1].decode('utf-8')
        now_try_poss = float(now_poss.split(';')[2].replace(',', '.'))
        # print(now_try_poss - try_poss)
        if now_try_poss - try_poss >= 5:
            wait_dist(0.65)
            lunohod_move(50)
        if adjust:
            center = [400, 300]
            btn = get_button()
            if btn[0] == 0:
                lunohod_move(-20, 10)
                
            elif abs(btn[0] - center[0]) > 20:
                rotate_to_button_R()
                time.sleep(1)
                lunohod_move(10)

        dst = lidar.get_distance()
        if dst > distance * 1.3 : 
            lunohod_move(100)
            
        elif distance < dst <= distance * 1.3 :
            #print('Distance:', dst, 'stop:', distance)
            lunohod_move(10)
            
        else: #elif dst <= distance :
            print('Distance:', dst)
            lunohod_move(0)
            return True
            
    return False


# Нажать на кнопку
# Параметры
#    dis - расстоние, по достижении которого необходимо прекратить давить на кнопку
def press_button(dis):
    poss = exec_command(robot, 'ROBOT:SENSORS:IMU:GyroZ')[1].decode('utf-8')
    try_poss = float(poss.split(';')[2].replace(',', '.')) 

    print('start press_button')
    rotate_to_button_R()
    rotate_to_button_F()
    ShoulderF = motor_posget(robot, 'L.ShoulderF')

    if abs(ShoulderF) < 70:
        lunohod_move(-20, 10)
        return False
    
    motor_posset(robot, 'HeadR', 16)
    motor_posset(robot, 'TorsoR', -16)
    result = goto_wall(ShoulderF, dis)  
    lunohod_move(-20, 10)
    print('finish press_button')

    return result


# Дохдаться пока указанный мотор не будет установлен в заданную позицию
def wait_motor_posset(unit, motor_id, target, tolerance=5): 
    motor_posset(unit, motor_id, target)
    attempts = 10 
    
    while True:
        time.sleep(1)
        current = motor_posget(unit, motor_id)
        
        attempts -= 1
        #print('attempts left', attempts)

        if attempts <= 0 or abs(target - current) <= tolerance:
            return True

    return False


# Установить угол поворота тележки вокруг вертикальной оси
def set_grad(grad_vol):
    grad_vol_f = ''
    poss = exec_command(robot, 'ROBOT:SENSORS:IMU:GyroZ')[1].decode('utf-8')
    try_poss = float(poss.split(';')[0].replace(',', '.')) 
    if grad_vol > try_poss:
        lunohod_rotate(-70)
    else:
        lunohod_rotate(70)
    while True:
        response = exec_command(robot, 'ROBOT:SENSORS:IMU:GyroZ')[1].decode('utf-8')
        grad_vol_f = float(response.split(';')[0].replace(',', '.'))
        if abs(grad_vol_f - grad_vol) < 0.5:
            lunohod_rotate(0)
            break


# Дождаться пока тележка не повернётся на необходимый градус
def wait_grad(grad):
    kak = exec_command(robot, 'ROBOT:SENSORS:IMU:GyroZ')[1].decode('utf-8')
    need_grad = float(kak.split(';')[0].replace(',', '.')) 
    set_grad(need_grad + grad)


# Откатиться от стены на требуемое расстояние
def wait_dist(need_dist):
    lunohod_move(-100)
    while True:
        dist = lidar.get_distance()
        if dist >= need_dist :
            lunohod_move(0)


# Повернуть тележку таким образом, чтобы расстояние до крайней правой и крайней левой точки было одинаковым
def treangle(speed):
    points = lidar.get_points(-2.5, 2.5, [1])
    if points[0]['distance'] <= points[-1]['distance']:
        Min = 0
        Max = -1
        lunohod_rotate(speed)
    else:
        lunohod_rotate(-1*speed)
        Max = 0
        Min = -1

    while True:
         points = lidar.get_points(-2.5, 2.5, [1])
         if points[Max]['distance'] <= points[Min]['distance']:
             # print(points[Max]['distance'], points[Min]['distance'])
             lunohod_rotate(0)
             break
