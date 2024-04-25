# coding=utf-8
# Insert in a script in Coppelia
# simRemoteApi.start(19999)
try:
    import sim
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

import numpy as np
import matplotlib.pyplot as plt
import time
import random

def SensorD(clientId=-1,
            range_data_signal_id="hokuyo_range_data",
            angle_data_signal_id="hokuyo_angle_data"):
    # the first call should be non-blocking to avoid getting out-of-sync angle data
    returnCodeRanges, string_range_data = sim.simxGetStringSignal(clientId, range_data_signal_id,
                                                                  sim.simx_opmode_streaming)

    # the second call should block to avoid out-of-sync scenarios
    # between your python script and the simulator's main loop
    # (your script may be slower than the simulator's main loop, thus
    # slowing down data processing)
    returnCodeAngles, string_angle_data = sim.simxGetStringSignal(clientId, angle_data_signal_id,
                                                                  sim.simx_opmode_blocking)

    # check the if both data were obtained correctly
    if returnCodeRanges == 0 and returnCodeAngles == 0:
        # unpack data from range and sensor messages
        raw_range_data = sim.simxUnpackFloats(string_range_data)
        raw_angle_data = sim.simxUnpackFloats(string_angle_data)

        return raw_range_data, raw_angle_data

    # return none in case were nothing was gotten from the simulator
    return None


print('Program started')
nrovezes = 240;
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:

    # comprimentos Pioneer
    L = 0.381  # Metros
    r = 0.0975  # Metros
    Resoluc = 0.025

    res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
    if res == sim.simx_return_ok:
        print('sucesso')
    else:
        print('Falha erro -->', res)

    sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot_wait)

    print('Conectado')
    sim.simxAddStatusbarMessage(clientID, 'Conexao activa...', sim.simx_opmode_oneshot_wait)
    robotname = 'Pioneer_p3dx'
    erro, robotHandle = sim.simxGetObjectHandle(clientID, robotname, sim.simx_opmode_oneshot_wait)
    # Handle RODAMENTOS
    returnCode, l_wheel = sim.simxGetObjectHandle(clientID, robotname + '_leftMotor', sim.simx_opmode_oneshot_wait)
    returnCode, r_wheel = sim.simxGetObjectHandle(clientID, robotname + '_rightMotor', sim.simx_opmode_oneshot_wait)
    # messagem coppelica--> inf robo
    [returnCode, positionrobot] = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_streaming)
    [returnCode, orientationrobot] = sim.simxGetObjectOrientation(clientID, robotHandle, -1, sim.simx_opmode_streaming)
    time.sleep(0.015)

    # Handle para os dados do LASER
    laser_range_data = "hokuyo_range_data"
    laser_angle_data = "hokuyo_angle_data"
    # inicio leitura
    returnCode = 1
    while returnCode != 0:
        returnCode, range_data = sim.simxGetStringSignal(clientID, laser_range_data, sim.simx_opmode_streaming + 10)

    raw_range_data, raw_angle_data = SensorD(clientID, laser_range_data, laser_angle_data)
    lasers = np.array([raw_angle_data, raw_range_data]).T
    returnCode, pos = sim.simxGetObjectPosition(clientID, robotHandle, -1, sim.simx_opmode_oneshot_wait)
    t = 0
    startTime = time.time()
    lastTime = startTime
    dt = 0
    i = 0
    theta = 0
    sim.simxAddStatusbarMessage(clientID, 'posicao obtida...', sim.simx_opmode_oneshot_wait)
    while t < nrovezes:
        now = time.time()
        dt = now - lastTime
        # laser
        raw_data, raw_angle_data = SensorD(clientID, laser_range_data, laser_angle_data)
        lasers = np.array([raw_angle_data, raw_data]).T
        RANGE_MAX = 5
        RANGE_LIMIT = 0.2
        PRIORI = random.gauss(1,0.50)
        if raw_angle_data[i] < RANGE_MAX * RANGE_LIMIT:
            taxaOC = 0.9
        else:
            taxaOC = 0.48

        elemento = 'ResizableFloor_5_25_element'  # CHAO ESQ DEREITA SUP
        erro, elementHandle = sim.simxGetObjectHandle(clientID, elemento,
                                                      sim.simx_opmode_oneshot_wait)  # handle/conexao com o robo
        returnCode, cantosup = sim.simxGetObjectPosition(clientID, elementHandle, -1, sim.simx_opmode_oneshot_wait)

        cantosupder = (cantosup[1] - 3.5)
        elemento = 'ResizableFloor_5_25_element7'  # nome do modelo
        erro, elementHandle = sim.simxGetObjectHandle(clientID, elemento,
                                                      sim.simx_opmode_oneshot_wait)  # handle/conexao com o robo
        returnCode, cantoinf = sim.simxGetObjectPosition(clientID, elementHandle, -1, sim.simx_opmode_oneshot_wait)

        cantoinfesq = (cantoinf[1] + 3.5)
        alturagrid = cantosupder/Resoluc
        larguragrid = cantoinfesq/Resoluc
        print('dimensao  ', alturagrid,larguragrid )
        posX = positionrobot[1]
        posY = positionrobot[2]

        xL = np.cos(raw_angle_data[i] + theta) * raw_angle_data[i] / Resoluc + (posX);
        # // + altgrid / 2;
        yL = np.sin(raw_angle_data[i] + theta) * raw_angle_data[i] / Resoluc + (posY);
        # posX e posY são as coordenadas do robô na GRID

        # Calcular todos as células de acordo com o algoritmo de Bresenham
        rows = abs(int(alturagrid))
        cols = abs(int(larguragrid))
        print(' rows',rows,'cols ',cols)
        line_bresenham = np.zeros((rows, cols), dtype=np.uint8)
        ca = 0
        # rr, cc = math.line(yL+RES, xL+RES, yL, xL)  # r0, c0, r1, c1
        x1, y1 = [xL, yL]
        x2, y2 = [xL + 3*Resoluc, yL + 3*Resoluc]
        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1)
        y2 = int(y2)
        dx = x2 - x1
        dy = y2 - y1

        is_steep = abs(dy) > abs(dx)  # pendente da linha
        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
            swapped = False

        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
        print(error)

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx

        print(points)

        # ATUALIZAR A GRID
        # Para cada célula da matriz por onde o feixe passa
        # Atualizar a GRID
        mxLyL = random.gauss(1,.5)
        apriori = mxLyL
        print('prob previa ', apriori)
        print(1 - pow(1 + ((taxaOC / (1 - taxaOC)) * ((1 - PRIORI) / PRIORI) * (apriori / (1 - apriori))), -1))
        mxLyL = 1 - pow(1 + ((taxaOC / (1 - taxaOC)) * ((1 - PRIORI) / PRIORI) * (apriori / (1 - apriori))), -1)
        print('prob nova ', mxLyL)

        # Atualizar xL, yL de acordo com o algoritmo de Bresenham
        # MAPEAMENTO - FIM

        # movimento ajustado
        v = 0
        w = np.deg2rad(0)

        sfrontal = int(len(lasers) / 2)
        sdireito = int(len(lasers) * 1 / 4)
        sesquerdo = int(len(lasers) * 3 / 4)

        if lasers[sfrontal, 1] < 0.6:
            v = 0.6
            w = np.deg2rad(-25)
        elif lasers[sdireito, 1] < 1:
            v = 0.3
            w = np.deg2rad(15)
        elif lasers[sesquerdo, 1] < 1:
            v = 0.3
            w = np.deg2rad(-25)
        else:
            v = 0.8
            w = 0

        #cinemática
        wl = v / r - (w * L) / (2 * r)
        wr = v / r + (w * L) / (2 * r)

        #  velocidades destino coppelia
        sim.simxSetJointTargetVelocity(clientID, l_wheel, wl, sim.simx_opmode_streaming + 5)
        sim.simxSetJointTargetVelocity(clientID, r_wheel, wr, sim.simx_opmode_streaming + 5)

        t = t + dt
        i = i + 1
        lastTime = now

    #parado robo e simulacao
    sim.simxSetJointTargetVelocity(clientID, r_wheel, 0, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(clientID, l_wheel, 0, sim.simx_opmode_oneshot_wait)
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)

else:
    print('Falha na conexao com remote API server')

print('Fechando programa')
