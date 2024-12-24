import krpc
import math
import time


conn = krpc.connect(name = 'AutoPilot')
vessel = conn.space_center.active_vessel


#контрольные параметры
vessel.control.sas = False
vessel.control.rcs = False
vessel.auto_pilot.engage()
vessel.control.throttle = 1.0


print(vessel.name)
time.sleep(3)
print('3 секунды до старта...')
time.sleep(1)
print('2 секунды до старта...')
time.sleep(1)
print('1 секунда до старта...')
time.sleep(1)


#расчет угла по мат. модели
def vessel_alpha():
    alpha = 90 * math.exp(-(vessel.met / 170)) * 1.5
    return alpha

#выход на орбиту
vessel.control.activate_next_stage()
vessel.auto_pilot.target_pitch_and_heading(90, 0)
flag_stage_12 = False
flag_stage_11 = False
apoasis_flag = False
while True:
    earth_high = vessel.flight().surface_altitude
    print(vessel.resources_in_decouple_stage(0).amount('LiquidFuel'))
    if flag_stage_12 == False and vessel.resources_in_decouple_stage(0).amount('LiquidFuel') < 17700:
        vessel.control.throttle = 0.0
        time.sleep(1)
        vessel.control.activate_next_stage()
        vessel.control.throttle = 1.0
        time.sleep(1)
        flag_stage_12 = True
        print('Первая ступень отсоединилась')
    if flag_stage_11 == False and vessel.resources_in_decouple_stage(0).amount('LiquidFuel') < 6700:
        vessel.control.throttle = 0.0
        time.sleep(1)
        vessel.control.activate_next_stage()
        vessel.control.throttle = 1.0
        time.sleep(1)
        flag_stage_11 = True
        print('Вторая ступень отсоединилась. Закончилось топливо')
    if earth_high > 15000:
        vessel.auto_pilot.target_pitch_and_heading(vessel_alpha(), 90)
    if apoasis_flag == False and vessel.orbit.apoapsis_altitude > 180000:
        vessel.auto_pilot.target_pitch_and_heading(0, 90)
        apoasis_flag = True
        print('Достигнут требуемый апоцентр')
        time.sleep(1)
    if apoasis_flag == True:
        if flag_stage_11 == True:
            vessel.control.throttle = 0.0
            break
        else:
            print('Отстыковка второй ступени. Достигнут требуемый апоцентр')
            vessel.control.throttle = 0.0
            time.sleep(1)
            vessel.control.activate_next_stage()
            vessel.control.throttle = 1.0
            time.sleep(1)
            vessel.control.throttle = 0.0
            break

#орбита
tta = vessel.orbit.time_to_apoapsis
print(f'Время до апоцентра {tta}')
while tta > 10:
    tta = vessel.orbit.time_to_apoapsis
    continue

vessel.control.throttle = 1.0
while abs(vessel.orbit.periapsis_altitude - vessel.orbit.apoapsis_altitude) > 30000:
    continue
vessel.control.throttle = 0.0
print('Орбита выровнена')


#маневр
time.sleep(1)
def delta_v():
    earth = conn.space_center.bodies['Kerbin']
    moon = conn.space_center.bodies['Mun']

    earth_r = vessel.orbit.radius
    moon_r = moon.orbit.radius
    r_streak = moon_r / earth_r
    earth_grav_param = earth.gravitational_parameter
    delta_v1 = math.sqrt(earth_grav_param * (2 / earth_r - 2 / (earth_r + moon_r)) * (math.sqrt((2 * r_streak) / r_streak + 1) - 1) * 0.2)

    return delta_v1


ut = conn.space_center.ut
delva_v1 = delta_v()
print(delva_v1)
time.sleep(1)
print('Произведен расчет скорости')

vessel.control.add_node(ut + 60, prograde=delva_v1)
node_time = 5
while True:
    if vessel.control.nodes[0].orbit.next_orbit.body != conn.space_center.bodies['Mun']:
        vessel.control.nodes[0].remove()
        vessel.control.add_node(ut + node_time, prograde=delva_v1)
        node_time += 5
    else:
        break

print('Создан маневр')
while vessel.control.nodes[0].time_to > 30:
    continue

vessel.auto_pilot.reference_frame = vessel.control.nodes[0].reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)

vessel.control.throttle = 1.0
while vessel.control.nodes[0].remaining_delta_v > 0.5:
    continue

vessel.control.nodes[0].remove()
print('Совершен переход на орбиту Луны')


#стыковка
vessel.control.throttle = 0.0
vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()
#вручную переключить двигатели
#вручную снять обътекатель
#вручную отсоединить корабли
#вручную создать цель
status = input('Status: ')
while status != 'True':
    continue

vessel.control.activate_next_stage()

vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.control.throttle = 0.01
time.sleep(0.5)
vessel.control.throttle = 0.0
vessel.auto_pilot.reference_frame = conn.space_center.target_docking_port.part.reference_frame
vessel.auto_pilot.target_direction = (0, -1, 0)
vessel.auto_pilot.wait()
vessel.control.throttle = 0.015
time.sleep(2)
vessel.control.throttle = 0.0


#отсоединить сатурн-5
status = input('Status: ')
while status != 'True':
    continue

vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (0, -1, 0)
vessel.auto_pilot.wait()
print('Стыковка произведена')


#маневр на луне
while vessel.orbit.body != conn.space_center.bodies['Mun']:
    continue

delta_v2 = 700
while True:
    vessel.control.add_node(conn.space_center.ut + vessel.orbit.time_to_periapsis, -1 * delta_v2)
    if vessel.control.nodes[0].orbit.periapsis_altitude > 90000:
        vessel.control.nodes[0].remove()
        delta_v2 += 0.1
    else:
        break

print('Создан маневр')

while vessel.control.nodes[0].time_to > 5:
    continue

vessel.auto_pilot.reference_frame = vessel.control.nodes[0].reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)

vessel.control.throttle = 1.0
while vessel.control.nodes[0].remaining_delta_v > 0.5:
    continue

vessel.control.throttle = 0.0
vessel.control.nodes[0].remove()

delta_v2 = 1
while True:
    vessel.control.add_node(conn.space_center.ut + vessel.orbit.time_to_periapsis, -1 * delta_v2)
    if abs(vessel.control.nodes[0].orbit.periapsis_altitude - vessel.control.nodes[0].orbit.apoapsis_altitude) > 1000:
        vessel.control.nodes[0].remove()
        delta_v2 += 0.1
    else:
        break

print('Создан маневр')

vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (0, -1, 0)
while vessel.control.nodes[0].time_to > 5:
    continue

vessel.auto_pilot.reference_frame = vessel.control.nodes[0].reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.control.throttle = 1.0
while vessel.control.nodes[0].remaining_delta_v > 0.5:
    continue

print('Орбита выровнена')

vessel.control.throttle = 0.0
vessel.control.nodes[0].remove()
vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (0, -1, 0)


#переместить экипаж
#отстыковать
#включить двигатель
#переключиться на аппарат
status = input('Status: ')
while status != 'True':
    continue


#маневр посадки
delta_v2 = 1
vessel = conn.space_center.active_vessel
while True:
    vessel.control.add_node(conn.space_center.ut + 20, -1 * delta_v2)
    if vessel.control.nodes[0].orbit.periapsis_altitude > 1000:
        vessel.control.nodes[0].remove()
        delta_v2 += 0.1
    else:
        break

print('Создан маневр')

vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (0, -1, 0)
vessel.auto_pilot.engage()
while vessel.control.nodes[0].time_to > 5:
    continue

vessel.auto_pilot.reference_frame = vessel.control.nodes[0].reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()
vessel.control.throttle = 1.0
while vessel.control.nodes[0].remaining_delta_v > 0.5:
    continue

vessel.control.throttle = 0.0
vessel.control.nodes[0].remove()

vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
vessel.auto_pilot.target_direction = (0, -1, 0)


#посадка
print('Корабль садится')
while vessel.flight().surface_altitude > 22000:
    continue

status = input('Status: ')
while status != 'True':
    continue

vessel.control.throttle = 1.0
while vessel.orbit.speed > 1:
    continue


vessel.flight().retrograde


vessel.control.throttle = 0.0
while vessel.flight().surface_altitude > 9500:
    continue

vessel.auto_pilot.reference_frame = vessel.surface_velocity_reference_frame
vessel.auto_pilot.target_direction = (0, -1, 0)


while vessel.flight().surface_altitude > 3000:
    continue

vessel.control.throttle = 1.0
while vessel.orbit.speed > 60:
    continue
vessel.control.throttle = 0.0

#вручную выпустить ножки
#вручную вкючить свет

while vessel.flight().surface_altitude > 700:
    continue

vessel.control.throttle = 0.8
while vessel.orbit.speed > 30:
    continue

vessel.control.throttle = 0.0
while vessel.flight().surface_altitude > 250:
    continue


while vessel.flight().surface_altitude > 20:
    if vessel.orbit.speed > 15:
        vessel.control.throttle = 0.8
        while vessel.orbit.speed > 10:
            continue
        vessel.control.throttle = 0.0
    continue

vessel.control.throttle = 0.15
while vessel.flight().surface_altitude > 5:
    continue

vessel.control.throttle = 0.5
time.sleep(0.5)

vessel.control.throttle = 0.0
print('Кораль успешно приземлился на Луну,')
print('миссия завершена!')