import pymurapi as mur

auv = mur.mur_init()

while True:
    auv.set_motor_power(0, 100)
    auv.set_motor_power(1, 100)

