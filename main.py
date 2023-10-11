# MicroPython
# mail: goctaprog@gmail.com
# MIT license
# import time
# Пожалуйста, прочитайте документацию на QMC5883L!
# Please read the QMC5883L documentation!
import math
from machine import I2C, Pin

import qmc5883mod
import time
from sensor_pack.bus_service import I2cAdapter


if __name__ == '__main__':
    # пожалуйста установите выводы scl и sda в конструкторе для вашей платы, иначе ничего не заработает!
    # please set scl and sda pins for your board, otherwise nothing will work!
    # https://docs.micropython.org/en/latest/library/machine.I2C.html#machine-i2c
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) # для примера
    # bus =  I2C(scl=Pin(4), sda=Pin(5), freq=100000)   # на esp8266    !
    # Внимание!!!
    # Замените id=1 на id=0, если пользуетесь первым портом I2C !!!
    # Warning!!!
    # Replace id=1 with id=0 if you are using the first I2C port !!!
    # i2c = I2C(0, scl=Pin(13), sda=Pin(12), freq=400_000) # для примера

    # i2c = I2C(id=1, scl=Pin(27), sda=Pin(26), freq=400_000)  # on Arduino Nano RP2040 Connect and Pico W tested!
    i2c = I2C(id=1, scl=Pin(7), sda=Pin(6), freq=400_000)  # create I2C peripheral at frequency of 400kHz
    adapter = I2cAdapter(i2c)  # адаптер для стандартного доступа к шине
    delay_func = time.sleep_ms
    tc = .0175  # temperature coefficient

    sensor = qmc5883mod.QMC5883L(adapter)
    print(f"Sensor id: {sensor.get_id()}")
    print(16 * "_")

    upd_rate = 0
    print("Continuous meas mode measurement. 2 Milli-Gauss Field Resolution!")
    sensor.start_measure(continuous_mode=True, update_rate=upd_rate, full_scale=False, over_sample_ratio=3)
    print(f"Is continuous meas mode: {sensor.is_continuous_meas_mode()}")
    wt = sensor.get_conversion_cycle_time()
    delay_func(wt)
    index = 0
    for mf_comp in sensor:
        delay_func(wt)
        if mf_comp:
            x = math.sqrt(sum(map(lambda val: val ** 2, mf_comp)))  # напряженность магнитного поля в условных ед.
            print(f"X: {mf_comp[0]}; Y: {mf_comp[1]}; Z: {mf_comp[2]}; temp.[°C]: {sensor.get_temperature(tc)}, {x}")
        index += 1
        if index > 30:
            break

    print("Continuous meas mode measurement. 8 Milli-Gauss Field Resolution!")
    sensor.start_measure(continuous_mode=True, update_rate=upd_rate, full_scale=True, over_sample_ratio=3)
    print(f"Is continuous meas mode: {sensor.is_continuous_meas_mode()}")
    wt = sensor.get_conversion_cycle_time()
    delay_func(wt)
    for mf_comp in sensor:
        delay_func(wt)
        if mf_comp:
            x = math.sqrt(sum(map(lambda val: val ** 2, mf_comp)))  # напряженность магнитного поля в условных ед.
            print(f"X: {mf_comp[0]}; Y: {mf_comp[1]}; Z: {mf_comp[2]}; temp.[°C]: {sensor.get_temperature(tc)}, {x}")

    # while True:
    #     if sensor.is_data_ready():
    #         x = sensor.get_meas_result('x')
    #         y = sensor.get_meas_result('y')
    #         z = sensor.get_meas_result('z')
    #         print(x, y, z)
