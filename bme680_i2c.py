# bme680_i2c.py - Driver real simplificado para BME680 en MicroPython
# Basado en lecturas directas de registros según datasheet

import time

class BME680_I2C:
    def __init__(self, i2c, address=0x77):
        self.i2c = i2c
        self.addr = address

        if self.addr not in self.i2c.scan():
            raise Exception("BME680 not found on I2C bus.")

        self._init_sensor()

    def _init_sensor(self):
        # Soft reset
        self.i2c.writeto_mem(self.addr, 0xE0, b'\xB6')
        time.sleep(0.01)

        # Configuración básica: oversampling de temperatura, presión y humedad
        self.i2c.writeto_mem(self.addr, 0x74, b'\x2F')  # ctrl_meas
        self.i2c.writeto_mem(self.addr, 0x72, b'\x01')  # ctrl_hum
        self.i2c.writeto_mem(self.addr, 0x75, b'\x10')  # config

    def _read_temp_raw(self):
        data = self.i2c.readfrom_mem(self.addr, 0x22, 3)
        raw_temp = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        return raw_temp

    def _read_pressure_raw(self):
        data = self.i2c.readfrom_mem(self.addr, 0x1F, 3)
        raw_press = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        return raw_press

    def _read_humidity_raw(self):
        data = self.i2c.readfrom_mem(self.addr, 0x25, 2)
        raw_hum = (data[0] << 8) | data[1]
        return raw_hum

    @property
    def temperature(self):
        # ⚠️ Valor bruto, sin calibración avanzada
        raw = self._read_temp_raw()
        return (raw / 16384.0) - 40  # Aproximación sencilla

    @property
    def pressure(self):
        raw = self._read_pressure_raw()
        return raw / 100.0  # Conversión básica a hPa

    @property
    def humidity(self):
        raw = self._read_humidity_raw()
        return raw / 1024.0  # Conversión simple a %

    @property
    def gas(self):
        return 1200  # Placeholder por ahora (requiere configuración compleja de calentador)

