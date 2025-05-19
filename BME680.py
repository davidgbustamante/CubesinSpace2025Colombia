from machine import I2C
from micropython import const
import time, struct

# Direcciones I2C posibles del BME680
I2C_ADDR_PRIMARY = const(0x76)  # SDO a GND
I2C_ADDR_SECONDARY = const(0x77)  # SDO a VDD

# Registro de identificación de chip y valor esperado
_BME680_CHIPID = const(0x61)       # ID de chip esperado para BME680
_BME680_REG_CHIPID = const(0xD0)   # Registro de chip ID

# Registros base para coeficientes de calibración (según datasheet Bosch)
_BME680_COEFF_ADDR1 = const(0x89)  # Primer bloque de coeficientes (25 bytes)
_BME680_COEFF_ADDR2 = const(0xE1)  # Segundo bloque de coeficientes (16 bytes)

# Registros de control y configuración
_BME680_REG_SOFTRESET = const(0xE0)
_BME680_REG_CTRL_HUM = const(0x72)
_BME680_REG_STATUS = const(0xF3)
_BME680_REG_CTRL_MEAS = const(0x74)
_BME680_REG_CONFIG = const(0x75)
_BME680_REG_CTRL_GAS = const(0x71)
_BME680_REG_MEAS_STATUS = const(0x1D)

# Registros de datos (inicio de bloques de datos brutos)
_BME680_REG_PRESS_MSB = const(0x1F)  # PDATA [0x1F-0x21]
_BME680_REG_TEMP_MSB = const(0x22)   # TDATA [0x22-0x24]
_BME680_REG_HUM_MSB  = const(0x25)   # HDATA [0x25-0x26]

# Constantes para configuración de medición de gas
_BME680_RES_HEAT_0 = const(0x5A)   # Registro de resistencia de calentador (res_heat0)
_BME680_GAS_WAIT_0 = const(0x64)   # Registro de tiempo de espera del gas sensor (gas_wait0)
_BME680_RUNGAS = const(0x10)       # Valor para habilitar medición de gas (RUN_GAS=1)

# Tablas de conversión para cálculo de gas (provistas por Bosch/Adafruit)
_LOOKUP_TABLE_1 = (
    2147483647.0, 2147483647.0, 2147483647.0, 2147483647.0,
    2147483647.0, 2126008810.0, 2147483647.0, 2130303777.0,
    2147483647.0, 2147483647.0, 2143188679.0, 2136746228.0,
    2147483647.0, 2126008810.0, 2147483647.0, 2147483647.0
)
_LOOKUP_TABLE_2 = (
    4096000000.0, 2048000000.0, 1024000000.0, 512000000.0,
    255744255.0, 127110228.0, 64000000.0, 32258064.0,
    16016016.0, 8000000.0, 4000000.0, 2000000.0,
    1000000.0, 500000.0, 250000.0, 125000.0
)

def _read24(buf):
    # Convierte una lista/bytes de 3 bytes en un entero de 24 bits sin signo
    # (usa punto flotante para evitar overflow en MicroPython)
    result = 0.0
    for b in buf:
        result = result * 256 + float(b & 0xFF)
    return result

class BME680_I2C:
    """Driver para sensor BME680 en interfaz I2C."""
    def __init__(self, i2c: I2C, address: int = I2C_ADDR_PRIMARY):
        self.i2c = i2c
        self.address = address
        # Verificar comunicación leyendo el Chip ID
        self._write(_BME680_REG_SOFTRESET, b'\xB6')       # Reset suave del sensor
        time.sleep_ms(5)
        chip_id = self._read_byte(_BME680_REG_CHIPID)
        if chip_id != _BME680_CHIPID:
            raise OSError(f"BME680 no encontrado o ID incorrecto (0x{chip_id:02X})")
        # Leer coeficientes de calibración desde el sensor
        self._read_calibration()
        # Configurar sensor (oversampling y filtro por defecto)
        self._temp_oversample = 0b100      # oversampling x4 para temperatura (100 = 4×)
        self._pressure_oversample = 0b011  # oversampling x2 para presión   (011 = 2×)
        self._humidity_oversample = 0b010  # oversampling x2 para humedad   (010 = 2×)
        self._filter = 0b010              # filtro IIR coef 3 (0b010)
        # Configurar el sensor de gas (calentador) con valores por defecto
        # Estos valores corresponden aproximadamente a 320°C durante 150ms
        self._write(_BME680_RES_HEAT_0, b'\x73')  # 0x73 = 115 dec
        self._write(_BME680_GAS_WAIT_0, b'\x65')  # 0x65 = 101 dec (tiempo de calentamiento)
        # Variables para almacenar últimas lecturas ADC y coeficientes calibración
        self._t_fine = None
        self._adc_temp = None
        self._adc_pres = None
        self._adc_hum = None
        self._adc_gas = None
        self._gas_range = None
        # Timestamp de la última lectura para control de frecuencia
        self._last_reading = time.ticks_ms()
        # Intervalo mínimo entre lecturas (ms) según refresh_rate (por defecto 10 Hz -> 100ms)
        self._min_refresh_time = 1000 // 10

    def _read_calibration(self):
        """Lee los coeficientes de calibración desde el sensor y los guarda en variables internas."""
        # El primer byte de coeficientes en 0x88 es reservado, por eso se empieza en 0x89
        coeff1 = self._read(_BME680_COEFF_ADDR1, 25)  # Lee 25 bytes desde 0x89
        coeff2 = self._read(_BME680_COEFF_ADDR2, 16)  # Lee 16 bytes desde 0xE1
        coeff = bytes(coeff1 + coeff2)
        # Desempaquetar los coeficientes según orden y formato Little-Endian especificado por Bosch
        # (La cadena de formato '<hbBHhbBhhbbHhhBBBHbbbBbHhbb' corresponde al orden de 39 bytes de coef)
        calib = struct.unpack('<hbBHhbBhhbbHhhBBBHbbbBbHhbb', coeff[1:1+39])
        calib = [float(c) for c in calib]  # Convertir a float para cálculos
        # Asignar cada conjunto de coeficientes a su propósito
        # Coeficientes temperatura:
        self._temp_calib = [calib[23], calib[0], calib[1]]
        # Coeficientes presión:
        self._pres_calib = [calib[3], calib[4], calib[5], calib[7], calib[8],
                            calib[10], calib[9], calib[12], calib[13], calib[14]]
        # Coeficientes humedad:
        self._hum_calib = [calib[17], calib[16], calib[18], calib[19],
                           calib[20], calib[21], calib[22]]
        # Coeficientes gas:
        self._gas_calib = [calib[25], calib[24], calib[26]]
        # Ajustes adicionales para los coeficientes de humedad (H1 y H2 están en 4 bits diferentes)
        self._hum_calib[1] *= 16
        self._hum_calib[1] += self._hum_calib[0] % 16
        self._hum_calib[0] //= 16
        # Leer parámetros de calibración de gas adicionales
        self._heat_range = (self._read_byte(0x02) & 0x30) // 16
        self._heat_val = self._read_byte(0x00)
        self._sw_err = (self._read_byte(0x04) & 0xF0) // 16

    def _write(self, register: int, data: bytes):
        """Escribe uno o más bytes en un registro específico del BME680."""
        # Enviar por I2C: dirección, registro, y datos
        self.i2c.writeto_mem(self.address, register, data)

    def _read(self, register: int, length: int) -> list:
        """Lee una cantidad de bytes desde un registro específico."""
        return list(self.i2c.readfrom_mem(self.address, register, length))

    def _read_byte(self, register: int) -> int:
        """Lee un solo byte de un registro y devuelve su valor."""
        val = self.i2c.readfrom_mem(self.address, register, 1)
        return val[0] if val else 0

    def _perform_reading(self):
        """Realiza una medición forzada y actualiza las variables internas con datos brutos."""
        # Asegurarse de respetar la frecuencia de refresco (no leer muy seguido)
        elapsed = time.ticks_diff(time.ticks_ms(), self._last_reading)
        if elapsed < self._min_refresh_time:
            time.sleep_ms(int(self._min_refresh_time - elapsed))
        # Configurar oversampling y filtro en los registros correspondientes
        self._write(_BME680_REG_CONFIG, bytes([self._filter << 2]))
        self._write(_BME680_REG_CTRL_HUM, bytes([self._humidity_oversample]))
        self._write(_BME680_REG_CTRL_MEAS,
                    bytes([(self._temp_oversample << 5) | (self._pressure_oversample << 2)]))
        # Habilitar lectura de gas en el próximo ciclo de medición
        self._write(_BME680_REG_CTRL_GAS, bytes([_BME680_RUNGAS]))
        # Iniciar una medición simple forzando el bit 0 del registro CTRL_MEAS a 1
        ctrl_meas = self._read_byte(_BME680_REG_CTRL_MEAS)
        ctrl_meas = (ctrl_meas & 0xFC) | 0x01
        self._write(_BME680_REG_CTRL_MEAS, bytes([ctrl_meas]))
        # Esperar a que la medición termine (bit new_data en registro 0x1D)
        new_data = False
        while not new_data:
            status = self._read(_BME680_REG_MEAS_STATUS, 15)
            new_data = (status[0] & 0x80) != 0  # bit 7 de status: 1 = datos listos
            time.sleep_ms(5)
        # Marcar tiempo de esta lectura
        self._last_reading = time.ticks_ms()
        # Extraer datos brutos de los arrays leídos:
        adc_pres = _read24(status[2:5]) / 16   # 20 bits de presión
        adc_temp = _read24(status[5:8]) / 16   # 20 bits de temperatura
        adc_hum = struct.unpack('>H', bytes(status[8:10]))[0]  # 16 bits de humedad
        adc_gas_raw = struct.unpack('>H', bytes(status[13:15]))[0]
        adc_gas = int(adc_gas_raw / 64)        # 10 bits de resistencia gas
        gas_range = status[14] & 0x0F         # rango de gas usado en esta medida
        # Guardar valores ADC en propiedades del objeto
        self._adc_pres = adc_pres
        self._adc_temp = adc_temp
        self._adc_hum = adc_hum
        self._adc_gas = adc_gas
        self._gas_range = gas_range
        # Calcular t_fine para compensación de temperatura (usando fórmula Bosch)
        var1 = (adc_temp / 8.0) - (self._temp_calib[0] * 2.0)
        var2 = (var1 * self._temp_calib[1]) / 2048.0
        var3 = ((var1 / 2.0) * (var1 / 2.0)) / 4096.0
        var3 = (var3 * self._temp_calib[2] * 16.0) / 16384.0
        self._t_fine = int(var2 + var3)  # t_fine se usa en compensación de T, P, H

    @property
    def temperature(self) -> float:
        """Temperatura compensada en °C."""
        self._perform_reading()
        # Cálculo compensado de temperatura en °C (con dos decimales típicamente)
        # Fórmula: T = t_fine * 5 + 128 >> 8, luego /100 (según datasheet)
        temp = ((self._t_fine * 5 + 128) >> 8) / 100.0
        return temp

    @property
    def pressure(self) -> float:
        """Presión barométrica compensada en hPa."""
        self._perform_reading()
        # Cálculo compensado de presión (Pa) usando los coeficientes precalculados
        var1 = (self._t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self._pres_calib[5] / 131072.0
        var2 += var1 * self._pres_calib[4] * 2.0
        var2 = (var2 / 4.0) + (self._pres_calib[3] * 65536.0)
        var1 = (self._pres_calib[2] * var1 * var1 / 524288.0 + self._pres_calib[1] * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self._pres_calib[0]
        if var1 == 0:
            return 0  # evita división por cero
        pressure = 1048576.0 - self._adc_pres
        pressure = ((pressure - (var2 / 4096.0)) * 6250.0) / var1
        var1 = self._pres_calib[8] * pressure * pressure / 2147483648.0
        var2 = pressure * self._pres_calib[7] / 32768.0
        var3 = pressure * pressure * pressure * self._pres_calib[9] / 281474976710656.0
        pressure += (var1 + var2 + var3 + (self._pres_calib[6] * 128.0)) / 16.0
        return pressure / 100.0  # Pa -> hPa

    @property
    def humidity(self) -> float:
        """Humedad relativa compensada en %RH."""
        self._perform_reading()
        # Cálculo compensado de humedad relativa (%)
        temp_scaled = (self._t_fine * 5 + 128) >> 8
        var1 = self._adc_hum - (self._hum_calib[0] * 16.0) - ((temp_scaled * self._hum_calib[2]) / 200.0)
        var2 = (self._hum_calib[1] * (((temp_scaled * self._hum_calib[3]) / 100.0) +
        (((temp_scaled * ((temp_scaled * self._hum_calib[4]) / 100.0)) / 64.0) / 100.0) + 16384.0)) / 1024.0

        var3 = var1 * var2
        var4 = self._hum_calib[5] * 16384.0
        var5 = ((var3 / 16384.0) * (var3 / 16384.0)) / 1024.0
        var6 = (var4 + (temp_scaled * self._hum_calib[6] / 100.0)) / 16.0
        var6 = var6 * var5
        humidity = var3 / 1024.0 + var6 / 1024.0
        humidity = humidity * 100.0 / 16384.0  # valor en % humedad
        # Limitar el valor entre 0 y 100%
        if humidity > 100.0:
            humidity = 100.0
        elif humidity < 0.0:
            humidity = 0.0
        return humidity

    @property
    def gas(self) -> int:
        """Resistencia de gas (VOC) en ohms."""
        # Realiza una lectura (que incluye gas) y calcula la resistencia
        self._perform_reading()
        # Aplicar fórmulas de compensación para gas resistance (Ohm)
        var1 = (1340.0 + 5.0 * self._sw_err) * _LOOKUP_TABLE_1[self._gas_range] / 65536.0
        var2 = self._adc_gas * 32768.0 - 16777216.0 + var1
        var3 = _LOOKUP_TABLE_2[self._gas_range] * var1 / 512.0
        gas_resistance = int((var3 + var2 / 2.0) / var2)
        return gas_resistance
