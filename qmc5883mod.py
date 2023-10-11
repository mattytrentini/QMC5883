"""MicroPython module for QMC5883L or HMC5883L Geomagnetic Sensor"""
# import struct
# import micropython

# MicroPython
# mail: goctaprog@gmail.com
# MIT license
from sensor_pack import bus_service, geosensmod
from sensor_pack.base_sensor import check_value, Iterator, TemperatureSensor
# import time

_dly_ms = 100, 20, 7, 5


class QMC5883L(geosensmod.GeoMagneticSensor, Iterator, TemperatureSensor):
    """QMC5883L or HMC5883L Geomagnetic Sensor."""

    def __init__(self, adapter: bus_service.BusAdapter, address: int = 0x0D):
        self._buf_2 = bytearray((0 for _ in range(2)))  # для хранения
        self._buf_6 = bytearray((0 for _ in range(6)))  # для хранения
        self._update_rate = 0
        # адрес 0x0D!
        check_value(address, (0x0D,), f"Invalid address value: {address}")
        super().__init__(adapter=adapter, address=address, big_byte_order=False)     # little endian
        self.setup()

    def _read_reg(self, reg_addr: int, bytes_count: int = 1) -> bytes:
        """Считывает значение из регистра по адресу регистра 0..0x10. Смотри _get_reg_address"""
        return self.adapter.read_register(self.address, reg_addr, bytes_count)

    def _write_reg(self, reg_addr: int, value: int, bytes_count: int = 1):
        """Записывает в регистр с адресом reg_addr значение value по шине."""
        bo = self._get_byteorder_as_str()[0]
        self.adapter.write_register(self.address, reg_addr, value, bytes_count, bo)

    def _get_ctrl_1(self) -> int:
        """возвращает содержимое первого регистра управления"""
        return self._read_reg(0x09)[0]

    def get_id(self):
        """Возвращает значение (Chip ID), которое равно 0xFF!"""
        return self._read_reg(0x0D)[0]

    def get_temperature(self, coefficient: float = 0.02) -> [int, float]:
        # raw = self._read_reg(0x07, 2)
        buf = self._buf_2
        self.adapter.read_buf_from_mem(self.address, 0x07, buf)  # 16 bit value (int16)
        return coefficient * self.unpack("h", buf)[0]  # signed short

    def is_continuous_meas_mode(self):
        """Возвращает Истина, когда включен режим периодических измерений!"""
        return 0 != (0x01 & self._get_ctrl_1())

    def in_standby_mode(self) -> bool:
        """Возвращает Истина, когда включен режим ожидания(экономичный режим)!"""
        return 0 == (0x01 & self._get_ctrl_1())

    def get_status(self) -> tuple:
        """Возвращает кортеж битов(номер бита): Data Skip (DOR) (2), Overflow flag (OVL) (1), Data Ready (0)"""
        stat = self._read_reg(0x06)[0]
        #           DOR                 OVL                 DRDY
        return 0 != (stat & 0x04), 0 != (stat & 0x02), 0 != (stat & 0x01)

    def is_data_ready(self) -> bool:
        """Возвращает флаг Data Ready (DRDY)"""
        return self.get_status()[2]

    def soft_reset(self):
        """Выполняет програмный сброс датчика"""
        val = self._read_reg(0x0A)[0]
        self._write_reg(0x0A, 0x80 | val)

    def start_measure(self, continuous_mode: bool = True, update_rate: int = 0,
                      full_scale: bool = False, over_sample_ratio: int = 3):
        """Запускает периодические измерения (continuous_mode is True) или переводит датчик в
        режим ожидания (continuous_mode is False).
        update_rate: 0-10 Hz; 1-50 Hz; 2-100 Hz; 3-200 Hz.  For most of compassing applications, recommend 10 Hz!
        full_scale: False-2 Gauss; True-8 Gauss;            Field ranges of the magnetic sensor!
        over_sample_ratio: 0-512; 1-256; 2-128; 3-64.       Larger OSR value leads to smaller filter bandwidth,
                                                            less in-band noise and higher power consumption."""
        check_value(update_rate, range(4), f"Invalid update rate: {update_rate}")
        check_value(over_sample_ratio, range(4), f"Invalid over sample ratio: {update_rate}")
        ctrl_reg1_val = (over_sample_ratio << 6) | (int(full_scale) << 4) | (update_rate << 2) | int(continuous_mode)
        self._update_rate = update_rate     # сохраняю

        self._write_reg(0x09, ctrl_reg1_val)

    def read_raw(self, axis_name: int) -> int:
        addr_reg = geosensmod.axis_name_to_reg_addr(axis_name, offset=0, multiplier=2)
        buf = self._buf_2
        self.adapter.read_buf_from_mem(self.address, addr_reg, buf)     # 16 bit value (int16)
        # bts = self._read_reg(reg_addr=addr_reg, bytes_count=2)   # 16 bit value (int16)
        return self.unpack('h', buf)[0]

    def _get_all_meas_result(self) -> tuple:
        buf = self._buf_6
        # для уменьшения выделения/возвращения памяти сборщиком мусора (gc)
        self.adapter.read_buf_from_mem(self.address, 0, buf)
        return self.unpack('hhh', buf)

    def get_conversion_cycle_time(self) -> int:
        """Возвращает время, в микросекундах(!), преобразования датчиком в зависимости от его настроек.
        Для режима периодических измерений, устанавливает частоту обновления значений величины магнитного поля
        update_rate должно быть в диапазоне от 0 до 3 включительно, что соответствует частотам:
        0 - 10 Hz; 1 - 50 Hz; 2 - 150 Hz; 2 - 100 Hz; 3 - 200 Hz"""
        upd_rate = self._update_rate
        check_value(upd_rate, range(4), f"Invalid update rate: {upd_rate}")
        return _dly_ms[upd_rate]

    def setup(self):
        # roll-over function disabled, INT_ENB: “0”: enable interrupt PIN, “1”: disable interrupt PIN
        # bit_name      bit number
        # SOFT_RST      7
        # ROL_PNT       6
        # INT_ENB       0
        self._write_reg(reg_addr=0x0A, value=0x00)
        # SET/RESET Period. It is recommended that the register 0BH is written by 0x01.
        self._write_reg(reg_addr=0x0B, value=0x01)

    def __iter__(self):
        return self

    def __next__(self):
        """возвращает результат только в режиме периодических измерений!"""
        if self.is_continuous_meas_mode and self.is_data_ready():
            return self.get_axis(-1)
        return None
