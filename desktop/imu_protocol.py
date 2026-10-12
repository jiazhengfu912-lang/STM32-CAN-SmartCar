"""解码版本化IMU遥测；显示年龄与控制端50ms样本期限分别处理。"""
import math
import struct


def decode_imu(payload, transport_age=0, transport_valid=True):
    """校验长度、版本和浮点量，保留原始值并派生可显示字段。"""
    if len(payload) != 112 or payload[0] != 1:
        return {'valid': False, 'position_valid': False, 'error': 'IMU 协议不兼容'}
    flags, calibration, mode = payload[1:4]
    uptime, sample, sequence = struct.unpack_from('<3I', payload, 4)
    raw = struct.unpack_from('<7h', payload, 16)
    values = struct.unpack_from('<7f', payload, 32)
    if not all(math.isfinite(value) for value in values):
        return {'valid': False, 'position_valid': False, 'error': 'IMU 非有限数据'}
    age = ((uptime-sample) & 0xffffffff)+max(0, transport_age)
    valid = bool(transport_valid and flags & 1 and age < 500)
    data = dict(zip(('roll_rad', 'pitch_rad', 'yaw_rad', 'x_m', 'y_m', 'heading_rad', 'gyro_z_rad_s'), values))
    data.update(zip(('ax', 'ay', 'az', 'temperature_raw', 'gx', 'gy', 'gz'), raw))
    data.update(zip(('i2c_errors', 'i2c_timeouts', 'sample_gaps', 'control_interval_max_ms',
                     'sample_interval_max_ms', 'control_late', 'tx_dropped', 'origin_id'), struct.unpack_from('<8I', payload, 60)))
    data.update(zip(('target_left', 'target_right', 'rpm_left', 'rpm_right', 'output_left', 'output_right'), struct.unpack_from('<6h', payload, 92)))
    data.update(valid=valid, attitude_valid=bool(valid and flags & 32), position_valid=bool(valid and flags & 2), discontinuous=bool(flags & 4),
                configured=bool(flags & 8), axes_confirmed=bool(flags & 16), calibration=calibration,
                mode=mode, uptime_ms=uptime, sample_ms=sample, sequence=sequence, age_ms=age,
                result=payload[30], diagnostic=payload[31], state=payload[104], fault=payload[105],
                identity=payload[106], version=struct.unpack_from('<I', payload, 108)[0],
                temperature_c=(raw[3]/132.48+25 if payload[106]==0x47 else
                               raw[3]/340+36.53 if payload[106]==0x68 else None))
    return data
