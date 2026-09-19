"""Generate a private header under ignored build output; no credentials in flags/logs."""
import json
from pathlib import Path

Import('env')
root = Path(env['PROJECT_DIR']).parents[1]
config_path = root / 'config.local.json'
config = json.loads(config_path.read_text(encoding='utf-8')) if config_path.exists() else dict(ssid='', password='', token='')
if config['ssid']:
    assert 1 <= len(config['ssid'].encode()) <= 32
    assert 8 <= len(config['password'].encode()) <= 63
    assert len(config['token']) == 32 and all(c in '0123456789abcdef' for c in config['token'])
folder = root / 'out/esp32/generated'
folder.mkdir(parents=True, exist_ok=True)
# Each byte is a separate C escape, so quotes, backslashes and Unicode SSIDs stay literal.
def literal(value):
    """把 UTF-8 字节转为安全 C 字符串，不在日志输出凭据。"""
    return '"' + ''.join('\\%03o' % b for b in value.encode('utf-8')) + '"'
header = '#pragma once\n' + ''.join('static const char WIFI_%s[] = %s;\n' % (key.upper(), literal(config[key]))
                                    for key in ('ssid', 'password', 'token'))
destination = folder / 'local_config.h'
if not destination.exists() or destination.read_text() != header:
    destination.write_text(header)
env.Append(CPPPATH=[str(folder)])
