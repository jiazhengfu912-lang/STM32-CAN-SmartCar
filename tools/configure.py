"""仅在本机终端输入 Wi-Fi 凭据，写入被 Git 忽略的本地配置；不访问设备。"""
from pathlib import Path
import getpass
import json
import secrets
import ipaddress
from urllib.parse import urlsplit


def main():
    """逐项检查 Wi-Fi 和局域网地址，使用随机认证令牌；不会打印密码或令牌。"""
    root=Path(__file__).resolve().parents[1];path=root/'config.local.json'
    previous=json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
    ssid=input('2.4 GHz Wi-Fi SSID: ').strip()
    password=getpass.getpass('Wi-Fi password (hidden): ')
    gateway=input('ESP32 URL (example http://172.20.10.4): ').strip().rstrip('/')
    parsed=urlsplit(gateway)
    if parsed.scheme!='http' or parsed.username or parsed.password or parsed.path or parsed.query or parsed.fragment:
        raise ValueError('请使用不带路径的局域网 HTTP 地址')
    address=ipaddress.ip_address(parsed.hostname)
    if not address.is_private or address.is_loopback or address.is_multicast: raise ValueError('地址必须是局域网单播 IP')
    if not 1<=len(ssid.encode())<=32 or not 8<=len(password.encode())<=63: raise ValueError('SSID 或密码长度不符合要求')
    token=previous.get('token') or secrets.token_hex(16)
    path.write_text(json.dumps(dict(ssid=ssid,password=password,token=token,gateway=gateway),ensure_ascii=False,indent=2),encoding='utf-8')
    print('Local configuration saved. Rebuild ESP32 firmware to apply Wi-Fi/token changes.')


if __name__=='__main__':main()
