import socket
from concurrent.futures import ThreadPoolExecutor, as_completed
import netifaces
import os,json
import argparse
import subprocess
import pexpect

def fingerprint(server_ip):
    cmd = f"ssh-keygen -R {server_ip}"
    subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=10)

    cmd = f"ssh-keyscan -t ed25519 {server_ip} >> ~/.ssh/known_hosts"
    subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=10)

def get_wired_ip():
    ip_list = []
    wired_prefixes = ('en', 'eth')
    for interface in netifaces.interfaces():
        if interface == 'lo':
            continue
        if interface.startswith(wired_prefixes):
            addrs = netifaces.ifaddresses(interface)
            if netifaces.AF_INET in addrs:
                ip_info = addrs[netifaces.AF_INET][0]
                ip = ip_info['addr']
                ip_list.append(ip)
    return ip_list

def check_server_thread(ip):
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        result = sock.connect_ex((ip, 29701))
        sock.close()
        if result == 0:
            print(f"find WebSocket server: {ip}:29701")
            return ip
    except:
        pass
    return None

def get_server_ip(net):

    ip_list = [f"192.168.{net}.{i}" for i in range(1, 255)]

    with ThreadPoolExecutor(max_workers=8) as executor:
        futures = [executor.submit(check_server_thread, ip) for ip in ip_list]
        for future in as_completed(futures):
            ip = future.result()
            if ip:
                return ip
    
def auto_update_server_ip(args):

    host_ip = None
    server_ip = None

    for ip in get_wired_ip():
        server_ip = get_server_ip(ip.split('.')[2])
        if server_ip:
            client_config_path = args.file
            with open(client_config_path, 'r') as f:
                client_config = json.load(f)
            client_config['client']['ip'] = server_ip
            with open(client_config_path, 'w') as f:
                json.dump(client_config, f, indent=4, sort_keys=False)
            break
    
    if not server_ip:
        return

    with open("/home/sunrise/Desktop/ip.txt", 'w') as f:
        f.write(host_ip)

    fingerprint(server_ip)

    cmd = f"scp -r /home/sunrise/Desktop/ip.txt swbot@{host_ip}:/home/swbot/Desktop/"
    child = pexpect.spawn("/bin/bash", ["-c", cmd], timeout=60, encoding='utf-8')

    try:
        child.expect(r"password:", timeout=10)
        child.sendline("passswbot")
        child.expect(pexpect.EOF, timeout=120)
    except pexpect.exceptions.TIMEOUT:
        log_print("Timeout, password prompt not appeared or too slow")
        return False
    except pexpect.exceptions.EOF:
        log_print("Process ended")
        return False
    finally:
        child.close()

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, default='', help='store config path')
    return parser.parse_args()

if __name__ == '__main__':
    args = parse_args()
    auto_update_server_ip(args)