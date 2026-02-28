import shutil
import time,datetime
import subprocess

import pexpect

import psutil
import os

def log_print(string):
    print(f"[{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')}] {string}")


class AutoUpdate:
    def __init__(self, interval, ip, git_address, temp_dir, password, executable_path):

        self.interval = interval

        self.git_address = git_address
        self.temp_dir = temp_dir
        self.password = password

        self.ip = ip

        self.executable_path = executable_path

        self.log_txt = os.path.join(self.temp_dir, "updatelog.txt")

    def fingerprint(self):
        cmd = f"ssh-keyscan -t ed25519 {self.ip} >> ~/.ssh/known_hosts"
        log_print(f"command: {cmd}")
        subprocess.run(cmd, shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=10)
        log_print("fingerprint finished")

    def clone(self,git_address, temp_dir, password):
        cmd = f"cd {temp_dir} && git clone -b main {git_address}"
        log_print(f"command: {cmd}")

        child = pexpect.spawn("/bin/bash", ["-c", cmd], timeout=60, encoding='utf-8')

        try:
            child.expect(r"password:", timeout=10)
            child.sendline(password)
            child.expect(pexpect.EOF, timeout=120)
        except pexpect.exceptions.TIMEOUT:
            log_print("Timeout, password prompt not appeared or too slow")
            return False
        except pexpect.exceptions.EOF:
            log_print("Process ended")
            return False
        finally:
            child.close()

        return True        
        # if child.exitstatus != 0:
        #     raise RuntimeError(f"git clone failed, exit code: {child.exitstatus}")
        

    def git_clone(self):
        log_print("Cloning...")

        if(os.path.exists(f"{self.temp_dir}/ai_algorithm_bin")):
            shutil.rmtree(f"{self.temp_dir}/ai_algorithm_bin")
        if(not self.clone(self.git_address, self.temp_dir, self.password)):
            log_print("try to clone project but failed")
            return False


        with open(self.log_txt, "a") as f:
            f.write(f"{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')} Clone finished\n")
        
        log_print(f"{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')} Clone finished")
        
        return True
        

    def is_executable_running(self, exec_path):
        abs_path = os.path.abspath(exec_path)
        for proc in psutil.process_iter(['pid', 'name', 'exe']):
            try:
                if proc.exe() == abs_path:
                    log_print(f"running! PID: {proc.pid}, cmdline: {proc.cmdline()}")
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return False

    def install(self):

        if self.is_executable_running(self.executable_path):
            return
        else:
            if not os.path.exists(f"{self.temp_dir}/ai_algorithm_bin"):
                return

            shutil.rmtree("/root/workspace/install/")
            shutil.copytree(
                f"{self.temp_dir}/ai_algorithm_bin", 
                "/root/workspace/install/"
            )

            with open(self.log_txt, "a") as f:
                f.write(f"{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')} update executable finished\n")
            
            log_print(f"{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')} update executable finished")

    def run_once(self):

        self.git_clone()

        next_run = time.time() + self.interval * 24 * 60 * 60 
    
        while time.time() < next_run:
            remaining_time = next_run - time.time()
            if remaining_time <= 0:
                break
            sleep_time = min(3600, remaining_time)
            time.sleep(sleep_time)

            # self.install()

    def run(self):

        result = subprocess.run(
            "systemctl stop perception.service",
            shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=300
        )

        if(self.git_clone()):
            self.install()

        result = subprocess.run(
            "systemctl restart perception.service",
            shell=True, executable="/bin/bash", capture_output=True, text=True, timeout=300
        )

        time.sleep(600)

        while True:
            self.run_once()

AutoUpdate_t = AutoUpdate(
    interval=1,
    ip = "61.129.72.17",
    git_address="git@61.129.72.17:/home/git/ai_algorithm_bin.git", 
    temp_dir="/home/sunrise/Desktop", 
    password="passgit5646",
    executable_path="/root/workspace/install/perception/lib/perception/perception_node"
)
AutoUpdate_t.fingerprint()
AutoUpdate_t.run()

# now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
# print(now_str)
