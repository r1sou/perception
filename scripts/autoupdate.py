import shutil
import time,datetime
import subprocess

import pexpect

import psutil
import os

class AutoUpdate:
    def __init__(self, interval, git_address, temp_dir, password, executable_path):

        self.interval = interval

        self.git_address = git_address
        self.temp_dir = temp_dir
        self.password = password

        self.executable_path = executable_path

        self.log_txt = os.path.join(self.temp_dir, "updatelog.txt")

    def clone(self,git_address, temp_dir, password):
        cmd = f"cd {temp_dir} && git clone -b main {git_address}"

        child = pexpect.spawn("/bin/bash", ["-c", cmd], timeout=60, encoding='utf-8')

        try:
            child.expect(r"password:", timeout=10)
            child.sendline(password)
            child.expect(pexpect.EOF, timeout=120)
        except pexpect.exceptions.TIMEOUT:
            print("Timeout, password prompt not appeared or too slow")
        except pexpect.exceptions.EOF:
            print("Process ended")
        finally:
            child.close()
        
        if child.exitstatus != 0:
            raise RuntimeError(f"git clone failed, exit code: {child.exitstatus}")
        

    def git_clone(self):
        print("Cloning...")

        if(os.path.exists(f"{self.temp_dir}/ai_algorithm_bin")):
            shutil.rmtree(f"{self.temp_dir}/ai_algorithm_bin")
        self.clone(self.git_address, self.temp_dir, self.password)


        with open(self.log_txt, "a") as f:
            f.write(f"{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')} Clone finished\n")
        
        print(f"{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')} Clone finished\n")
        
        

    def is_executable_running(self, exec_path):
        abs_path = os.path.abspath(exec_path)
        for proc in psutil.process_iter(['pid', 'name', 'exe']):
            try:
                if proc.exe() == abs_path:
                    print(f"running! PID: {proc.pid}, cmdline: {proc.cmdline()}")
                    return True
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return False

    def install(self):

        if self.is_executable_running(self.executable_path):
            return
        else:
            shutil.rmtree("/root/workspace/install/")
            shutil.copytree(
                f"{self.temp_dir}/ai_algorithm_bin", 
                "/root/workspace/install/"
            )

            with open(self.log_txt, "a") as f:
                f.write(f"{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')} update executable finished\n")
            
            print(f"{datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')} update executable finished\n")

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

        self.git_clone()
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
    git_address="git@61.129.72.17:/home/git/ai_algorithm_bin.git", 
    temp_dir="/home/sunrise/Desktop", 
    password="passgit5646",
    executable_path="/root/workspace/install/perception/lib/perception/perception_node"
)
AutoUpdate_t.run()

# now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
# print(now_str)
