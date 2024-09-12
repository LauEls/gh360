import subprocess
import os
import signal

from src.environments import Env

class ProcessHandler:
    def __init__(self):
        self.process_dict = {}
        self.process_commands_dict = {}
        for env in Env:
            self.process_dict[env] = []
            self.process_dict[env].append(subprocess.Popen('', shell=True, executable="/bin/bash"))
            self.process_commands_dict[env] = []

    def add_process(self, process_command: str, env: Env = Env.default):
        self.process_commands_dict[env].append(process_command)

    def run_process(self, env: Env = Env.default):
        process_running = False
        for e in Env:
            for process in self.process_dict[e]:
                if process.poll() is None:
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
                    process.wait()
                    process_running = True

        if not process_running:
            for process_command in self.process_commands_dict[Env.default]:
                self.process_dict[Env.default] = []
                self.process_dict[Env.default].append(subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid))

            if env != Env.default:
                for process_command in self.process_commands_dict[env]:
                    self.process_dict[env] = []
                    self.process_dict[env].append(subprocess.Popen(process_command, shell=True, executable="/bin/bash", preexec_fn=os.setsid))

            return True
        
        return False
    
    def close_process(self):
        for env in Env:
            for process in self.process_dict[env]:
                if process.poll() is None:
                    os.killpg(os.getpgid(process.pid), signal.SIGINT)
                    process.wait()
        return True