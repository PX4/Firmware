#!/usr/bin/env python3

import queue
import time
import os
import atexit
import subprocess
import threading
import errno
from typing import Any, Dict, List, TextIO, Optional


class Runner:
    def __init__(self,
                 log_dir: str,
                 model: str,
                 case: str,
                 verbose: bool):
        self.name = ""
        self.cmd = ""
        self.cwd = ""
        self.args: List[str]
        self.env: Dict[str, str]
        self.model = model
        self.case = case
        self.log_filename = ""
        self.log_fd: TextIO
        self.verbose = verbose
        self.output_queue: queue.Queue[str] = queue.Queue()
        self.start_time = time.time()
        self.log_dir = log_dir
        self.log_filename = ""
        self.stop_thread: Any[threading.Event] = None

    def set_log_filename(self, log_filename: str) -> None:
        self.log_filename = log_filename

    def get_log_filename(self) -> str:
        return self.log_filename

    def start(self) -> None:
        if self.verbose:
            print("Running: {}".format(" ".join([self.cmd] + self.args)))

        atexit.register(self.stop)

        if self.verbose:
            print("Logging to {}".format(self.log_filename))
        self.log_fd = open(self.log_filename, 'w')

        self.process = subprocess.Popen(
            [self.cmd] + self.args,
            cwd=self.cwd,
            env=self.env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True
        )

        self.stop_thread = threading.Event()
        self.thread = threading.Thread(target=self.process_output)
        self.thread.start()

    def process_output(self) -> None:
        assert self.process.stdout is not None
        while True:
            line = self.process.stdout.readline()
            if not line and \
                    (self.stop_thread.is_set() or self.poll is not None):
                break
            if not line or line == "\n":
                continue
            self.output_queue.put(line)
            self.log_fd.write(line)
            self.log_fd.flush()

    def poll(self) -> Optional[int]:
        return self.process.poll()

    def wait(self, timeout_min: float) -> Optional[int]:
        try:
            return self.process.wait(timeout=timeout_min*60)
        except subprocess.TimeoutExpired:
            print("Timeout of {} min{} reached, stopping...".
                  format(timeout_min, "s" if timeout_min > 1 else ""))
            self.stop()
            print("stopped.")
            return errno.ETIMEDOUT

    def get_output_line(self) -> Optional[str]:
        while True:
            try:
                return self.output_queue.get(block=True, timeout=0.1)
            except queue.Empty:
                return None

    def stop(self) -> int:
        atexit.unregister(self.stop)

        if not self.stop_thread:
            return 0

        returncode = self.process.poll()
        if returncode is None:

            if self.verbose:
                print("Terminating {}".format(self.cmd))
            self.process.terminate()

            try:
                returncode = self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                pass

            if returncode is None:
                if self.verbose:
                    print("Killing {}".format(self.cmd))
                self.process.kill()
                returncode = self.process.poll()

        if self.verbose:
            print("{} exited with {}".format(
                self.cmd, self.process.returncode))

        self.stop_thread.set()
        self.thread.join()
        self.log_fd.close()

        return self.process.returncode

    def time_elapsed_s(self) -> float:
        return time.time() - self.start_time

    def add_to_env_if_set(self, var: str) -> None:
        if var in os.environ:
            self.env[var] = os.environ[var]


class Px4Runner(Runner):
    def __init__(self, workspace_dir: str, log_dir: str,
                 model: str, case: str, speed_factor: float,
                 debugger: str, verbose: bool):
        super().__init__(log_dir, model, case, verbose)
        self.name = "px4"
        self.cmd = workspace_dir + "/build/px4_sitl_default/bin/px4"
        self.cwd = workspace_dir + "/build/px4_sitl_default/tmp/rootfs"
        self.args = [
                workspace_dir + "/ROMFS/px4fmu_common",
                "-s",
                "etc/init.d-posix/rcS",
                "-t",
                workspace_dir + "/test_data",
                "-d"
            ]
        self.env = {"PATH": str(os.environ['PATH']),
                    "PX4_SIM_MODEL": self.model,
                    "PX4_SIM_SPEED_FACTOR": str(speed_factor)}
        self.debugger = debugger

        if not self.debugger:
            pass
        elif self.debugger == "valgrind":
            self.args = ["--track-origins=yes", "--leak-check=full", "-v",
                         self.cmd] + self.args
            self.cmd = "valgrind"
        elif self.debugger == "callgrind":
            self.args = ["--tool=callgrind", "-v", self.cmd] + self.args
            self.cmd = "valgrind"
        elif self.debugger == "gdb":
            self.args = ["--args", self.cmd] + self.args
            self.cmd = "gdb"
        else:
            print("Using custom debugger " + self.debugger)
            self.args = [self.cmd] + self.args
            self.cmd = self.debugger


class GzserverRunner(Runner):
    def __init__(self,
                 workspace_dir: str,
                 log_dir: str,
                 model: str,
                 case: str,
                 speed_factor: float,
                 verbose: bool):
        super().__init__(log_dir, model, case, verbose)
        self.name = "gzserver"
        self.cwd = workspace_dir
        self.env = {"PATH": os.environ['PATH'],
                    "HOME": os.environ['HOME'],
                    "GAZEBO_PLUGIN_PATH":
                    workspace_dir + "/build/px4_sitl_default/build_gazebo",
                    "GAZEBO_MODEL_PATH":
                    workspace_dir + "/Tools/sitl_gazebo/models",
                    "PX4_SIM_SPEED_FACTOR": str(speed_factor)}
        self.add_to_env_if_set("DISPLAY")
        self.add_to_env_if_set("PX4_HOME_LAT")
        self.add_to_env_if_set("PX4_HOME_LON")
        self.add_to_env_if_set("PX4_HOME_ALT")
        self.cmd = "gzserver"
        self.args = ["--verbose",
                     workspace_dir + "/Tools/sitl_gazebo/worlds/" +
                     "empty.world"]


class GzmodelspawnRunner(Runner):
    def __init__(self,
                 workspace_dir: str,
                 log_dir: str,
                 model: str,
                 case: str,
                 verbose: bool):
        super().__init__(log_dir, model, case, verbose)
        self.name = "gzmodelspawn"
        self.cwd = workspace_dir
        self.env = {"PATH": os.environ['PATH'],
                    "HOME": os.environ['HOME'],
                    "GAZEBO_PLUGIN_PATH":
                    workspace_dir + "/build/px4_sitl_default/build_gazebo",
                    "GAZEBO_MODEL_PATH":
                    workspace_dir + "/Tools/sitl_gazebo/models"}
        self.add_to_env_if_set("DISPLAY")
        self.cmd = "gz"
        self.args = ["model", "--spawn-file", workspace_dir +
                     "/Tools/sitl_gazebo/models/" +
                     self.model + "/" + self.model + ".sdf",
                     "--model-name", self.model,
                     "-x", "1.01", "-y", "0.98", "-z", "0.83"]


class GzclientRunner(Runner):
    def __init__(self,
                 workspace_dir: str,
                 log_dir: str,
                 model: str,
                 case: str,
                 verbose: bool):
        super().__init__(log_dir, model, case, verbose)
        self.name = "gzclient"
        self.cwd = workspace_dir
        self.env = dict(os.environ, **{
            "GAZEBO_MODEL_PATH": workspace_dir + "/Tools/sitl_gazebo/models"})
        self.add_to_env_if_set("DISPLAY")
        self.cmd = "gzclient"
        self.args = ["--verbose"]


class TestRunner(Runner):
    def __init__(self,
                 workspace_dir: str,
                 log_dir: str,
                 model: str,
                 case: str,
                 mavlink_connection: str,
                 verbose: bool):
        super().__init__(log_dir, model, case, verbose)
        self.name = "mavsdk_tests"
        self.cwd = workspace_dir
        self.env = {"PATH": os.environ['PATH']}
        self.cmd = workspace_dir + \
            "/build/px4_sitl_default/mavsdk_tests/mavsdk_tests"
        self.args = ["--url", mavlink_connection, case]
