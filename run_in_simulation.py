#!/usr/bin/env python3
"""Execute a submission"""
import argparse
import os
import signal
import subprocess
import tempfile
import time
import json
import traceback
import socket
import logging
import pathlib
import sys


episode_length = 2 * 60 * 1000


class LocalExecutionConfig:
    """Configuration for local execution."""

    singularity_user_image = None
    singularity_backend_image = None
    host_output_dir = None
    robot_data_log_path = "/output/robot_data.dat"
    camera_data_log_path = "/output/camera_data.dat"
    singularity_binary = "singularity"
    visual = False

    git_repo = None
    git_branch = None
    git_ssh_command = None

    def __init__(self):
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--output-dir",
            "-o",
            type=str,
            required=True,
            help="""Path to the output directory.  All output files will be
                    stored there.""",
        )
        parser.add_argument(
            "--repository",
            "-r",
            type=str,
            required=True,
            help="""Git repository with the user code.""",
        )
        parser.add_argument(
            "--branch",
            "-b",
            type=str,
            default="master",
            help="""Branch of the Git repository that is used.""",
        )
        parser.add_argument(
            "--backend-image",
            type=str,
            required=True,
            help="""Path to the Singularity image for the backend.""",
        )
        parser.add_argument(
            "--user-image",
            type=str,
            help="""Path to the Singularity image for the user code.  If not
            specified, the same image as for the backend is used.""",
        )
        parser.add_argument(
            "--visualize",
            "-v",
            action="store_true",
            help="""Show visualization of the simulation.""",
        )
        args = parser.parse_args()

        self.host_output_dir = args.output_dir
        self.visual = args.visualize
        self.git_repo = args.repository
        self.git_branch = args.branch
        self.singularity_backend_image = os.path.abspath(args.backend_image)
        if args.user_image:
            self.singularity_user_image = os.path.abspath(args.user_image)
        else:
            self.singularity_user_image = self.singularity_backend_image


class SubmissionRunner:
    """Run a submission."""

    def __init__(self, config):
        """Initialize.

        Args:
            config:  Configuration structure, either SubmissionSystemConfig or
                LocalExecutionConfig.
        """
        self.config = config
        self.goal = None

    def clone_user_repository(self):
        """Clone the user repository."""
        logging.info(
            "Clone user git repository %s (%s)",
            self.config.git_repo,
            self.config.git_branch,
        )

        if self.config.git_ssh_command:
            os.environ["GIT_SSH_COMMAND"] = self.config.git_ssh_command

        git_cmd = [
            "git",
            "clone",
            "--recurse-submodules",
            "-b",
            self.config.git_branch,
            self.config.git_repo,
            "usercode",
        ]
        subprocess.run(git_cmd, check=True)

        # get current revision
        git_cmd = [
            "git",
            "--git-dir",
            "usercode/.git",
            "rev-parse",
            "HEAD",
        ]
        revision_bytes = subprocess.check_output(git_cmd)
        self.git_revision = revision_bytes.decode("utf-8").strip()

    def _sample_goal(self, difficulty: int):
        """Sample goal of the given difficulty level."""
        # sample the goal using the move_cube module in the container
        cmd = [
            self.config.singularity_binary,
            "run",
            "-eC",
            self.config.singularity_backend_image,
            "python3 -m trifinger_simulation.tasks move_cube sample_goal {:d}".format(
                difficulty
            ),
        ]
        try:
            output_bytes = subprocess.check_output(cmd)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(e.stdout.decode("utf-8"))

        # convert bytes to string
        output = output_bytes.decode("utf-8")

        goal_json = None
        for line in output.split("\n"):
            if line.startswith("pybullet build time"):
                continue
            else:
                goal_json = line
                break

        if not goal_json:
            raise RuntimeError("Failed to sample goal.")

        return goal_json

    def _validate_goal_file(self, source_path, filename):
        cmd = [
            self.config.singularity_binary,
            "run",
            "-eC",
            "-B",
            source_path,
            self.config.singularity_backend_image,
            (
                "python3 -m trifinger_simulation.tasks move_cube"
                " validate_goal_file {}".format(filename)
            ),
        ]
        try:
            subprocess.run(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=True
            )
        except subprocess.CalledProcessError as e:
            raise RuntimeError(e.stdout.decode("utf-8"))

    def load_goal(self, source_path: str):
        """Sample or load the goal for this submission."""
        # expect the user to provide a file "goal.json" at the root of the repository
        goal_file = os.path.join(source_path, "usercode/goal.json")
        try:
            self._validate_goal_file(source_path, goal_file)

            with open(goal_file, "r") as fh:
                goalconfig = json.load(fh)

            self.difficulty = int(goalconfig["difficulty"])
            if "goal" in goalconfig:
                goal = goalconfig["goal"]
                self.goal = json.dumps(goal)
            else:
                self.goal = self._sample_goal(self.difficulty)

        except Exception as e:
            raise RuntimeError(
                "Failed to load goal configuration.  Make sure you provide a valid"
                " 'goal.json' in your code repository.\n"
                " Error: %s" % e
            )

    def build_workspace(self, workspace_path):
        """Build the workspace with the user code."""
        logging.info("Build the user code")
        build_cmd = [
            self.config.singularity_binary,
            "exec",
            "--cleanenv",
            "--contain",
            "-B",
            "{}:/ws".format(workspace_path),
            self.config.singularity_user_image,
            "bash",
            "-c",
            ". /setup.bash; cd /ws; catbuild",
        ]
        proc = subprocess.run(
            build_cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
        )

        # store output
        stdout_file = os.path.join(self.config.host_output_dir, "build_output.txt")
        with open(stdout_file, "wb") as fh:
            fh.write(proc.stdout)

    def start_backend(self):
        """Start the backend."""
        logging.info("Run the backend")

        indicator_file_name = "backend_ready_indicator"
        self.backend_indicator_file = pathlib.Path(
            self.config.host_output_dir, indicator_file_name
        )

        # make sure the file does not exist in the beginning
        if self.backend_indicator_file.exists():
            self.backend_indicator_file.unlink()

        backend_rosrun_cmd = " ".join(
            [
                "rosrun trifinger_simulation pybullet_backend.py",
                "--finger-type trifingerpro",
                "--add-cube",
                "--real-time-mode",
                "--visualize" if self.config.visual else "",
                "--robot-logfile {}".format(self.config.robot_data_log_path),
                "--camera-logfile {}".format(self.config.camera_data_log_path),
                "--max-number-of-actions {}".format(episode_length),
                "--ready-indicator /output/{}".format(indicator_file_name),
            ]
        )

        run_backend_cmd = [
            self.config.singularity_binary,
            "exec",
            "--cleanenv",
            "--contain",
            # "--nv",  # FIXME
            "-B",
            "/dev,{}:/output".format(self.config.host_output_dir), #/etc/trifingerpro,
            self.config.singularity_backend_image,
            "bash",
            "-c",
            ". /setup.bash; {}".format(backend_rosrun_cmd),
        ]
        logging.info("Start backend")
        logging.debug(" ".join(run_backend_cmd))
        self.backend_process = subprocess.Popen(run_backend_cmd, start_new_session=True)

        logging.info("Wait until backend is ready...")
        start_time = time.time()
        while not self.backend_indicator_file.is_file():
            time.sleep(5)

            # if the backend takes too long to initialize, abort
            if (time.time() - start_time) > 60:
                logging.critical("FAILURE: Backend did not start in time.")
                return False

        logging.info("Backend is ready.")
        return True

    def stop_backend(self):
        """Stop the backend process."""

        # TODO timeout
        logging.info("Wait until backend has stopped")
        while self.backend_indicator_file.is_file():
            time.sleep(5)

            if self.backend_process.poll() is not None:
                # backend terminated without deleting the indicator file
                return False

        logging.info("Backend has stopped.  Give some time to store logs.")
        try:
            self.backend_process.wait(60)
            logging.info("Backend process terminated.")
            return True
        except subprocess.TimeoutExpired:
            pass

        logging.info("Backend still running.  Send SIGINT.")
        # the backend spawns several subprocesses by itself, so kill the whole process
        # group instead of just the main process (otherwise some processes will keep
        # running in the backgound).
        backend_pgid = os.getpgid(self.backend_process.pid)
        os.killpg(backend_pgid, signal.SIGINT)
        try:
            self.backend_process.wait(10)
        except subprocess.TimeoutExpired:
            logging.warning("Backend still running.  Send SIGTERM.")
            try:
                os.killpg(backend_pgid, signal.SIGTERM)
                self.backend_process.wait(3)
            except subprocess.TimeoutExpired:
                logging.error("Backend still running.  Send SIGKILL.")
                # FIXME this does not seem to kill everything, the pybullet gui is still
                # running when this script terminates...
                os.killpg(backend_pgid, signal.SIGKILL)
                self.backend_process.wait()

        return True

    def run_user_code(self, workspace_path):
        """Run the user script."""
        assert self.goal is not None

        logging.info("Run the user code.")

        # create user output directory if it does not yet exist
        user_output_dir = os.path.join(self.config.host_output_dir, "user")
        if not os.path.exists(user_output_dir):
            os.mkdir(user_output_dir)

        # store the goal to a file
        goal_file = os.path.join(self.config.host_output_dir, "goal.json")
        goal_info = {
            "difficulty": self.difficulty,
            "goal": json.loads(self.goal),
        }
        with open(goal_file, "w") as fh:
            json.dump(goal_info, fh, indent=4)

        # binding full /dev as only binding /dev/shm does not work with --contain
        exec_cmd = (
            ". /setup.bash;"
            ". /ws/devel/setup.bash;"
            "/ws/src/usercode/run {:d} {!r}"
        )
        run_user_cmd = [
            self.config.singularity_binary,
            "exec",
            "--cleanenv",
            "--contain",
            "-B",
            "{}:/ws,/dev,{}:/output".format(workspace_path, user_output_dir),
            self.config.singularity_user_image,
            "bash",
            "-c",
            exec_cmd.format(self.difficulty, self.goal),
        ]

        try:
            # TODO make sure the user cannot spawn processes that keep running after the
            # main one terminates (probably same method as for backend should be used).
            proc = subprocess.run(
                run_user_cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            logging.info("User code terminated.")
            stdout = proc.stdout
            stderr = proc.stderr
            returncode = proc.returncode
        except subprocess.CalledProcessError as e:
            logging.error(
                "User code exited with non-zero exist status: %d",
                e.returncode,
            )
            stdout = e.stdout
            stderr = e.stderr
            returncode = e.returncode
            # TODO: indicate this somehow to the user

        # store output
        stdout_file = os.path.join(self.config.host_output_dir, "user_stdout.txt")
        stderr_file = os.path.join(self.config.host_output_dir, "user_stderr.txt")
        with open(stdout_file, "wb") as fh:
            fh.write(stdout)
        with open(stderr_file, "wb") as fh:
            fh.write(stderr)

        return returncode

    def store_info(self):
        """Store some information about this submission into a file."""
        info = {
            "git_revision": self.git_revision,
            "robot_name": socket.gethostname(),
            "timestamp": time.asctime(),
        }
        info_file = os.path.join(self.config.host_output_dir, "info.json")
        with open(info_file, "w") as fh:
            json.dump(info, fh, indent=4)

    def store_report(self, backend_error, user_returncode):
        """Store a "report" file with some information about the result.

        This file contains some information whether execution was successful or
        if there was some error.  It is created at the very end, so it also
        serves as a indicator that the execution is over.
        """
        report = {
            "backend_error": backend_error,
        }

        if not backend_error:
            report["user_returncode"] = user_returncode

        report_file = os.path.join(self.config.host_output_dir, "report.json")
        with open(report_file, "w") as fh:
            json.dump(report, fh, indent=4)

    def run(self):
        """Run the whole pipeline."""
        try:
            with tempfile.TemporaryDirectory(prefix="run_submission-") as ws_dir:
                logging.info("Use temporary workspace %s", ws_dir)

                user_returncode = None

                # create "src" directory and cd into it
                src_dir = os.path.join(ws_dir, "src")
                os.mkdir(src_dir)
                os.chdir(src_dir)

                self.clone_user_repository()
                self.load_goal(src_dir)
                self.store_info()
                os.chdir(ws_dir)
                self.build_workspace(ws_dir)
                backend_okay = self.start_backend()

                if backend_okay:
                    user_returncode = self.run_user_code(ws_dir)
                    backend_okay = self.stop_backend()

                # create the report last, so it can be used as indicator that
                # the execution is over
                self.store_report(not backend_okay, user_returncode)

                logging.info("Finished.")
        except Exception as e:
            logging.critical("FAILURE: %s", e)

            # FIXME just for debugging, remove later
            traceback.print_exc()

            error_report_file = os.path.join(
                self.config.host_output_dir, "error_report.txt"
            )
            with open(error_report_file, "w") as fh:
                fh.write("Submission failed with the following error:\n{}\n".format(e))


def main():
    log_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(
        format="[SUBMISSION %(levelname)s %(asctime)s] %(message)s",
        level=logging.DEBUG,
        handlers=[log_handler],
    )

    config = LocalExecutionConfig()
    runner = SubmissionRunner(config)
    runner.run()


if __name__ == "__main__":
    main()
