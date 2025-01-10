import os
import argparse

import tomllib
import paramiko
import stat

from typing import Mapping
from pathlib import Path
from colorama import Fore

import time


APP_CONFIG_FILE = "app.toml"


def handle_args():
    parser = argparse.ArgumentParser(prog="deploy")
    subparsers = parser.add_subparsers(
        dest="subcommand", help="action for deployment script to take", required=True
    )

    parser_clean = subparsers.add_parser(
        "clean",
        help="clean files on the target device",
    )
    parser_clean.add_argument(
        "-a",
        "--all",
        action="store_true",
        help="flag if the entire deployment directory should be wiped",
    )

    parser_load = subparsers.add_parser(
        "load", help="load files onto the target device"
    )

    parser_setup = subparsers.add_parser("setup", help="setup the device")
    parser_setup.add_argument("-l", "--load", action="store_true")

    parser_run = subparsers.add_parser(
        "run",
        help="run the file on the target device",
    )
    parser_run.add_argument(
        "-e", "--entry", help="alternative entry point to be executed"
    )
    parser_run.add_argument(
        "-l", "--load", action="store_true", help="loads files onto the target device"
    )

    args = parser.parse_args()

    return args


def main():
    # configuration provided in TOML file

    with open(APP_CONFIG_FILE, "rb") as f:
        config = tomllib.load(f)

    proj_path = Path(config["proj"]["proj_path"])
    app_path = proj_path / config["proj"]["app_name"]

    setup_script_path = app_path / config["exec"]["setup_script"]
    run_script_path = app_path / config["exec"]["run_script"]

    environment = {
        "VENV_NAME": config["proj"]["venv_name"],
        "APP_NAME": config["proj"]["app_name"],
        "ENTRY": config["exec"]["default_entry"],
    }

    args = handle_args()

    # setup the SSH instance with the RPi

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(
        hostname=config["rpi"]["hostname"],
        username=config["rpi"]["username"],
        password=config["rpi"]["password"],
    )

    sftp = ssh.open_sftp()

    # handles the potential options the script can take

    local_app_path = Path(os.path.dirname(__file__)) / config["proj"]["app_name"]

    if args.subcommand is None:
        # handle default deployment
        pass
    elif args.subcommand == "clean":
        # cleans the specified directory off the remote
        print(
            "Attempting to clean directory '{}'".format(
                proj_path if args.all else app_path
            )
        )
        recurse_clear_sftp_dir(sftp, proj_path if args.all else app_path)
    elif args.subcommand == "load":
        # loads/overwrites files on the remote
        load(sftp, local_app_path, proj_path, app_path)
    elif args.subcommand == "setup":
        # triggers the setup script
        if args.load:
            load(sftp, local_app_path, proj_path, app_path)
        run_shell_script(ssh, setup_script_path, proj_path, environment)
    elif args.subcommand == "run":
        # triggers the default run script
        if args.entry is not None:
            environment["ENTRY"] = args.entry
        if args.load:
            load(sftp, local_app_path, proj_path, app_path)
        run_shell_script(ssh, run_script_path, proj_path, environment)
    else:
        print(Fore.RED + f"Unsupported subcommand '{args.subcommand}'" + Fore.RESET)


def recurse_clear_sftp_dir(client: paramiko.SFTPClient, remote_dir: Path):
    try:
        entries = client.listdir_attr(str(remote_dir))
    except FileNotFoundError:
        print(Fore.RED + f"Unable to find directory: '{remote_dir}'" + Fore.RESET)
        exit()

    for entry in entries:
        file_path = f"{remote_dir}/{entry.filename}"

        if stat.S_ISDIR(entry.st_mode):
            recurse_clear_sftp_dir(client, file_path)
        else:
            print(f"\tRemoving file '{file_path}'")
            client.remove(file_path)
    print(f"Deleting directory '{remote_dir}'")
    client.rmdir(str(remote_dir))


def create_remote_dir(client: paramiko.SFTPClient, remote_path: Path):
    try:
        print(f"Creating directory:\n\tremote path: {remote_path}")
        client.mkdir(str(remote_path))
        print(Fore.GREEN + "Success" + Fore.RESET)
    except:
        print(Fore.RED + "Failed" + Fore.RESET)


def create_remote_file(
    client: paramiko.SFTPClient, local_path: Path, remote_path: Path
):
    try:
        print(
            f"Copying file:\n\tlocal path: {local_path}\n\tremote path: {remote_path}"
        )

        with open(local_path, "rb") as local_file:
            data = local_file.read()

        remote_file = client.file(str(remote_path), "wb")
        remote_file.write(data)
        remote_file.close()

        print(Fore.GREEN + "Success" + Fore.RESET)
    except:
        print(Fore.RED + "Failed" + Fore.RESET)


def load(
    client: paramiko.SFTPClient,
    local_app_path: Path,
    remote_proj_path: Path,
    remote_app_path: Path,
):
    create_remote_dir(client, remote_proj_path)
    create_remote_dir(client, remote_app_path)

    for root, dirs, files in os.walk(local_app_path, topdown=True):
        local_root = Path(root).relative_to(local_app_path)

        for dir in dirs:
            remote_dir_path = remote_app_path / local_root / dir
            create_remote_dir(client, remote_dir_path)

        for file in files:
            local_file_path = local_app_path / local_root / file
            remote_file_path = remote_app_path / local_root / file

            create_remote_file(client, local_file_path, remote_file_path)


def run_shell_script(
    client: paramiko.SSHClient,
    script_path: Path,
    cwd: Path,
    environment: Mapping[str, str],
):
    print(f"Running shell script '{script_path}' from working directory '{cwd}'")

    envs = ";".join(
        [f'export {env_key}="{env_val}"' for env_key, env_val in environment.items()]
    )
    command = f"{envs}; cd {str(cwd)}; bash {script_path}"

    stdin, stdout, stderr = client.exec_command(command=command, get_pty=True)
    while not stdout.channel.exit_status_ready():
        out = stdout.channel.recv(4096)
        print(out.decode(), end="")
        time.sleep(0.01)


if __name__ == "__main__":
    main()
