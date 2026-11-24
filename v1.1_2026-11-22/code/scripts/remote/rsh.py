"""在目标机器上执行命令（paramiko）。

用法：
  python rsh.py "命令"                 执行一条命令
  python rsh.py --sudo "命令"          用 sudo 执行（自动送密码）
  python rsh.py --file 本地文件 [远端] 上传文件
  python rsh.py --get 远端文件 本地    下载文件
  python rsh.py --script 本地.sh       上传脚本并执行

环境变量可覆盖：RSH_HOST / RSH_USER / RSH_PASS / RSH_PORT
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

import paramiko

# Windows 控制台默认不是 UTF-8，远程返回的中文会乱码
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HOST = os.environ.get("RSH_HOST", "192.168.31.149")
USER = os.environ.get("RSH_USER", "shaobing")
PASS = os.environ.get("RSH_PASS", "202009")
PORT = int(os.environ.get("RSH_PORT", "22"))


def connect(timeout: float = 20.0) -> paramiko.SSHClient:
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, port=PORT, username=USER, password=PASS,
              timeout=timeout, banner_timeout=timeout, auth_timeout=timeout,
              look_for_keys=False, allow_agent=False)
    return c


def run(cmd: str, sudo: bool = False, timeout: float = 1800.0,
        get_pty: bool = False, echo: bool = True) -> tuple[int, str, str]:
    if sudo:
        cmd = f"sudo -S -p '' bash -lc {shell_quote(cmd)}"
    else:
        cmd = f"bash -lc {shell_quote(cmd)}"
    cli = connect()
    try:
        stdin, stdout, stderr = cli.exec_command(cmd, timeout=timeout,
                                                 get_pty=get_pty)
        if sudo:
            stdin.write(PASS + "\n")
            stdin.flush()
        out = stdout.read().decode("utf-8", "replace")
        err = stderr.read().decode("utf-8", "replace")
        rc = stdout.channel.recv_exit_status()
    finally:
        cli.close()
    if echo:
        if out.strip():
            print(out.rstrip())
        if err.strip():
            print("[stderr]", err.rstrip(), file=sys.stderr)
    return rc, out, err


def shell_quote(s: str) -> str:
    return "'" + s.replace("'", "'\"'\"'") + "'"


def put(local: str, remote: str) -> None:
    cli = connect()
    try:
        sftp = cli.open_sftp()
        _mkdirs(sftp, str(Path(remote).parent))
        sftp.put(local, remote)
        print(f"上传 {local} -> {remote}")
        sftp.close()
    finally:
        cli.close()


def get(remote: str, local: str) -> None:
    cli = connect()
    try:
        sftp = cli.open_sftp()
        sftp.get(remote, local)
        print(f"下载 {remote} -> {local}")
        sftp.close()
    finally:
        cli.close()


def _mkdirs(sftp, path: str) -> None:
    if not path or path == "/":
        return
    parts = Path(path).parts
    cur = ""
    for p in parts:
        cur = f"{cur}/{p}" if cur else p
        if cur in ("", "/"):
            continue
        try:
            sftp.stat(cur)
        except IOError:
            try:
                sftp.mkdir(cur)
            except IOError:
                pass


def main() -> int:
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 1
    if args[0] == "--sudo":
        rc, _, _ = run(" ".join(args[1:]), sudo=True)
        return rc
    if args[0] == "--file":
        put(args[1], args[2] if len(args) > 2 else f"/tmp/{Path(args[1]).name}")
        return 0
    if args[0] == "--get":
        get(args[1], args[2])
        return 0
    if args[0] == "--script":
        local = args[1]
        name = Path(local).name
        remote = f"/tmp/{name}"
        put(local, remote)
        rc, _, _ = run(f"chmod +x {remote} && {remote} " + " ".join(args[2:]),
                       sudo=False)
        return rc
    rc, _, _ = run(" ".join(args))
    return rc


if __name__ == "__main__":
    sys.exit(main())
