"""把本地目录递归上传到远程（paramiko）。

用法：python rput.py <本地目录> <远程目录>
"""
from __future__ import annotations

import os
import posixpath
import stat
import sys
from pathlib import Path

import paramiko

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HOST = os.environ.get("RSH_HOST", "192.168.31.149")
USER = os.environ.get("RSH_USER", "shaobing")
PASS = os.environ.get("RSH_PASS", "202009")

SKIP_DIRS = {".git", "__pycache__", ".pytest_cache", "build", "install", "log"}


def ensure_dir(sftp, path: str) -> None:
    parts, cur = path.strip("/").split("/"), ""
    for p in parts:
        cur = f"{cur}/{p}" if cur else f"/{p}"
        try:
            sftp.stat(cur)
        except IOError:
            sftp.mkdir(cur)


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    local_root = Path(sys.argv[1]).resolve()
    remote_root = sys.argv[2].rstrip("/")

    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, username=USER, password=PASS,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()
    n = 0
    try:
        ensure_dir(sftp, remote_root)
        for dirpath, dirnames, filenames in os.walk(local_root):
            dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
            rel = os.path.relpath(dirpath, local_root)
            rdir = remote_root if rel == "." else posixpath.join(
                remote_root, rel.replace(os.sep, "/"))
            ensure_dir(sftp, rdir)
            for f in filenames:
                lp = os.path.join(dirpath, f)
                rp = posixpath.join(rdir, f)
                sftp.put(lp, rp)
                n += 1
    finally:
        sftp.close()
        cli.close()
    print(f"上传 {n} 个文件 -> {remote_root}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
