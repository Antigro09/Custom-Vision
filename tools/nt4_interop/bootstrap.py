#!/usr/bin/env python3
"""Prepare fixed released artifacts in this harness's ignored local cache.

Network downloads happen only with --download (and --download-jdk separately).
No package manager, global install, service, camera, or controller is used.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import tarfile
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parent
CACHE = ROOT / ".cache"
LOCK = json.loads((ROOT / "artifacts.lock.json").read_text())


def verify(path, record):
    if not path.is_file():
        raise FileNotFoundError(f"Missing {path}; run bootstrap.py --download first")
    if path.stat().st_size != record["bytes"]:
        raise ValueError(f"Wrong byte count: {path.name}")
    if hashlib.sha256(path.read_bytes()).hexdigest() != record["sha256"]:
        raise ValueError(f"SHA256 mismatch: {path.name}")


def obtain(path, record):
    if path.exists():
        verify(path, record)
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".part")
    try:
        with urllib.request.urlopen(record["url"], timeout=30) as response, temporary.open("wb") as target:
            count = 0
            while chunk := response.read(1024 * 1024):
                count += len(chunk)
                if count > record["bytes"]:
                    raise ValueError(f"Download exceeds pinned size: {path.name}")
                target.write(chunk)
        verify(temporary, record)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def java_home():
    local = CACHE / "jdk" / "jdk-25.0.4.1+1" / "Contents" / "Home"
    home = Path(os.environ.get("JAVA_HOME", local))
    javac = home / "bin" / "javac"
    version = subprocess.run([str(javac), "-version"], text=True, capture_output=True,
                             timeout=10, check=True).stdout.strip()
    if not version.startswith("javac 25."):
        raise RuntimeError(f"Java 25 required for alpha-7; got {version}")
    return home


def build():
    if platform.system() != "Darwin":
        raise RuntimeError("This native artifact lock is the macOS profile; no Linux/Windows claim")
    artifacts = CACHE / "artifacts"
    native = CACHE / "native"
    native.mkdir(parents=True, exist_ok=True)
    jars = []
    for record in LOCK["artifacts"]:
        path = artifacts / record["name"]
        verify(path, record)
        if path.suffix == ".jar":
            jars.append(str(path))
        else:
            with zipfile.ZipFile(path) as archive:
                # Flatten only the released shared libraries into the cache.
                for name in archive.namelist():
                    if name.endswith(".dylib") and name.startswith("osx/universal/shared/"):
                        (native / Path(name).name).write_bytes(archive.read(name))
    home = java_home()
    classes = CACHE / "classes"
    classes.mkdir(exist_ok=True)
    classpath = os.pathsep.join(jars)
    subprocess.run([str(home / "bin" / "javac"), "--release", "25", "-cp", classpath,
                    "-d", str(classes), str(ROOT / "Alpha7Server.java")], check=True, timeout=30)
    settings = {"java": str(home / "bin" / "java"), "classpath": str(classes) + os.pathsep + classpath,
                "native": str(native), "wpilib_version": LOCK["wpilib_version"]}
    (CACHE / "runtime.json").write_text(json.dumps(settings, indent=2) + "\n")
    print(json.dumps({"status": "built", "java_release": 25, "wpilib_version": LOCK["wpilib_version"]}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--download", action="store_true", help="Download pinned WPILib artifacts (~4 MB)")
    parser.add_argument("--download-jdk", action="store_true", help="Download isolated pinned macOS ARM JDK (~136 MB)")
    args = parser.parse_args()
    if args.download:
        for record in LOCK["artifacts"]:
            obtain(CACHE / "artifacts" / record["name"], record)
    if args.download_jdk:
        if (platform.system(), platform.machine()) != ("Darwin", "arm64"):
            raise RuntimeError("Pinned optional JDK archive is macOS ARM64 only; set JAVA_HOME to Java 25")
        jdk = LOCK["jdk"]
        path = CACHE / jdk["name"]
        obtain(path, jdk)
        destination = CACHE / "jdk"
        destination.mkdir(exist_ok=True)
        with tarfile.open(path) as archive:
            archive.extractall(destination, filter="data")  # Python 3.12+ safe extraction
    build()


if __name__ == "__main__":
    main()
