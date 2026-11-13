"""Recorder file ownership and process identity, without platform shell commands."""
import os
from pathlib import Path
import subprocess


class FileLock:
    def __init__(self, path):
        self.path = Path(path)
        self.stream = None

    def __enter__(self):
        self.stream = self.path.open('a+b')
        try:
            if os.name == 'nt':
                import msvcrt
                if self.path.stat().st_size == 0:
                    self.stream.write(b'\0'); self.stream.flush()
                self.stream.seek(0)
                msvcrt.locking(self.stream.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self.stream.close()
            raise BlockingIOError('Another recorder owns this directory') from exc
        return self

    def __exit__(self, *exc):
        if os.name == 'nt':
            import msvcrt
            self.stream.seek(0)
            msvcrt.locking(self.stream.fileno(), msvcrt.LK_UNLCK, 1)
        else:
            import fcntl
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
        self.stream.close()


def process_start(pid):
    """Return a PID-reuse-resistant identity, or None when it is no longer running."""
    try:
        pid = int(pid)
        if pid <= 0:
            return None
        if os.name != 'nt':
            proc = Path(f'/proc/{pid}/stat')
            if proc.exists():
                fields = proc.read_text().rsplit(') ', 1)[1].split()
                return None if fields[0] == 'Z' else fields[19]
            return subprocess.check_output(['ps', '-p', str(pid), '-o', 'lstart='],
                                           text=True, timeout=2).strip() or None
        import ctypes
        from ctypes import wintypes as w
        api = ctypes.WinDLL('kernel32', use_last_error=True)
        api.OpenProcess.argtypes = [w.DWORD, w.BOOL, w.DWORD]
        api.OpenProcess.restype = w.HANDLE
        api.CloseHandle.argtypes = [w.HANDLE]
        api.GetProcessTimes.argtypes = [w.HANDLE] + [ctypes.POINTER(w.FILETIME)] * 4
        api.GetExitCodeProcess.argtypes = [w.HANDLE, ctypes.POINTER(w.DWORD)]
        handle = api.OpenProcess(0x1000, False, pid)
        if not handle:
            return None
        try:
            code = w.DWORD()
            times = [w.FILETIME() for _ in range(4)]
            if not api.GetExitCodeProcess(handle, ctypes.byref(code)) or code.value != 259:
                return None
            if not api.GetProcessTimes(handle, *(ctypes.byref(value) for value in times)):
                return None
            return str((times[0].dwHighDateTime << 32) | times[0].dwLowDateTime)
        finally:
            api.CloseHandle(handle)
    except (OSError, ValueError, IndexError, subprocess.SubprocessError):
        return None


def detached_options():
    if os.name == 'nt':
        return dict(creationflags=subprocess.CREATE_NEW_PROCESS_GROUP | subprocess.CREATE_NO_WINDOW)
    return dict(start_new_session=True)
