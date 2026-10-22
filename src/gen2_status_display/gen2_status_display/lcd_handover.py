"""LCD ownership between gen2-bootscreen.service (root, early boot) and the status_display node.

Only one process may drive the ST7789 (SPI + DC/RST/BL GPIO). The owner holds an flock on LOCK.
The status node, before opening the panel, creates REQUEST and waits for the lock; the boot screen
sees REQUEST, draws a last frame and exits (the kernel drops the lock with the process).
"""
import fcntl
import os
import time

LOCK = '/dev/shm/gen2_lcd.lock'
REQUEST = '/dev/shm/gen2_lcd.request'


def _open_lock():
    # No O_CREAT on an existing file: /dev/shm is sticky + world-writable, and fs.protected_regular
    # refuses O_CREAT on another user's file there even for root (root boot screen vs. user node).
    try:
        return os.open(LOCK, os.O_RDWR)
    except FileNotFoundError:
        fd = os.open(LOCK, os.O_RDWR | os.O_CREAT, 0o666)
    try:
        os.fchmod(fd, 0o666)  # root creates it at boot, the node runs as the user
    except OSError:
        pass
    return fd


def try_own():
    """Boot screen: take the panel if nobody has it. Returns the lock fd or None."""
    fd = _open_lock()
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        os.close(fd)
        return None
    try:
        os.unlink(REQUEST)  # stale request from a previous boot (/dev/shm is tmpfs, but be safe)
    except OSError:
        pass
    return fd


def requested():
    return os.path.exists(REQUEST)


def take_over(timeout_s=4.0):
    """Status node: ask the boot screen to leave and wait for the lock.

    Returns the lock fd (keep it open for the life of the process). After timeout_s the panel is
    taken anyway (a hung boot screen must not keep the status screen off) and None is returned.
    """
    try:
        fd = _open_lock()
    except OSError:
        return None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return fd
    except OSError:
        pass
    try:
        with open(REQUEST, 'w'):
            pass
        os.chmod(REQUEST, 0o666)
    except OSError:
        pass
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        time.sleep(0.05)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            try:
                os.unlink(REQUEST)
            except OSError:
                pass
            time.sleep(0.1)  # let the old owner's GPIO lines finish releasing
            return fd
        except OSError:
            pass
    os.close(fd)
    return None
