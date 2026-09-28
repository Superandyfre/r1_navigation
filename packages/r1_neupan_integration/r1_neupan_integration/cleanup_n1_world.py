#!/usr/bin/env python3
"""Stop only Gazebo processes launched with the selected N1 world."""

import os
import signal
import sys
import time
from pathlib import Path


def process_command(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as stream:
            return stream.read().replace(b'\0', b' ').decode(errors='replace').strip()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return ''


def process_parent(pid):
    try:
        with open(f'/proc/{pid}/stat', encoding='ascii') as stream:
            fields = stream.read().split()
        return int(fields[3])
    except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError, IndexError):
        return None


def world_path_aliases(world_path):
    """Return source/install spellings for the same workspace world file."""
    requested = os.path.abspath(world_path)
    aliases = {requested, os.path.realpath(requested)}
    parts = Path(requested).parts

    # colcon's symlink install commonly runs Gazebo with the install spelling
    # while an interrupted launch cleanup receives the source spelling (or
    # the reverse). Derive the paired path without matching by basename alone.
    if 'install' in parts:
        index = parts.index('install')
        if len(parts) > index + 3:
            workspace = Path(*parts[:index])
            package = parts[index + 1]
            relative = Path(*parts[index + 3:])
            aliases.add(str(workspace / 'src' / package / relative))
    if 'src' in parts:
        index = parts.index('src')
        if len(parts) > index + 2:
            workspace = Path(*parts[:index])
            package = parts[index + 1]
            relative = Path(*parts[index + 2:])
            aliases.add(str(workspace / 'install' / package / 'share' / package / relative))

    return {os.path.normpath(alias) for alias in aliases}


def matching_gazebo_pids(world_path):
    aliases = world_path_aliases(world_path)
    matches = set()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        pid = int(entry)
        command = process_command(pid)
        if not any(alias in command for alias in aliases):
            continue
        if ' gz sim ' in f' {command} ' or 'gzserver' in command or 'gzclient' in command:
            matches.add(pid)
    return matches


def descendants(roots):
    all_pids = {
        int(entry) for entry in os.listdir('/proc') if entry.isdigit()
    }
    selected = set(roots)
    changed = True
    while changed:
        changed = False
        for pid in all_pids - selected:
            parent = process_parent(pid)
            if parent in selected:
                selected.add(pid)
                changed = True
    return selected


def signal_pids(pids, sig):
    for pid in sorted(pids):
        if pid == os.getpid():
            continue
        try:
            os.kill(pid, sig)
        except (ProcessLookupError, PermissionError):
            pass


def main(args=None):
    del args
    if len(sys.argv) != 2:
        raise SystemExit('usage: cleanup_n1_world WORLD_SDF')
    world_path = os.path.realpath(sys.argv[1])
    roots = matching_gazebo_pids(world_path)
    pids = descendants(roots)
    if not pids:
        print(f'N1 world cleanup: no Gazebo process matched {world_path}', flush=True)
        return

    print(f'N1 world cleanup: stopping pids={sorted(pids)} for {world_path}', flush=True)
    # Launch shutdown already signals the parent `gz` wrapper.  Signal the
    # exact world child once and return immediately so a second launch event
    # cannot interrupt this fallback helper while it waits.
    signal_pids(pids, signal.SIGTERM)
    try:
        time.sleep(1.0)
    except KeyboardInterrupt:
        # launch may stop this helper when the parent gz process exits.  The
        # requested world has already received TERM, so this is a clean exit.
        return
    remaining = descendants(matching_gazebo_pids(world_path))
    if remaining:
        signal_pids(remaining, signal.SIGKILL)
