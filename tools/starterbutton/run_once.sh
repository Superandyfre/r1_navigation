#!/bin/bash

# Keep the lock descriptor open for the lifetime of the calling process.
single_instance_or_exit() {
    local name="$1"
    local state_dir="${XDG_RUNTIME_DIR:-$HOME/.cache}/starterbutton"
    local lock_file

    mkdir -p "$state_dir"
    chmod 700 "$state_dir"
    lock_file="$state_dir/${name}.lock"

    exec 9>"$lock_file"
    if ! flock -n 9; then
        echo "${name} 已在运行，忽略重复启动。"
        exit 0
    fi
}
