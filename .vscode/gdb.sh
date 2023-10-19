#!/bin/bash
sleep 1

while ! [ -f /home/user/build/RPiSingleAPM/.vscode/connect-ok ]; do
    sleep 0.2
done

sleep 2

proxychains -q /home/user/build/openwrt-sdk-rockchip-armv8_gcc-11.2.0_musl.Linux-x86_64/staging_dir/toolchain-aarch64_generic_gcc-11.2.0_musl/bin/aarch64-openwrt-linux-gdb "$@"
