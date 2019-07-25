#!/usr/bin/env sh
sleep 65
echo 1 > /sys/devices/system/cpu/cpu1/online
echo 1 > /sys/devices/system/cpu/cpu2/online
echo 1 > /sys/devices/system/cpu/cpu3/online
echo 1 > /sys/devices/system/cpu/cpu4/online
echo 1 > /sys/devices/system/cpu/cpu5/online

echo performance > /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
echo performance > /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor

echo 1120000000 > /sys/devices/17000000.gp10b/devfreq/17000000.gp10b/min_freq

echo 1600000000 > /sys/kernel/debug/bpmp/debug/clk/emc/rate
echo 1 > /sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked
