# Linux kernet module for precise signal output and PWM based on the given time events list

> Work in progress

## Preparation
1. Install development environment 
```bash
sudo apt-get install build-essential linux-headers-`uname -r`
```
or
```bash
apt install raspberrypi-kernel{,-headers} 
```

2. Make (from folder with sources)
```bash
make
```

3. Insert the module:
```bash
sudo insmod ptime_control.ko
```
Check the module leaded:
```bash
lsmod | grep "dir ptime_control"
```

4. View kernel log for `printk`:
```bash
sudo dmesg
```

5. Remove module:
```bash
sudo rmmod ptime_control
```
