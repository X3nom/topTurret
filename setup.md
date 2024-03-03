# (all) the things I had to set to get this working

### required commands
- `pip3 install -r requirements.txt` (you might need to add `--break-system-packages` on some distros)

### setting up
- `i2cdetect -y 1` for checking i2c address of the mpu5060 gyroscope

### troubleshooting commands
- error:`ImportError: cannot import name 'ImageTk' from 'PIL'` run:`sudo apt-get install python3-pil python3-pil.imagetk`