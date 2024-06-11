# (all) the things I had to set to get this working

### required commands, etc
- `pip3 install -r requirements.txt` (you might need to add `--break-system-packages` on some distros)
- add `dtoverlay=pwm` to `boot/firmware/config.txt`

### setting up
- `i2cdetect -y 1` for checking i2c address of the mpu5060 gyroscope

### troubleshooting commands
- error:`ImportError: cannot import name 'ImageTk' from 'PIL'` run:`sudo apt-get install python3-pil python3-pil.imagetk`

- if running vith video writing, use `ffmpeg -framerate 4 -i %d.jpg -c:v libx264 -pix_fmt yuv420p output.ts` to combine frames into video