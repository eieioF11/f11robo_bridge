# f11robo_bridge

## Install
### serial setup
```bash
cd f11robo_bridge/setup
chmod 777 *
sudo sh udev_setup.sh
```
### ydlidar

```bash
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK/build
cmake ..
make
sudo make install
```