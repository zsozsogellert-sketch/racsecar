# Robot irányítása
ROS 2 python package ![Static Badge](https://img.shields.io/badge/ROS_2-Humble-34aec5)[![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/humble/)
## Telepítés

Munkakönyvtár: `~/ros2_ws/`.

### Klónozás:

```bash
cd ~/ros2_ws/src
```
```bash
git clone https://github.com/zsozsogellert-sketch/racsecar
```
### ros2 könyvtár telepítés

```bash
cd ~/ros2_ws
```
```bash
colcon build --packages-select racsecar --symlink-install
```

### könytár futtatása

```bash
source ~/ros2_ws/install/setup.bash
```
```bash
ros2 run racsecar megoldas.py
```