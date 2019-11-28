# esp32-bno055

**Danger! Work in progress.**

```bash
cd examples/hello-bno

$IDF_PATH/tools/idf.py app
$IDF_PATH/tools/idf.py flash
$IDF_PATH/tools/idf.py monitor

cd examples/quat-bno

$IDF_PATH/tools/idf.py app
$IDF_PATH/tools/idf.py flash
$IDF_PATH/tools/idf.py monitor
```

The monitor should be printing a stream of quaternions 

To run the animation:<br/>
Quit the monitor: Ctrl-]  (The esp32 app will still be running)

```bash
cd animation
python3 BNO_tty_Scene.py
```

If nothing happens, press the reset button at the esp32 board<br/>
python3 dependencies: vtk, math, numpy, serial, time


```

