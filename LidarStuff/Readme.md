### Lidar Support Code: 
The following files contain different components of the Lidar system. Below each file are instructions on how to run it. 

**init/init_i2c.py** - Initializes sensors in order to change their addresses. `runme0`, `runme1`, `runme2`, and `runme3` initialize each sensor. 
```sh
$ python init_i2c.py
```

**runme.c** (runme) - Initializes a handle on a *VL6180* sensor and changes its address. Compile it using gcc, then run. 
```sh
$ gcc -o runme runme.c
$ ./runme
```

**readfrom4.c** (read4) - Tests 4 Lidar objects on 0x20, 0x21, 0x22, 0x23. Compile it using gcc, then run. 
```sh
$ gcc -o read4 readfrom4.c
$ ./read4
```

**vl6180_pi.c + vl6180_pi.h** - Library used to connect to *VL6180* sensors. 
```C
#include "vl6180_pi.c"
```
