### 设备号

- 类型：dev_t

- 分为主设备号，次设备号

- 使用操作宏

  ```C
  #define MINORBITS 20
  #define MINORMASK ((1U << MINORBITS) - 1)
  
  #define MAJOR(dev) ((unsigned int) ((dev) >> MINORBITS))
  #define MINOR(dev) ((unsigned int) ((dev) & MINORMASK))
  #define MKDEV(ma,mi) (((ma) << MINORBITS) | (mi)) 
  ```

  

