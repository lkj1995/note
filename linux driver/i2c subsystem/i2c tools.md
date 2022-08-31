调用关系

```
[ DTS adapter| client ]  ---> [i2c-imx driver]
										|
										v
   [i2c bus]     <---  [i2c-core adapter dev | client dev]
    /      \     <---  [i2c-dev adapter dev(parent) -> i2c_dev dev(child)]       
  /	        \
 		[adapter dev]     
 		[client dev]
 		[i2c_dev dev]
 
```





