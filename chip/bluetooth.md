### 可变形参使用方法
```c
#include "stdarg.h"

char buff[80];
void test（int* aaa,...）{
  int sta;  
  va_list bbb;         //创建一个变量bbb
  va_start(bbb,aaa);   //开始
  sta = vsprintf(buff,aaa,bbb);
  va_arg(bbb,int);  
  va_end(bbb);  //结束
  
}

int main(void){
  int i = 5;
  float f = 7.00;
  char str[] = "strstr"; 
  test("%d %f %str",i,f,str);
  printf(" %s ",buff);
}
```

