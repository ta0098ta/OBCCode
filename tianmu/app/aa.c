
#include <stdio.h>
#include <stdlib.h>

int main () {
        int num0 = 100;
        int num1 = 10;

        int num = num0<<8;
        int num3 = num+num1;
        printf ("num = %d\n,    num0<<8 = %d\n,   num0<<8 + num1  = %d    \n", num, num0 << 8 , num3);
}
