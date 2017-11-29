#include <stdlib.h>
#include <stdio.h>
#include <malloc.h>

int main (){

        int a = 10;
        int b = 0x104;
        unsigned char   *p ;
        p = (unsigned char *) malloc(b);
        printf ("p = %p\n", p);
        printf ("hello world\n");
}
