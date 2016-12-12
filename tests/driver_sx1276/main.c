#include <stdio.h>
#include "sx1276.h"

int main(void)
{
    puts("Hello!\n");
    radio_init();
    puts("Passed radio_init!\n");
    set_lmic_frame("hola\n", 4); 
    starttx();
    puts("Compiled!\n");
}
