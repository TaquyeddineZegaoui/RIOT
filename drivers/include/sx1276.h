#ifndef SX1276_H_
#define SX1276_H_
#ifdef __cplusplus
extern "C"
{
#endif

void radio_init (void);
void set_lmic_frame(char *buf, size_t size);
void starttx (void);
    

#ifdef __cplusplus
}
#endif
#endif
