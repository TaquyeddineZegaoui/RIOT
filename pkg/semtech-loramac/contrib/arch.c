void memset1( uint8_t *dst, uint8_t value, uint16_t size )
{
    while( size-- )
    {
        *dst++ = value;
    }
}
