/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
Description: Helper functions implementation
License: Revised BSD License, see LICENSE.TXT file include in the project
Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __UTILITIES_H__
#define __UTILITIES_H__

/*!
 * \brief Returns the minimum value betwen a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval minValue Minimum value
 */
#define MIN( a, b ) ( ( ( a ) < ( b ) ) ? ( a ) : ( b ) )

/*!
 * \brief Returns the maximum value betwen a and b
 *
 * \param [IN] a 1st value
 * \param [IN] b 2nd value
 * \retval maxValue Maximum value
 */
#define MAX( a, b ) ( ( ( a ) > ( b ) ) ? ( a ) : ( b ) )

/*!
 * \brief Returns 2 raised to the power of n
 *
 * \param [IN] n power value
 * \retval result of raising 2 to the power n
 */
#define POW2( n ) ( 1 << n )


/*!
 * \brief Converts a nibble to an hexadecimal character
 * 
 * \param [IN] a   Nibble to be converted
 * \retval hexChar Converted hexadecimal character
 */
int8_t Nibble2HexChar( uint8_t a );

#endif // __UTILITIES_H__

