#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__


// Макрос для создание битовой маски
#define _BV(x) (1<<(x))

#undef SERIAL_DEBUG
#ifdef SERIAL_DEBUG
#define IF_SERIAL_DEBUG(x) ({x;})
#else
#define IF_SERIAL_DEBUG(x)
#endif


#endif // __RF24_CONFIG_H__
// vim:ai:cin:sts=2 sw=2 ft=cpp