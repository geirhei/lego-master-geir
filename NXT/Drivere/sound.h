#ifndef SOUND_H_
#define SOUND_H_

void sound_init(void);
void sound_interrupt_enable(unsigned long typ);
void sound_interrupt_disable(void);
void sound_enable(void);
void sound_disable(void);

void sound_freq(unsigned long freq, unsigned long ms);
void sound_freq_vol(unsigned long freq, unsigned long ms, int vol);
void sound_play_sample(unsigned char *data, unsigned long length, unsigned long freq, int vol);
void sound_play_stream_setup(unsigned long freq, int vol);
void sound_play_stream(unsigned char *data, unsigned long length, unsigned char (*callback)());
void sound_play_stream_next(unsigned char *data, unsigned long length);
unsigned char sound_get_play_status(void);
void sound_set_volume(int vol);
int sound_get_volume(void);
int sound_get_time(void);

#define MAXVOL 100

#endif /*SOUND_H_*/
