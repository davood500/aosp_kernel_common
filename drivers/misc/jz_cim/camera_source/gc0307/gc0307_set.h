#ifndef gc0307_SET_H
#define gc0307_SET_H

#include<linux/i2c.h>

void gc0307_init_setting(struct i2c_client *client);
void preview_set(struct i2c_client *client);
void capture_set(struct i2c_client *client);
void size_switch(struct i2c_client *client,int width,int height,int setmode);

void gc0307_read_shutter(struct i2c_client *client);

#endif

