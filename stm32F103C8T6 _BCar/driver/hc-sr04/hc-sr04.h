#ifndef HC_SR04_H
#define HC_SR04_H

void HC_SR04_Init(void);
void line_gpio_init(void);

float hc_sr04_distance(void);
uint8_t read_line_sensor(void);

#endif /* HC_SR04_H */
