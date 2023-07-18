#ifndef __ENCODER_H
#define __ENCODER_H

void UpdateRpm(void);
void ResetEncoderVariables(void);
int FilterRpm(char motorIndex);
void ResetSecondaryEncoderVariables(void);
#endif /* __ENCODER_H */
