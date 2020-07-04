/******************************INFO**********************************/
//AUTOR:JULIO CESAR MATIAS
//DESENVOLVIDO OFICIALMENTE PARA A CONTROLADORA DE VOO JCFLIGHT
//KALMAN FILTER PARA GYROSCOPIO DA MPU6050
//DATA:JULHO DE 2019
/********************************************************************/

float Q_GyroProcessNoise   = 0.01;
float Q_GyroNoise          = 0.0003;
float R_GyroNoise          = 0.01;

float GyroRoll_Bias        = 0;
float GyroPitch_Bias       = 0;
float GyroYaw_Bias         = 0;

float PredictionRoll_00    = 0,
      PredictionRoll_01    = 0,
      PredictionRoll_10    = 0,
      PredictionRoll_11    = 0;

float PredictionPitch_00   = 0,
      PredictionPitch_01   = 0,
      PredictionPitch_10   = 0,
      PredictionPitch_11   = 0;

float PredictionYaw_00     = 0,
      PredictionYaw_01     = 0,
      PredictionYaw_10     = 0,
      PredictionYaw_11     = 0;

int16_t KF2GyroRollResult  = 0;
int16_t KF2GyroPitchResult = 0;
int16_t KF2GyroYawResult   = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  delay(50);
  //GYRO ROLL
  Serial.print(KF2GyroRoll(analogRead(0), analogRead(0) / 131));
  Serial.print(" ");
  Serial.print(analogRead(0));
  //GYRO PITCH
  Serial.print(" ");
  Serial.print(KF2GyroPitch(analogRead(1), analogRead(1) / 131));
  Serial.print(" ");
  Serial.print(analogRead(1));
  //GYRO YAW
  Serial.print(" ");
  Serial.print(KF2GyroYaw(analogRead(2), analogRead(2) / 131));
  Serial.print(" ");
  Serial.print(analogRead(2));
  Serial.println();
}

int16_t KF2GyroRoll(int16_t GyroRoll, float GyroRate) {
  float RollDifference, EstimateError;
  float KF2_0, KF2_1;

  KF2GyroRollResult +=  0.02f * (GyroRate - GyroRoll_Bias);
  PredictionRoll_00 += -0.02f * (PredictionRoll_10 + PredictionRoll_01) + Q_GyroProcessNoise * 0.02f;
  PredictionRoll_01 += -0.02f * PredictionRoll_11;
  PredictionRoll_10 += -0.02f * PredictionRoll_11;
  PredictionRoll_11 += +Q_GyroNoise * 0.02f;
  RollDifference     = GyroRoll - KF2GyroRollResult;
  EstimateError      = PredictionRoll_00 + R_GyroNoise;
  KF2_0              = PredictionRoll_00 / EstimateError;
  KF2_1              = PredictionRoll_10 / EstimateError;
  KF2GyroRollResult += KF2_0 * RollDifference;
  GyroRoll_Bias     += KF2_1 * RollDifference;
  PredictionRoll_00 -= KF2_0 * PredictionRoll_00;
  PredictionRoll_01 -= KF2_0 * PredictionRoll_01;
  PredictionRoll_10 -= KF2_1 * PredictionRoll_00;
  PredictionRoll_11 -= KF2_1 * PredictionRoll_01;
  return KF2GyroRollResult;
}


int16_t KF2GyroPitch(int16_t GyroPitch, float GyroRate) {
  float PitchDifference, EstimateError;
  float KF2_0, KF2_1;

  KF2GyroPitchResult +=  0.02f * (GyroRate - GyroPitch_Bias);
  PredictionPitch_00 += -0.02f * (PredictionPitch_10 + PredictionPitch_01) + Q_GyroProcessNoise * 0.02f;
  PredictionPitch_01 += -0.02f * PredictionPitch_11;
  PredictionPitch_10 += -0.02f * PredictionPitch_11;
  PredictionPitch_11 += +Q_GyroNoise * 0.02f;
  PitchDifference     = GyroPitch - KF2GyroPitchResult;
  EstimateError       = PredictionPitch_00 + R_GyroNoise;
  KF2_0               = PredictionPitch_00 / EstimateError;
  KF2_1               = PredictionPitch_10 / EstimateError;
  KF2GyroPitchResult += KF2_0 * PitchDifference;
  GyroYaw_Bias       += KF2_1 * PitchDifference;
  PredictionPitch_00 -= KF2_0 * PredictionPitch_00;
  PredictionPitch_01 -= KF2_0 * PredictionPitch_01;
  PredictionPitch_10 -= KF2_1 * PredictionPitch_00;
  PredictionPitch_11 -= KF2_1 * PredictionPitch_01;
  return KF2GyroPitchResult;
}

int16_t KF2GyroYaw(int16_t GyroYaw, float GyroRate) {
  float YawDifference, EstimateError;
  float KF2_0, KF2_1;

  KF2GyroYawResult +=  0.02f * (GyroRate - GyroYaw_Bias);
  PredictionYaw_00 += -0.02f * (PredictionYaw_10 + PredictionYaw_01) + Q_GyroProcessNoise * 0.02f;
  PredictionYaw_01 += -0.02f * PredictionYaw_11;
  PredictionYaw_10 += -0.02f * PredictionYaw_11;
  PredictionYaw_11 += +Q_GyroNoise * 0.02f;
  YawDifference     = GyroYaw - KF2GyroYawResult;
  EstimateError     = PredictionYaw_00 + R_GyroNoise;
  KF2_0             = PredictionYaw_00 / EstimateError;
  KF2_1             = PredictionYaw_10 / EstimateError;
  KF2GyroYawResult += KF2_0 * YawDifference;
  GyroYaw_Bias     += KF2_1 * YawDifference;
  PredictionYaw_00 -= KF2_0 * PredictionYaw_00;
  PredictionYaw_01 -= KF2_0 * PredictionYaw_01;
  PredictionYaw_10 -= KF2_1 * PredictionYaw_00;
  PredictionYaw_11 -= KF2_1 * PredictionYaw_01;
  return KF2GyroYawResult;
}
