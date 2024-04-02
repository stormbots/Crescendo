package com.revrobotics;

import com.revrobotics.jni.CANSparkMaxJNI;

public class SparkFlexFixes {
  /**
   * Workaround for issue where SPARK Flex API doesn't allow you to set this.
   *
   * @param spark CANSparkFlex to set
   * @param depth Encoder Average depth in the range [1, 64] default is 64
   * @return REVLibError::kOk if all is good
   */
  public static REVLibError setFlexEncoderAverageDepth(CANSparkFlex spark, int depth) {
    if (depth < 1 || depth > 64) {
      throw new IllegalArgumentException(
          "Quadrature average depth must be in the range of [1, 64]");
    }
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetAverageDepth(spark.sparkMaxHandle, depth));
  }

  public static int getFlexEncoderAverageDepth(CANSparkFlex spark) {
    return CANSparkMaxJNI.c_SparkMax_GetAverageDepth(spark.sparkMaxHandle);
  }

  /**
   * Workaround for issue where SPARK Flex API doesn't allow you to set this.
   *
   * @param spark CANSparkFlex to set
   * @param period_ms the sample delta period is milliseconds (derivative delta) default is 100
   * @return REVLibError::kOk if all is good
   */
  public static REVLibError setFlexEncoderSampleDelta(CANSparkFlex spark, int period_ms) {
    if (period_ms < 1 || period_ms > 100) {
      throw new IllegalArgumentException(
          "Quadrature measurement period must be in the range of [1, 100]");
    }
    return REVLibError.fromInt(
        CANSparkMaxJNI.c_SparkMax_SetMeasurementPeriod(spark.sparkMaxHandle, period_ms));
  }

  public static int getFlexEncoderSampleDelta(CANSparkFlex spark) {
    return CANSparkMaxJNI.c_SparkMax_GetMeasurementPeriod(spark.sparkMaxHandle);
  }
}
