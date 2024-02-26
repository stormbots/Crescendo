// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.MathUtil;
import java.util.Objects;

/**
 * Represents CustomColors.
 *
 * <p>Limited to 12 bits of precision.
 */
@SuppressWarnings("MemberName")
public class CustomColor {
  /** Red component (0-255). */
  public final double red;

  /** Green component (0-255). */
  public final double green;

  /** Blue component (0-255). */
  public final double blue;

  private String m_name;

  /** Constructs a default CustomColor (black). */
  public CustomColor() {
    red = 0.0;
    green = 0.0;
    blue = 0.0;
  }

  /**
   * Constructs a CustomColor from doubles.
   *
   * @param red Red value (0-255)
   * @param green Green value (0-255)
   * @param blue Blue value (0-255)
   */
  public CustomColor(double red, double green, double blue) {
    this.red = roundAndClamp(red);
    this.green = roundAndClamp(green);
    this.blue = roundAndClamp(blue);
    this.m_name = null;
  }

  /**
   * Constructs a CustomColor from ints.
   *
   * @param red Red value (0-255)
   * @param green Green value (0-255)
   * @param blue Blue value (0-255)
   */
//   public CustomColor(int red, int green, int blue) {
//     this(red / 255.0, green / 255.0, blue / 255.0);
//   }

  /**
   * Constructs a CustomColor from a CustomColor8Bit.
   *
   * @param color The CustomColor
   */
  public CustomColor(Color8Bit color) {
    this(color.red / 255.0, color.green / 255.0, color.blue / 255.0);
  }

  /**
   * Constructs a CustomColor from doubles.
   *
   * @param red Red value (0-255)
   * @param green Green value (0-255)
   * @param blue Blue value (0-255)
   */
  public CustomColor(double red, double green, double blue, String name) {
    this.red = roundAndClamp(red);
    this.green = roundAndClamp(green);
    this.blue = roundAndClamp(blue);
    this.m_name = name;
  }

  /**
   * Constructs a CustomColor from a hex string.
   *
   * @param hexString a string of the format <code>#RRGGBB</code>
   * @throws IllegalArgumentException if the hex string is invalid.
   */
  public CustomColor(String hexString) {
    if (hexString.length() != 7 || !hexString.startsWith("#")) {
      throw new IllegalArgumentException("Invalid hex string \"" + hexString + "\"");
    }

    this.red = Integer.valueOf(hexString.substring(1, 3), 16) / 255.0;
    this.green = Integer.valueOf(hexString.substring(3, 5), 16) / 255.0;
    this.blue = Integer.valueOf(hexString.substring(5, 7), 16) / 255.0;
  }

  /**
   * Creates a CustomColor from HSV values.
   *
   * @param h The h value [0-180)
   * @param s The s value [0-255]
   * @param v The v value [0-255]
   * @return The CustomColor
   */
  public static CustomColor fromHSV(int h, int s, int v) {
    // Loosely based on
    // https://en.wiipedia.org/wii/HSL_and_HSV#HSV_to_RGB
    // The hue range is split into 60 degree regions where in each region there
    // is one rgb component at a low value (m), one at a high value (v) and one
    // that changes (X) from low to high (X+m) or high to low (v-X)

    // Difference between highest and lowest value of any rgb component
    final int chroma = (s * v) / 255;

    // Because hue is 0-180 rather than 0-360 use 30 not 60
    final int region = (h / 30) % 6;

    // Remainder converted from 0-30 to 0-255
    final int remainder = (int) Math.round((h % 30) * (255 / 30.0));

    // Value of the lowest rgb component
    final int m = v - chroma;

    // Goes from 0 to chroma as hue increases
    final int X = (chroma * remainder) >> 8;

    switch (region) {
      case 0:
        return new CustomColor(v, X + m, m);
      case 1:
        return new CustomColor(v - X, v, m);
      case 2:
        return new CustomColor(m, v, X + m);
      case 3:
        return new CustomColor(m, v - X, v);
      case 4:
        return new CustomColor(X + m, m, v);
      default:
        return new CustomColor(v, m, v - X);
    }
  }

  @Override
  public boolean equals(Object other) {
    if (this == other) {
      return true;
    }
    if (other == null || getClass() != other.getClass()) {
      return false;
    }

    CustomColor color = (CustomColor) other;
    return Double.compare(color.red, red) == 0
        && Double.compare(color.green, green) == 0
        && Double.compare(color.blue, blue) == 0;
  }

  @Override
  public int hashCode() {
    return Objects.hash(red, green, blue);
  }

  @Override
  public String toString() {
    if (m_name == null) {
      // cache hex conversion
      m_name = toHexString();
    }
    return m_name;
  }

  /**
   * Return this CustomColor represented as a hex string.
   *
   * @return a string of the format <code>#RRGGBB</code>
   */
  public String toHexString() {
    return String.format(
        "#%02X%02X%02X", (int) (red), (int) (green), (int) (blue));
  }

  private static double roundAndClamp(double value) {
    return MathUtil.clamp(Math.ceil(value * (1 << 12)) / (1 << 12), 0.0, 255.0);
  }
  //TODO: Update hex codes
  /*
   * FIRST CustomColors
   */

  /** 0x1560BD. */
  public static final CustomColor kDenim = new CustomColor(50.0, 30.0, 255.0, "kDenim");

  /** 0x0066B3. */
  public static final CustomColor kFirstBlue = new CustomColor(0.0, 50.0, 255.0, "kFirstBlue");

  /** 0xED1C24. */
  public static final CustomColor kFirstRed =
      new CustomColor(255.0, 15.0, 0.0, "kFirstRed");

  /*
   *
   */
  
  /** 0xF0F8FF. */
  public static final CustomColor kAliceBlue = new CustomColor(75.0, 100.0, 255.0, "kAliceBlue");

  /** 0x00FFFF. */
  public static final CustomColor kAqua = new CustomColor(0.0, 255.0, 100.0, "kAqua");
  
  /** 0x00FFFF. */
  public static final CustomColor kAmber = new CustomColor(255.0, 40.0, 0.0, "kAmber");

  /** 0x7FFFD4. */
  public static final CustomColor kAquamarine = new CustomColor(0.0, 255.0, 75.0, "kAquamarine");

  /** 0xF0FFFF. */
  public static final CustomColor kAzure = new CustomColor(0.0, 50.0, 255.0, "kAzure");

  /** 0x8A2BE2. */
  public static final CustomColor kBlueViolet =
      new CustomColor(138.0, 0.0, 226.0, "kBlueViolet");

  /** 0xFF7F50. */
  public static final CustomColor kCoral = new CustomColor(255.0, 25.0, 5.0, "kCoral");

  /** 0x6495ED. */
  public static final CustomColor kCornflowerBlue =
      new CustomColor(100.0, 30.0, 255.0, "kCornflowerBlue");

  /** 0xDC143C. */
  public static final CustomColor kCherry = new CustomColor(255.0, 2.0, 2.0, "kCherry");

  /** 0x00FFFF. */
  public static final CustomColor kCyan = new CustomColor(0.0, 255.0, 255.0, "kCyan");

  /** 0x00BFFF. */
  public static final CustomColor kDeepSkyBlue = new CustomColor(0.0, 25.0, 255.0, "kDeepSkyBlue");

  /** 0x1E90FF. */
  public static final CustomColor kDodgerBlue = new CustomColor(30.0, 14.0, 255.0, "kDodgerBlue");

  /** 0x228B22. */
  public static final CustomColor kForestGreen =
      new CustomColor(0.0, 255.0, 12.0, "kForestGreen");

  /** 0xFFD700. */
  public static final CustomColor kGold = new CustomColor(255.0, 125.0, 0.0, "kGold");

  /** 0xDAA520. */
  public static final CustomColor kGoldenrod =
      new CustomColor(255.0, 60.0, 0.0, "kGoldenrod");

  /** 0xADFF2F. */
  public static final CustomColor kGreenYellow = new CustomColor(255.0, 205.0, 0.0, "kGreenYellow");

  /** 0xF0FFF0. */
  public static final CustomColor kHoneydew = new CustomColor(150.0, 255.0, 25.0, "kHoneydew");

  /** 0xFF69B4. */
  public static final CustomColor kHotPink = new CustomColor(255.0, 0.0, 15.0, "kHotPink");

  /** 0x4B0082. */
  public static final CustomColor kIndigo = new CustomColor(75.0, 0.0, 150.0, "kIndigo");

  /** 0xE6E6FA. */
  public static final CustomColor kLavender = new CustomColor(150.0, 15.0, 130.0, "kLavender");

  /** 0x7CFC00. */
  public static final CustomColor kLawnGreen = new CustomColor(50.0, 252.0, 0.0, "kLawnGreen");

  //TODO: automate light and pale colors with hsv to rgb
  /** 0xADD8E6. */
  public static final CustomColor kLightBlue =
      new CustomColor(173.0, 216.0, 230.0, "kLightBlue");

  /** 0xF08080. */
  public static final CustomColor kLightCoral =
      new CustomColor(255.0, 50.0, 10.0, "kLightCoral");

  /** 0x90EE90. */
  public static final CustomColor kLightGreen =
      new CustomColor(50.0, 255.0, 25.0, "kLightGreen");

  /** 0xFFB6C1. */
  public static final CustomColor kLightPink = new CustomColor(255.0, 10.0, 50.0, "kLightPink");

  /** 0xFFA07A. */
  public static final CustomColor kLightSalmon = new CustomColor(255.0, 20.0, 10.0, "kLightSalmon");

  /** 0x20B2AA. */
  public static final CustomColor kLightSeaGreen =
      new CustomColor(50.0, 200.0, 50.0, "kLightSeaGreen");

  /** 0x87CEFA. */
  public static final CustomColor kLightSkyBlue =
      new CustomColor(135.0, 206.0, 235.0, "kLightSkyBlue");

  /** 0x32CD32. */
  public static final CustomColor kLimeGreen =
      new CustomColor(75.0, 255.0, 0.0, "kLimeGreen");

  /** 0xFF00FF. */
  public static final CustomColor kMagenta = new CustomColor(255.0, 0.0, 200.0, "kMagenta");

  /** 0x191970. */
  public static final CustomColor kMidnightBlue =
      new CustomColor(25.0, 25.0, 200.0, "kMidnightBlue");

  /** 0xF5FFFA. */
  public static final CustomColor kMintcream = new CustomColor(100.0, 255.0, 50.0, "kMintcream");

  /** 0xFFE4E1. */
  public static final CustomColor kPaleRose = new CustomColor(255.0, 50.0, 50.0, "kPaleRose");

  /** 0x808000. */
  public static final CustomColor kOlive = new CustomColor(150.0, 142.0, 5.0, "kOlive");

  /** 0x6B8E23. */
  public static final CustomColor kOliveDrab =
      new CustomColor(150.0, 142.0, 10.0, "kOliveDrab");

  /** 0xFFA500. */
  public static final CustomColor kOrange = new CustomColor(255.0, 25.0, 0.0, "kOrange");

  /** 0xFF4500. */
  public static final CustomColor kOrangeRed = new CustomColor(255.0, 10.0, 0.0, "kOrangeRed");

  /** 0xDA70D6. */
  public static final CustomColor kOrchid = new CustomColor(250.0, 0.0, 150.0, "kOrchid");
//TODO
  // /** 0xEEE8AA. */
  // public static final CustomColor kPaleGoldenrod =
  //     new CustomColor(238.0, 232.0, 170.0, "kPaleGoldenrod");

  // /** 0x98FB98. */
  // public static final CustomColor kPaleGreen =
  //     new CustomColor(152.0, 251.0, 152.0, "kPaleGreen");

  // /** 0xAFEEEE. */
  // public static final CustomColor kPaleTurquoise =
  //     new CustomColor(175.0, 238.0, 238.0, "kPaleTurquoise");

  // /** 0xDB7093. */
  // public static final CustomColor kPaleVioletRed =
  //     new CustomColor(119.0, 112.0, 147.0, "kPaleVioletRed");

  // /** 0xFFEFD5. */
  // public static final CustomColor kPapayaWhip = new CustomColor(255.0, 239.0, 213.0, "kPapayaWhip");

  // /** 0xFFDAB9. */
  // public static final CustomColor kPeachPuff = new CustomColor(255.0, 218.0, 185.0, "PeachPuff");

  // /** 0xCD853F. */
  // public static final CustomColor kPeru = new CustomColor(205.0, 133.0, 63.0, "kPeru");

  /** 0xFFC0CB. */
  public static final CustomColor kPink = new CustomColor(255.0, 0.0, 25.0, "kPink");

  /** 0xDDA0DD. */
  public static final CustomColor kPlum = new CustomColor(221.0, 160.0, 221.0, "kPlum");

  /** 0xB0E0E6. */
  public static final CustomColor kPowderBlue =
      new CustomColor(176.0, 224.0, 230.0, "kPowderBlue");

  /** 0x4169E1. */
  public static final CustomColor kRoyalBlue =
      new CustomColor(65.0, 105.0, 225.0, "kRoyalBlue");

  /** 0xFA8072. */
  public static final CustomColor kSalmon = new CustomColor(255.0, 8.0, 3.0, "kSalmon");

  /** 0x2E8B57. */
  public static final CustomColor kSeaGreen =
      new CustomColor(0.0, 200.0, 25.0, "kSeaGreen");

  /** 0xFFF5EE. */
  public static final CustomColor kSeashell = new CustomColor(255.0, 245.0, 238.0, "kSeashell");

  /** 0xA0522D. */
  public static final CustomColor kSienna = new CustomColor(160.0, 82.0, 45.0, "kSienna");

  /** 0xC0C0C0. */
  public static final CustomColor kSilver = new CustomColor(192.0, 192.0, 192.0, "kSilver");

  /** 0x87CEEB. */
  public static final CustomColor kSkyBlue = new CustomColor(50.0, 75.0, 235.0, "kSkyBlue");

  /** 0x00FF7F. */
  public static final CustomColor kSpringGreen = new CustomColor(0.0, 255.0, 127.0, "kSpringGreen");

  /** 0x4682B4. */
  public static final CustomColor kSteelBlue =
      new CustomColor(70.0, 70.0, 200.0, "kSteelBlue");

  /** 0x008080. */
  public static final CustomColor kTeal = new CustomColor(0.0, 128.0, 128.0, "kTeal");

  /** 0xD8BFD8. */
  public static final CustomColor kThistle = new CustomColor(216.0, 191.0, 216.0, "kThistle");

  /** 0xFF6347. */
  public static final CustomColor kTomato = new CustomColor(255.0, 99.0, 71.0, "kTomato");

  /** 0x40E0D0. */
  public static final CustomColor kTurquoise =
      new CustomColor(64.0, 224.0, 208.0, "kTurquoise");

  /** 0xEE82EE. */
  public static final CustomColor kViolet = new CustomColor(238.0, 130.0, 238.0, "kViolet");

  /** 0xF5DEB3. */
  public static final CustomColor kWheat = new CustomColor(245.0, 222.0, 179.0, "kWheat");

  /** 0x9ACD32. */
  public static final CustomColor kYellowGreen =
      new CustomColor(175.0, 255.0, 0.0, "kYellowGreen");
}


