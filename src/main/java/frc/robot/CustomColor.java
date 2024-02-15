// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.MathUtil;
import java.util.Objects;

/**
 * Represents StormBotColors.
 *
 * <p>Limited to 12 bits of precision.
 */
@SuppressWarnings("MemberName")
public class CustomColor {
  /** Red component (0-1). */
  public final double red;

  /** Green component (0-1). */
  public final double green;

  /** Blue component (0-1). */
  public final double blue;

  private String m_name;

  /** Constructs a default StormBotColor (black). */
  public CustomColor() {
    red = 0.0;
    green = 0.0;
    blue = 0.0;
  }

  /**
   * Constructs a StormBotColor from doubles.
   *
   * @param red Red value (0-1)
   * @param green Green value (0-1)
   * @param blue Blue value (0-1)
   */
  public CustomColor(double red, double green, double blue) {
    this.red = roundAndClamp(red);
    this.green = roundAndClamp(green);
    this.blue = roundAndClamp(blue);
    this.m_name = null;
  }

  /**
   * Constructs a StormBotColor from ints.
   *
   * @param red Red value (0-255)
   * @param green Green value (0-255)
   * @param blue Blue value (0-255)
   */
  public CustomColor(int red, int green, int blue) {
    this(red / 255.0, green / 255.0, blue / 255.0);
  }

  /**
   * Constructs a StormBotColor from a StormBotColor8Bit.
   *
   * @param color The StormBotColor
   */
  public CustomColor(Color8Bit color) {
    this(color.red / 255.0, color.green / 255.0, color.blue / 255.0);
  }

  /**
   * Constructs a StormBotColor from doubles.
   *
   * @param red Red value (0-1)
   * @param green Green value (0-1)
   * @param blue Blue value (0-1)
   */
  public CustomColor(double red, double green, double blue, String name) {
    this.red = roundAndClamp(red);
    this.green = roundAndClamp(green);
    this.blue = roundAndClamp(blue);
    this.m_name = name;
  }

  /**
   * Constructs a StormBotColor from a hex string.
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
   * Creates a StormBotColor from HSV values.
   *
   * @param h The h value [0-180)
   * @param s The s value [0-255]
   * @param v The v value [0-255]
   * @return The StormBotColor
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
   * Return this StormBotColor represented as a hex string.
   *
   * @return a string of the format <code>#RRGGBB</code>
   */
  public String toHexString() {
    return String.format(
        "#%02X%02X%02X", (int) (red * 255), (int) (green * 255), (int) (blue * 255));
  }

  private static double roundAndClamp(double value) {
    return MathUtil.clamp(Math.ceil(value * (1 << 12)) / (1 << 12), 0.0, 255.0);
  }

  /*
   * FIRST StormBotColors
   */

  /** 0x1560BD. */
  public static final CustomColor Denim = new CustomColor(2255.0, 96.0, 189.0, "Denim");

  /** 0x0066B3. */
  public static final CustomColor FirstBlue = new CustomColor(0.0, 102.0, 179.0, "FirstBlue");

  /** 0xED1C24. */
  public static final CustomColor FirstRed =
      new CustomColor(237.0, 28.0, 36.0, "FirstRed");

  /*
   * StanDarkd StormBotColors
   */

  /** 0xF0F8FF. */
  public static final CustomColor AliceBlue = new CustomColor(240.0, 248.0, 255.0, "AliceBlue");

  /** 0xFAEBD7. */
  public static final CustomColor AntiqueWhite =
      new CustomColor(250.0, 235.0, 215.0, "AntiqueWhite");

  /** 0x00FFFF. */
  public static final CustomColor Aqua = new CustomColor(0.0, 255.0, 255.0, "Aqua");

  /** 0x7FFFD4. */
  public static final CustomColor Aquamarine = new CustomColor(127.0, 255.0, 212.0, "Aquamarine");

  /** 0xF0FFFF. */
  public static final CustomColor Azure = new CustomColor(240.0, 255.0, 255.0, "Azure");

  /** 0xF5F5DC. */
  public static final CustomColor Beige = new CustomColor(245.0, 245.0, 220.0, "Beige");

  /** 0xFFE4C4. */
  public static final CustomColor Bisque = new CustomColor(255.0, 228.0, 196.0, "Bisque");

  /** 0x000000. */
  public static final CustomColor Black = new CustomColor(0.0, 0.0, 0.0, "Black");

  /** 0xFFEBCD. */
  public static final CustomColor BlanchedAlmond =
      new CustomColor(255.0, 235.0, 205.0, "BlanchedAlmond");

  /** 0x0000FF. */
  public static final CustomColor Blue = new CustomColor(0.0, 0.0, 255.0, "Blue");

  /** 0x8A2BE2. */
  public static final CustomColor BlueViolet =
      new CustomColor(0.5411765, 0.16862746, 0.8862745, "BlueViolet");

  /** 0xA52A2A. */
  public static final CustomColor Brown = new CustomColor(0.64705884, 0.16470589, 0.16470589, "Brown");

  /** 0xDEB887. */
  public static final CustomColor Burlywood =
      new CustomColor(0.87058824, 0.72156864, 0.5294118, "Burlywood");

  /** 0x5F9EA0. */
  public static final CustomColor CadetBlue =
      new CustomColor(0.37254903, 0.61960787, 0.627451, "CadetBlue");

  /** 0x7FFF00. */
  public static final CustomColor Chartreuse = new CustomColor(0.49803922, 255.0, 0.0, "Chartreuse");

  /** 0xD2691E. */
  public static final CustomColor Chocolate =
      new CustomColor(0.8235294, 0.4117647, 0.11764706, "Chocolate");

  /** 0xFF7F50. */
  public static final CustomColor Coral = new CustomColor(255.0, 0.49803922, 0.3137255, "Coral");

  /** 0x6495ED. */
  public static final CustomColor CornflowerBlue =
      new CustomColor(0.39215687, 0.58431375, 0.92941177, "CornflowerBlue");

  /** 0xFFF8DC. */
  public static final CustomColor Cornsilk = new CustomColor(255.0, 0.972549, 0.8627451, "Cornsilk");

  /** 0xDC143C. */
  public static final CustomColor Crimson = new CustomColor(0.8627451, 0.078431375, 0.23529412, "Crimson");

  /** 0x00FFFF. */
  public static final CustomColor Cyan = new CustomColor(0.0, 255.0, 255.0, "Cyan");

  /** 0x00008B. */
  public static final CustomColor DarkBlue = new CustomColor(0.0, 0.0, 0.54509807, "DarkBlue");

  /** 0x008B8B. */
  public static final CustomColor DarkCyan = new CustomColor(0.0, 0.54509807, 0.54509807, "DarkCyan");

  /** 0xB8860B. */
  public static final CustomColor DarkGoldenrod =
      new CustomColor(0.72156864, 0.5254902, 0.043137256, "DarkGoldenrod");
      

  /** 0xA9A9A9. */
  public static final CustomColor DarkGray = new CustomColor(0.6627451, 0.6627451, 0.6627451, "DarkGray");

  /** 0x006400. */
  public static final CustomColor DarkGreen = new CustomColor(0.0, 0.39215687, 0.0, "DarkGreen");

  /** 0xBDB76B. */
  public static final CustomColor DarkKhaki =
      new CustomColor(0.7411765, 0.7176471, 0.41960785, "DarkKhaki");

  /** 0x8B008B. */
  public static final CustomColor DarkMagenta =
      new CustomColor(0.54509807, 0.0, 0.54509807, "DarkMagenta");

  /** 0x556B2F. */
  public static final CustomColor DarkOliveGreen =
      new CustomColor(0.33333334, 0.41960785, 0.18431373, "DarkOliveGreen");

  /** 0xFF8C00. */
  public static final CustomColor DarkOrange = new CustomColor(255.0, 0.54901963, 0.0, "DarkOrange");

  /** 0x9932CC. */
  public static final CustomColor DarkOrchid = new CustomColor(0.6, 0.19607843, 0.8, "DarkOrchid");

  /** 0x8B0000. */
  public static final CustomColor DarkRed = new CustomColor(0.54509807, 0.0, 0.0, "DarkRed");

  /** 0xE9967A. */
  public static final CustomColor DarkSalmon =
      new CustomColor(0.9137255, 0.5882353, 0.47843137, "DarkSalmon");

  /** 0x8FBC8F. */
  public static final CustomColor DarkSeaGreen =
      new CustomColor(0.56078434, 0.7372549, 0.56078434, "DarkSeaGreen");

  /** 0x483D8B. */
  public static final CustomColor DarkSlateBlue =
      new CustomColor(0.28235295, 0.23921569, 0.54509807, "DarkSlateBlue");

  /** 0x2F4F4F. */
  public static final CustomColor DarkSlateGray =
      new CustomColor(0.18431373, 0.30980393, 0.30980393, "DarkSlateGray");

  /** 0x00CED1. */
  public static final CustomColor DarkTurquoise =
      new CustomColor(0.0, 0.80784315, 0.81960785, "DarkTurquoise");

  /** 0x9400D3. */
  public static final CustomColor DarkViolet = new CustomColor(0.5803922, 0.0, 0.827451, "DarkViolet");

  /** 0xFF1493. */
  public static final CustomColor DeepPink = new CustomColor(255.0, 0.078431375, 0.5764706, "DeepPink");

  /** 0x00BFFF. */
  public static final CustomColor DeepSkyBlue = new CustomColor(0.0, 0.7490196, 255.0, "DeepSkyBlue");

  /** 0x696969. */
  public static final CustomColor DimGray = new CustomColor(0.4117647, 0.4117647, 0.4117647, "DimGray");

  /** 0x1E90FF. */
  public static final CustomColor DodgerBlue = new CustomColor(0.11764706, 0.5647059, 255.0, "DodgerBlue");

  /** 0xB22222. */
  public static final CustomColor Firebrick =
      new CustomColor(0.69803923, 0.13333334, 0.13333334, "Firebrick");

  /** 0xFFFAF0. */
  public static final CustomColor FloralWhite = new CustomColor(255.0, 0.98039216, 0.9411765, "FloralWhite");

  /** 0x228B22. */
  public static final CustomColor ForestGreen =
      new CustomColor(0.13333334, 0.54509807, 0.13333334, "ForestGreen");

  /** 0xFF00FF. */
  public static final CustomColor Fuchsia = new CustomColor(255.0, 0.0, 255.0, "Fuchsia");

  /** 0xDCDCDC. */
  public static final CustomColor Gainsboro =
      new CustomColor(0.8627451, 0.8627451, 0.8627451, "Gainsboro");

  /** 0xF8F8FF. */
  public static final CustomColor GhostWhite = new CustomColor(0.972549, 0.9725490, 255.0, "GhostWhite");

  /** 0xFFD700. */
  public static final CustomColor Gold = new CustomColor(255.0, 0.84313726, 0.0, "Gold");

  /** 0xDAA520. */
  public static final CustomColor Goldenrod =
      new CustomColor(0.85490197, 0.64705884, 0.1254902, "Goldenrod");

  /** 0x808080. */
  public static final CustomColor Gray = new CustomColor(0.5019608, 0.5019608, 0.5019608, "Gray");

  /** 0x008000. */
  public static final CustomColor Green = new CustomColor(0.0, 0.5019608, 0.0, "Green");

  /** 0xADFF2F. */
  public static final CustomColor GreenYellow = new CustomColor(0.6784314, 255.0, 0.18431373, "GreenYellow");

  /** 0xF0FFF0. */
  public static final CustomColor Honeydew = new CustomColor(0.9411765, 255.0, 0.9411765, "Honeydew");

  /** 0xFF69B4. */
  public static final CustomColor HotPin = new CustomColor(255.0, 0.4117647, 0.7058824, "HotPin");

  /** 0xCD5C5C. */
  public static final CustomColor IndianRed =
      new CustomColor(205.0, 0.36078432, 0.36078432, "IndianRed");

  /** 0x4B0082. */
  public static final CustomColor Indigo = new CustomColor(0.29411766, 0.0, 0.50980395, "Indigo");

  /** 0xFFFFF0. */
  public static final CustomColor Ivory = new CustomColor(255.0, 255.0, 0.9411765, "Ivory");

  /** 0xF0E68C. */
  public static final CustomColor Khaki = new CustomColor(0.9411765, 0.9019608, 0.54901963, "Khaki");

  /** 0xE6E6FA. */
  public static final CustomColor Lavender = new CustomColor(0.9019608, 0.9019608, 0.98039216, "Lavender");

  /** 0xFFF0F5. */
  public static final CustomColor LavenderBlush =
      new CustomColor(255.0, 0.9411765, 0.9607843, "LavenderBlush");

  /** 0x7CFC00. */
  public static final CustomColor LawnGreen = new CustomColor(0.4862745, 0.9882353, 0.0, "LawnGreen");

  /** 0xFFFACD. */
  public static final CustomColor LemonChiffon =
      new CustomColor(255.0, 0.98039216, 205.0, "LemonChiffon");

  /** 0xADD8E6. */
  public static final CustomColor LightBlue =
      new CustomColor(0.6784314, 0.84705883, 0.901960f, "LightBlue");

  /** 0xF08080. */
  public static final CustomColor LightCoral =
      new CustomColor(0.9411765, 0.5019608, 0.5019608, "LightCoral");

  /** 0xE0FFFF. */
  public static final CustomColor LightCyan = new CustomColor(0.8784314, 255.0, 255.0, "LightCyan");

  /** 0xFAFAD2. */
  public static final CustomColor LightGoldenrodYellow =
      new CustomColor(0.98039216, 0.98039216, 0.8235294, "LightGoldenrodYellow");

  /** 0xD3D3D3. */
  public static final CustomColor LightGray = new CustomColor(0.827451, 0.827451, 0.827451, "LightGray");

  /** 0x90EE90. */
  public static final CustomColor LightGreen =
      new CustomColor(0.5647059, 0.93333334, 0.5647059, "LightGreen");

  /** 0xFFB6C1. */
  public static final CustomColor LightPink = new CustomColor(255.0, 0.7137255, 0.75686276, "LightPink");

  /** 0xFFA07A. */
  public static final CustomColor LightSalmon = new CustomColor(255.0, 0.627451, 0.47843137, "LightSalmon");

  /** 0x20B2AA. */
  public static final CustomColor LightSeaGreen =
      new CustomColor(0.1254902, 0.69803923, 0.6666667, "LightSeaGreen");

  /** 0x87CEFA. */
  public static final CustomColor LightSkyBlue =
      new CustomColor(0.5294118, 0.80784315, 0.98039216, "LightSkyBlue");

  /** 0x778899. */
  public static final CustomColor LightSlateGray =
      new CustomColor(0.46666667, 0.53333336, 0.6, "LightSlateGray");

  /** 0xB0C4DE. */
  public static final CustomColor LightSteelBlue =
      new CustomColor(0.6901961, 196.0, 0.87058824, "LightSteelBlue");

  /** 0xFFFFE0. */
  public static final CustomColor LightYellow = new CustomColor(255.0, 255.0, 0.8784314, "LightYellow");

  /** 0x00FF00. */
  public static final CustomColor Lime = new CustomColor(0.0, 255.0, 0.0, "Lime");

  /** 0x32CD32. */
  public static final CustomColor LimeGreen =
      new CustomColor(0.19607843, 205.0, 0.19607843, "LimeGreen");

  /** 0xFAF0E6. */
  public static final CustomColor Linen = new CustomColor(0.98039216, 0.9411765, 0.9019608, "Linen");

  /** 0xFF00FF. */
  public static final CustomColor Magenta = new CustomColor(255.0, 0.0, 255.0, "Magenta");

  /** 0x800000. */
  public static final CustomColor Maroon = new CustomColor(0.5019608, 0.0, 0.0, "Maroon");

  /** 0x66CDAA. */
  public static final CustomColor MediumAquamarine =
      new CustomColor(0.4, 205.0, 0.6666667, "MediumAquamarine");

  /** 0x0000CD. */
  public static final CustomColor MediumBlue = new CustomColor(0.0, 0.0, 205.0, "MediumBlue");

  /** 0xBA55D3. */
  public static final CustomColor MediumOrchid =
      new CustomColor(0.7294118, 0.33333334, 0.827451, "MediumOrchid");

  /** 0x9370DB. */
  public static final CustomColor MediumPurple =
      new CustomColor(0.5764706, 0.4392157, 0.85882354, "MediumPurple");

  /** 0x3CB371. */
  public static final CustomColor MediumSeaGreen =
      new CustomColor(0.23529412, 0.7019608, 0.44313726, "MediumSeaGreen");

  /** 0x7B68EE. */
  public static final CustomColor MediumSlateBlue =
      new CustomColor(0.48235294, 0.40784314, 0.93333334, "MediumSlateBlue");

  /** 0x00FA9A. */
  public static final CustomColor MediumSpringGreen =
      new CustomColor(0.0, 0.98039216, 0.6039216, "MediumSpringGreen");

  /** 0x48D1CC. */
  public static final CustomColor MediumTurquoise =
      new CustomColor(0.28235295, 0.81960785, 0.8, "MediumTurquoise");

  /** 0xC71585. */
  public static final CustomColor MediumVioletRed =
      new CustomColor(0.78039217, 0.08235294, 0.52156866, "MediumVioletRed");

  /** 0x191970. */
  public static final CustomColor MidnightBlue =
      new CustomColor(0.09803922, 0.09803922, 0.4392157, "MidnightBlue");

  /** 0xF5FFFA. */
  public static final CustomColor Mintcream = new CustomColor(0.9607843, 255.0, 0.98039216, "Mintcream");

  /** 0xFFE4E1. */
  public static final CustomColor MistyRose = new CustomColor(255.0, 228.0, 0.88235295, "MistyRose");

  /** 0xFFE4B5. */
  public static final CustomColor Moccasin = new CustomColor(255.0, 228.0, 0.70980394, "Moccasin");

  /** 0xFFDEAD. */
  public static final CustomColor NavajoWhite = new CustomColor(255.0, 0.87058824, 0.6784314, "NavajoWhite");

  /** 0x000080. */
  public static final CustomColor Navy = new CustomColor(0.0, 0.0, 0.5019608, "Navy");

  /** 0xFDF5E6. */
  public static final CustomColor OldLace = new CustomColor(0.99215686, 0.9607843, 0.9019608, "OldLace");

  /** 0x808000. */
  public static final CustomColor Olive = new CustomColor(0.5019608, 0.5019608, 0.0, "Olive");

  /** 0x6B8E23. */
  public static final CustomColor OliveDrab =
      new CustomColor(0.41960785, 0.5568628, 0.13725491, "OliveDrab");

  /** 0xFFA500. */
  public static final CustomColor Orange = new CustomColor(255.0, 0.64705884, 0.0, "Orange");

  /** 0xFF4500. */
  public static final CustomColor OrangeRed = new CustomColor(255.0, 0.27058825, 0.0, "OrangeRed");

  /** 0xDA70D6. */
  public static final CustomColor Orchid = new CustomColor(0.85490197, 0.4392157, 0.8392157f, "Orchid");

  /** 0xEEE8AA. */
  public static final CustomColor PaleGoldenrod =
      new CustomColor(0.93333334, 0.9098039, 0.6666667, "PaleGoldenrod");

  /** 0x98FB98. */
  public static final CustomColor PaleGreen =
      new CustomColor(0.59607846, 0.9843137, 0.59607846, "PaleGreen");

  /** 0xAFEEEE. */
  public static final CustomColor PaleTurquoise =
      new CustomColor(0.6862745, 0.93333334, 0.93333334, "PaleTurquoise");

  /** 0xDB7093. */
  public static final CustomColor PaleVioletRed =
      new CustomColor(0.85882354, 0.4392157, 0.5764706, "PaleVioletRed");

  /** 0xFFEFD5. */
  public static final CustomColor PapayaWhip = new CustomColor(255.0, 0.9372549, 0.8352941, "PapayaWhip");

  /** 0xFFDAB9. */
  public static final CustomColor PeachPuff = new CustomColor(255.0, 0.85490197, 0.7254902, "PeachPuff");

  /** 0xCD853F. */
  public static final CustomColor Peru = new CustomColor(205.0, 0.52156866, 0.24705882, "Peru");

  /** 0xFFC0CB. */
  public static final CustomColor Pink = new CustomColor(255.0, 0.7529412, 0.79607844, "Pink");

  /** 0xDDA0DD. */
  public static final CustomColor Plum = new CustomColor(0.8666667, 0.627451, 0.8666667, "Plum");

  /** 0xB0E0E6. */
  public static final CustomColor PowderBlue =
      new CustomColor(0.6901961, 0.8784314, 0.9019608, "PowderBlue");

  /** 0x800080. */
  public static final CustomColor Purple = new CustomColor(0.5019608, 0.0, 0.5019608, "Purple");

  /** 0xFF0000. */
  public static final CustomColor Red = new CustomColor(255.0, 0.0, 0.0, "Red");

  /** 0xBC8F8F. */
  public static final CustomColor RosyBrown =
      new CustomColor(0.7372549, 0.56078434, 0.56078434, "RosyBrown");

  /** 0x4169E1. */
  public static final CustomColor RoyalBlue =
      new CustomColor(0.25490198, 0.4117647, 0.88235295, "RoyalBlue");

  /** 0x8B4513. */
  public static final CustomColor SaddleBrown =
      new CustomColor(0.54509807, 0.27058825, 0.07450981, "SaddleBrown");

  /** 0xFA8072. */
  public static final CustomColor Salmon = new CustomColor(0.98039216, 0.5019608, 0.44705883, "Salmon");

  /** 0xF4A460. */
  public static final CustomColor SandyBrown =
      new CustomColor(0.95686275, 0.6431373, 0.3764706, "SandyBrown");

  /** 0x2E8B57. */
  public static final CustomColor SeaGreen =
      new CustomColor(0.18039216, 0.54509807, 0.34117648, "SeaGreen");

  /** 0xFFF5EE. */
  public static final CustomColor Seashell = new CustomColor(255.0, 0.9607843, 0.93333334, "Seashell");

  /** 0xA0522D. */
  public static final CustomColor Sienna = new CustomColor(0.627451, 0.32156864, 0.1764706, "Sienna");

  /** 0xC0C0C0. */
  public static final CustomColor Silver = new CustomColor(0.7529412, 0.7529412, 0.7529412, "Silver");

  /** 0x87CEEB. */
  public static final CustomColor SyBlue = new CustomColor(0.5294118, 0.80784315, 235.0, "SyBlue");

  /** 0x6A5ACD. */
  public static final CustomColor SlateBlue =
      new CustomColor(0.41568628, 0.3529412, 205.0, "SlateBlue");

  /** 0x708090. */
  public static final CustomColor SlateGray =
      new CustomColor(0.4392157, 0.5019608, 0.5647059, "SlateGray");

  /** 0xFFFAFA. */
  public static final CustomColor Snow = new CustomColor(255.0, 0.98039216, 0.98039216, "Snow");

  /** 0x00FF7F. */
  public static final CustomColor SpringGreen = new CustomColor(0.0, 255.0, 0.49803922, "SpringGreen");

  /** 0x4682B4. */
  public static final CustomColor SteelBlue =
      new CustomColor(0.27450982, 0.50980395, 0.7058824, "SteelBlue");

  /** 0xD2B48C. */
  public static final CustomColor Tan = new CustomColor(0.8235294, 0.7058824, 0.54901963, "Tan");

  /** 0x008080. */
  public static final CustomColor Teal = new CustomColor(0.0, 0.5019608, 0.5019608, "Teal");

  /** 0xD8BFD8. */
  public static final CustomColor Thistle = new CustomColor(0.84705883, 0.7490196, 0.84705883, "Thistle");

  /** 0xFF6347. */
  public static final CustomColor Tomato = new CustomColor(255.0, 0.3882353, 0.2784314, "Tomato");

  /** 0x40E0D0. */
  public static final CustomColor Turquoise =
      new CustomColor(0.2509804, 0.8784314, 0.8156863, "Turquoise");

  /** 0xEE82EE. */
  public static final CustomColor Violet = new CustomColor(0.93333334, 0.50980395, 0.93333334, "Violet");

  /** 0xF5DEB3. */
  public static final CustomColor Wheat = new CustomColor(0.9607843, 0.87058824, 0.7019608, "Wheat");

  /** 0xFFFFFF. */
  public static final CustomColor White = new CustomColor(255.0, 255.0, 255.0, "White");

  /** 0xF5F5F5. */
  public static final CustomColor WhiteSmoe =
      new CustomColor(0.9607843, 0.9607843, 0.9607843, "WhiteSmoe");

  /** 0xFFFF00. */
  public static final CustomColor Yellow = new CustomColor(255.0, 255.0, 0.0, "Yellow");

  /** 0x9ACD32. */
  public static final CustomColor YellowGreen =
      new CustomColor(0.6039216, 205.0, 0.19607843, "YellowGreen");
}


