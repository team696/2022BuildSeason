// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSub extends SubsystemBase {
  public AddressableLED rightLED;
  public AddressableLEDBuffer rightBuffer;

//   public AddressableLED leftLED;
//   public AddressableLEDBuffer leftBuffer;

  
// public int m_rainbowFirstPixelHue;
public int pinkGreenFirstPixHue;
  
  /** Creates a new LEDSub. */
  public LEDSub() {
    rightLED = new AddressableLED(9);
    // leftLED = new AddressableLED(8);
    pinkGreenFirstPixHue = 60;

    // // Reuse buffer
    // // Default to a length of 60, start empty output
    // // Length is expensive to set, so only set it once, then just update data
    rightBuffer = new AddressableLEDBuffer(60);
    // leftBuffer = new AddressableLEDBuffer(60);
  
    rightLED.setLength(rightBuffer.getLength());
    // leftLED.setLength(leftBuffer.getLength());

    // // Set the data
    rightLED.setData(rightBuffer);
    rightLED.start();

    // leftLED.setData(leftBuffer);
    // leftLED.start();
    
    
  }
  // private void rainbow() {
  //   // For every pixel
  //   for (var i = 0; i < rightBuffer.getLength(); i++) {
  //     // Calculate the hue - hue is easier for rainbows because the color
  //     // shape is a circle so only one value needs to precess
  //     final var hue = (m_rainbowFirstPixelHue + (i * 180 / rightBuffer.getLength())) % 180;
  //     // Set the value
  //     rightBuffer.setHSV(i, hue, 255, 128);
  //   }
  //   // Increase by to make the rainbow "move"
  //   m_rainbowFirstPixelHue += 3;
  //   // Check bounds
  //   m_rainbowFirstPixelHue %= 180;
  // }

  public void pinkGreen(){

   int  redstep = (0 - 255)/rightBuffer.getLength();
   int greenStep = (255 - 0)/rightBuffer.getLength();
   int blueStep = (0 -221)/rightBuffer.getLength();

    for (var i = 0; i < rightBuffer.getLength(); i++) {
      
      // final var hue = (pinkGreenFirstPixHue + (i * 180 / rightBuffer.getLength())) % 155;
      int r = 255 + i * redstep;
      int g = 0 + i * greenStep;
      int b = 221 + i * blueStep;

      rightBuffer.setRGB(i, r, g, b);
    }

    pinkGreenFirstPixHue += 3;

    pinkGreenFirstPixHue %= 155;
  }

  public void setRightLEDs(int r, int g, int b){
    for (var i = 0; i < rightBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, r, g, b);
   }
  }

  
  public void setRightLEDsHSV(int h, int s, int v){
    for (var i = 0; i < rightBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setHSV(i, h, s, v);
   }
  }
  

  // public void setLeftLEDs(int r, int g, int b){

  //   for (var i = 0; i < leftBuffer.getLength(); i++) {
  //     // Sets the specified LED to the RGB values for red
  //     leftBuffer.setRGB(i, r, g, b);
  //  }
  // }


  

  @Override
  public void periodic() {
  //   for (var i = 0; i < rightBuffer.getLength(); i++) {
  //     // Sets the specified LED to the RGB values for red
  //     rightBuffer.setRGB(i, 255, 0, 0);
  //  }
//    for (var i = 5; i < 10; i++) {
//     // Sets the specified LED to the RGB values for red
//     rightBuffer.setRGB(i, 255, 255, 0);
//  }
  //  pinkGreen();
   rightLED.setData(rightBuffer);


   System.out.println(rightBuffer.getLength());


//    for (var i = 0; i < leftBuffer.getLength(); i++) {
//     // Sets the specified LED to the RGB values for red
//     leftBuffer.setRGB(i, 0, 255, 0);
//  }
 
//  leftLED.setData(rightBuffer);

    // This method will be called once per scheduler run
  }
}
