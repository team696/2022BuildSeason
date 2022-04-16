// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSub extends SubsystemBase {
  public AddressableLED rightLED;
  public AddressableLEDBuffer rightBuffer;

//   public AddressableLED leftLED;
//   public AddressableLEDBuffer leftBuffer;

  
public int m_rainbowFirstPixelHue;
public int pinkGreenFirstPixHue;


int r;
int g;
int b;
  
  /** Creates a new LEDSub. */
  public LEDSub() {
    rightLED = new AddressableLED(9);
    // leftLED = new AddressableLED(8);
    pinkGreenFirstPixHue = 60;
    m_rainbowFirstPixelHue = 60;

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
  public void rainbow() {
    
    // For every pixel
    for (var i = 0; i < rightBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 155 / rightBuffer.getLength())) % 155;
      // Set the value
      rightBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 155;
  }
//   private int mapint(int x, int in_min, int in_max, int out_min, int out_max){
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

//-------------Pink and Green LED Code-------------------------
//--First Robot Code by Abu :)

//Call static variables
  private int pinkGreen_timer=0,LEDnumber=0,DelayedLEDnumber=-24,DelayedLEDnumber2=-1;
//Make a Function
  public void pinkGreen(){
//Each time the function is called, the timer is changed by 1
  pinkGreen_timer++;
//When the function is called 50 times, the code advances
  if(pinkGreen_timer == 5)
  {
    //Resets timer
    pinkGreen_timer=0; 
    //Call variable(used for the blue value of RGB) 
    //int b=0;
    //Graduadly fades green into white

    for (int i =  0; i <= 255000; i++)
    {
      int refrenceValue = Math.round(i/1000);
      rightBuffer.setRGB(LEDnumber, refrenceValue, 255, refrenceValue);
    }

    //Graduadly fades white to pink
    if(DelayedLEDnumber2 >= 0)
    {
    for (int i =  0; i <= 255000; i++)
    {
      int refrenceValue = Math.round(i/1000);
      rightBuffer.setRGB(DelayedLEDnumber2, 255, (255-refrenceValue), 255);
    }
    }
    //rightBuffer.setRGB(LEDnumber,255,0,235);
    //Moves on to the next led

    LEDnumber++;
    if(LEDnumber == 50)
    {
      LEDnumber = 0;
    }
    DelayedLEDnumber++;
    if(DelayedLEDnumber == 50)
    {
      DelayedLEDnumber = 0;
    }
    DelayedLEDnumber2++;
    if(DelayedLEDnumber2 == 50)
    {
      DelayedLEDnumber2 = 0;
    }

    //Resets variable(used for the blue value of RGB) 

    //b=0;
    //Checks if there is desired number of pink LEDs

    if (DelayedLEDnumber >= 0)
    {
      //Gradually fades pink into green

      rightBuffer.setRGB(DelayedLEDnumber,0,255,0);
      // for (int i = 255000; i == 0; i--)
      // {
      // int refrenceValue2 = Math.round(i/1000);
      // if(refrenceValue2-20 >= 0)
      // {
      // b = refrenceValue2 - 20;
      // }
      //   rightBuffer.setRGB(DelayedLEDnumber, refrenceValue2, (255-refrenceValue2), b);
      // }
    }
  }
}
//--------------End of Pink and Green LED Code ----------------

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
