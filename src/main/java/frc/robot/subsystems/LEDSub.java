// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSub extends SubsystemBase {
  public AddressableLED rightLED;
  public AddressableLEDBuffer rightBuffer;


/* TODO JUST DONT TOUCH ANY OF THIS SHIT, WE MIGHT WANT TO MAKE IT LOOK COOL FOR DEMISTRATIONS */
  
public int m_rainbowFirstPixelHue;
public int pinkGreenFirstPixHue;


int r;
int g;
int b;
  
  /** Creates a new LEDSub. */
  public LEDSub() {
    rightLED = new AddressableLED(8);
    // leftLED = new AddressableLED(8);
    pinkGreenFirstPixHue = 60;
    m_rainbowFirstPixelHue = 60;

    // // Reuse buffer
    // // Default to a length of 60, start empty output
    // // Length is expensive to set, so only set it once, then just update data
    rightBuffer = new AddressableLEDBuffer(80);
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
// //-------------Pink and Green LED Code-------------------------

// //--------Alternating Pink and Green---------

// //Call static variables
// private int pinkGreen_timer_A=0,LEDnumber_A=0,DelayedLEDnumber_A=-12,LEDnumber2_A=25,DelayedLEDnumber2_A=13; //_A stands for Alternating

// //Make a function
//   public void pinkGreen_Alt(){ //_Alt stands for Alternating

// //Each time the function is called, the timer is changed by 1
//   pinkGreen_timer_A++;

// //When the function is called 5 times, the code advances *Change this value to 1 or remove if function for max speed*
//   if(pinkGreen_timer_A == 5)
//   {
//     //Resets timer
//     pinkGreen_timer_A=0; 

//     //Changes LED color from Green to pink
//     rightBuffer.setRGB(LEDnumber_A,255,0,255);
//     rightBuffer.setRGB(LEDnumber2_A,255,0,255);

//     //Moves on to the next led
//     LEDnumber_A++;
//     if(LEDnumber_A == 24)
//     {
//       LEDnumber_A = 0;
//     }

//     DelayedLEDnumber_A++;
//     if(DelayedLEDnumber_A == 24)
//     { 
//       DelayedLEDnumber_A = 0;
//     }

//     DelayedLEDnumber2_A++;
//     if(DelayedLEDnumber2_A == 50)
//     {
//       DelayedLEDnumber2_A = 0;
//     }

//     LEDnumber2_A++;
//     if(LEDnumber2_A== 50)
//     {
//       LEDnumber2_A = 0;
//     }

//     //Checks if there is desired number of pink LEDs
//     if (DelayedLEDnumber_A >= 0)
//     {
//       //Changes LED color from pink to green
//       rightBuffer.setRGB(DelayedLEDnumber_A,0,255,0);
//     }
    
//     //Does the same thing but for the other side
//     if (DelayedLEDnumber2_A >= 25)
//     {
//       rightBuffer.setRGB(DelayedLEDnumber2_A,0,255,0);
//     }
//   }
// }

// //----Bouncing Pink on LED Strip-----

// //Call static variables
// private int pinkGreen_timer_B=0,LEDnumber_B=0,DelayedLEDnumber_B=-3; //_B stands for Bouncing
// private boolean endReached = false, DelayedEndReached = false;

// //Make a function
//   public void pinkLightBounce()
//   {
//     //Each time the function is called, the timer is changed by 1
//     pinkGreen_timer_B++;

//     //When the function is called 3 times, the code advances *Change this value to 1 or remove if function for max speed*
//     if(pinkGreen_timer_B == 3)
//     {
//       //Resets timer
//       pinkGreen_timer_B=0; 

//       //Changes LED color from Green to pink
//       rightBuffer.setRGB(LEDnumber_B,255,0,255);

//       //Moves on to the next led until it reaches the end. If it reaches the end, it will reverse direction
//       if(endReached)
//       {
//         LEDnumber_B--;
//         if(LEDnumber_B == 0)
//         {
//         endReached = false;
//         }
//       }
//       else
//       {
//         LEDnumber_B++;
//         if(LEDnumber_B == 50)
//         {
//         endReached = true;
//         }
//       }
      
//       if(DelayedEndReached)
//       {
//         DelayedLEDnumber_B--;
//         if(DelayedLEDnumber_B == 0)
//         {
//           DelayedEndReached = false;
//         }
//       }
//       else
//       {
//         DelayedLEDnumber_B++;
//         if(DelayedLEDnumber_B == 50)
//         {
//           DelayedEndReached = true;
//         }
//       }

//       //Checks if there is desired number of pink LEDs
//       if (DelayedLEDnumber_B >= 0)
//       {
//         //Changes LED color from pink to green
//         rightBuffer.setRGB(DelayedLEDnumber_B,0,255,0);
//       }
//     }
//   }

//   //----Breathing Pink and Green-----

  //Call static variables
  private int LEDnumber=0;

  //Call the map function
  private int mapint(int x, int in_min, int in_max, int out_min, int out_max)
  {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  //Make a function
  public void pinkGreen_Brth() //Brth stands for Breathing
  {
    for (var i = 0; i <= 50; i++) {
      //Maps the first half of the i value to 0 to 255
      int refrenceValue1 = mapint(i,0,24,0,255);

      //Maps the second half of the i value to 0 to 255
      int refrenceValue2 = mapint(i,25,50,0,255);

      //Checks if the for loop is in its first half
      if(i < 25)
      {
      //Sets the specified RGB values to the LED (Fades from green to pink)
      rightBuffer.setRGB(LEDnumber, refrenceValue1, (255-refrenceValue1), refrenceValue1);  
      }

      //Checks if the for loop is in its second half
      else if(i >= 25)
      {
      //Sets the specified RGB values to the LED (Fades from pink to green)
      rightBuffer.setRGB(LEDnumber, (255-refrenceValue2), refrenceValue2, (255-refrenceValue2)); 
      }
      
   }

   //Moves on to the next LED
   LEDnumber++;

   //If it reached the last LED, it will restart from the beginning
   if(LEDnumber==50)
   {
     LEDnumber=0;
   }
  }

// //--------------End of Pink and Green LED Code ----------------

  public void setRightLEDs(int r, int g, int b){
    for (var i = 0; i <  rightBuffer.getLength() ; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, r, g, b);
   }
   rightLED.setData(rightBuffer);

  }

  
  public void setRightLEDsHSV(int h, int s, int v){
    for (var i = 0; i < rightBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setHSV(i, h, s, v);
   }
  //  rightLED.setData(rightBuffer);

  }

  public void armenianFlag(){
    for (var i = 11; i < 16; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 255, 0,0 );
    }

    for (var i = 16; i < 21; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 0, 20, 127);
    }

    for (var i = 21; i < 25; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 255, 100, 0);
    }

    // rightLED.setData(rightBuffer);



  }

  public void italianFlag(){
    for (var i = 0; i < 3; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 255, 0,0 );
    }

    for (var i = 3; i < 7; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 255, 255, 255);
    }

    for (var i = 7; i < 11; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 0, 255, 0);
    }
    // rightLED.setData(rightBuffer);

  }


  public void japaneseFlag(){
    for (var i = 25; i < 29; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 255, 255,255 );
    }

    for (var i = 29; i < 32; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 255, 0, 0);
    }

    for (var i = 32; i < 36; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 255, 255, 255);
    }
    // rightLED.setData(rightBuffer);

  }
  public void americanFlag(){
    for (var i = 36; i < 41; i++) {
      // Sets the specified LED to the RGB values for red
      rightBuffer.setRGB(i, 0, 0,255 );
    }
     
      rightBuffer.setRGB(41, 255, 0, 0);
      rightBuffer.setRGB(42, 255, 255, 255);
      rightBuffer.setRGB(43, 255, 0, 0);
      rightBuffer.setRGB(44, 255, 255, 255);
      rightBuffer.setRGB(45, 255, 0, 0);
      rightBuffer.setRGB(46, 255, 255, 255);
      rightBuffer.setRGB(47, 255, 0, 0);
      rightBuffer.setRGB(48, 255, 255, 255);
      rightBuffer.setRGB(49, 255, 0, 0);
      rightBuffer.setRGB(50, 255, 255, 255);

      // rightLED.setData(rightBuffer);


    // for (var i = 16; i < 21; i++) {
    //   // Sets the specified LED to the RGB values for red
    //   rightBuffer.setRGB(i, 0, 20, 127);
    // }

    // for (var i = 21; i < 25; i++) {
    //   // Sets the specified LED to the RGB values for red
    //   rightBuffer.setRGB(i, 255, 100, 0);
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




//    for (var i = 0; i < leftBuffer.getLength(); i++) {
//     // Sets the specified LED to the RGB values for red
//     leftBuffer.setRGB(i, 0, 255, 0);
//  }
 
//  leftLED.setData(rightBuffer);

    // This method will be called once per scheduler run
  }
}
