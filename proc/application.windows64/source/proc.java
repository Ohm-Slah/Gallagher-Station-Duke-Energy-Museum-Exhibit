import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import processing.serial.*; 
import processing.video.*; 
import processing.sound.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class proc extends PApplet {



//codeanticode.gsvideo.*;


SoundFile file;
Movie Intro;
Movie Phase0;
Movie Phase1;
Movie Phase2;
Movie Complete;
Movie Fail;
Serial myPort;
String val;
int vidToPlay;

public void setup() 
{
  Phase0 = new Movie(this, "PHASE0.mov");
  Phase1 = new Movie(this, "PHASE1.mov");
  Phase2 = new Movie(this, "PHASE2.mov");
  Complete = new Movie(this, "COMPLETE.mov");
  Fail = new Movie(this, "FAIL.mov");
  
  file = new SoundFile(this, "phone.mp3");
  //fullScreen();
  String portName = Serial.list()[2];
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n');
  
  
  //delay(5000);
  //Intro = new Movie(this, "");
  
  //delay(5000);
  Phase0.loop();
  Phase1.loop();
  Phase2.loop();
  Complete.loop();
  Fail.loop();
  file.stop();
  Phase0.stop();
  Phase1.stop();
  Phase2.stop();
  Complete.stop();
  Fail.stop();
  //file.loop();
  
  //Phase0.stop();
  //Phase1.stop(); 
  //Phase2.stop(); 
  //Fail.stop(); 
  //Complete.stop();
  //file.stop();
  
}

public void movieEvent(Movie myMovie) 
{
  myMovie.read();
}

public void serialEvent(Serial myPort)
{
  try 
  {
    val = myPort.readStringUntil('\n');         // read it and store it in val
  
    if (val != null)  
    {
      val = trim(val);
      println(val); //print it out in the console
      switch (val) 
      {
        case "RESPOND": myPort.write("1");break;
        case "PHASE ZERO": myPort.write("1"); Phase1.stop(); Phase2.stop(); Fail.stop(); Complete.stop(); Phase0.play(); vidToPlay = 0; break;
        case "PHASE ONE": myPort.write("1"); Phase0.stop(); Phase2.stop(); Fail.stop(); Complete.stop(); Phase1.play();  vidToPlay = 1; break;
        case "PHASE TWO": myPort.write("1"); Phase1.stop(); Phase1.noLoop(); Phase2.play(); vidToPlay = 2; break;
        case "PHASE THREE": myPort.write("1");break;
        case "PHASE FOUR": myPort.write("1"); break;
        case "FAILURE": myPort.write("1"); Phase1.stop(); Phase2.stop(); Complete.stop();file.stop(); vidToPlay = 20; break;
        case "COMPLETE": myPort.write("1"); Complete.play(); Phase1.stop(); Phase2.stop(); Fail.stop(); vidToPlay = 10; break;
        case "RING": myPort.write("1"); Phase1.stop(); Phase2.stop();file.play();Fail.play(); vidToPlay = 20; break;
        //case "SLEEP": myPort.write("1"); text("Sleep", 0, 40);break;
        default: break;
      }
    }
  }
  catch(RuntimeException e) 
  {
    e.printStackTrace();
  }
  
}

public void draw()
{
  try 
  {
    switch(vidToPlay)
    {
     case 0: image(Phase0, 0, 0); break;
     case 1: image(Phase1, 0, 0); break;
     case 2: image(Phase2, 0, 0); break;
     case 10: image(Complete, 0, 0); break;
     case 20: image(Fail, 0, 0); break;
     default: break;
    }
  }
  catch(RuntimeException e) 
  {
    e.printStackTrace();
  }
}
  public void settings() {  size(1820, 950); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "--present", "--window-color=#666666", "--stop-color=#cccccc", "proc" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
