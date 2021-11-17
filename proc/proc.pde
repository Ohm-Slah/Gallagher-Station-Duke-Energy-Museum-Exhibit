import processing.serial.*;

Serial myPort;
String val;

void setup() 
{
  String portName = Serial.list()[1];
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n');
  size(400,400);
  fill(0);
  textSize(32);
  ellipse(300,300,50,50);
}

void draw()
{

}

void serialEvent(Serial myPort)
{
  val = myPort.readStringUntil('\n');         // read it and store it in val
  
    if (val != null) 
    {
      fill(0);
      textSize(32);
      val = trim(val);
      println(val); //print it out in the console
      //fill(0);
      switch (val) {
        case "RESPOND": myPort.write("1");text("INIT", 0, 40); break;
        case "PHASE ZERO": myPort.write("1");text("Zero", 0, 40); break;
        case "PHASE ONE": myPort.write("1"); text("One", 0, 40);break;
        case "PHASE TWO": myPort.write("1"); text("Two", 0, 40);break;
        case "PHASE THREE": myPort.write("1"); text("Three", 0, 40);break;
        case "PHASE FOUR": myPort.write("1"); text("Four", 0, 40);break;
        case "FAILURE": myPort.write("1"); text("Failure", 0, 40);break;
        case "COMPLETE": myPort.write("1"); text("Complete", 0, 40);break;
        case "SLEEP": myPort.write("1"); text("Sleep", 0, 40);break;
        default: break;
      }
      
    }
}
