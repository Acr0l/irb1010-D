char msgEnd = ';';
String instruccion;
bool newMsg = false;

int speed1 = 0;

void setup()
{
  Serial.begin(115200);
  Serial3.begin(38400);   // Inicializamos  el puerto serie
}

void loop()
{

  if (Serial3.available() > 0) {
    Serial.println("Se envía señal");
    instruccion = readBuff(); //Leer el mensaje entrante
    Serial.println(instruccion);
  }
}

String readBuff() {
  String buffArray;
  //int i = 0;

  while (Serial3.available() > 0) { //Entro a este while mientras exista algo en el puerto serial
    char buff = Serial3.read(); //Leo el byte entrante
    if (buff == msgEnd) {
      newMsg = true;
      break; //Si el byte entrante coincide con mi delimitador, me salgo del while
    } else {
      buffArray += buff; //Si no, agrego el byte a mi string para construir el mensaje
      //i += 1;
    }
    delay(10);
  }

  return buffArray;  //Retorno el mensaje
}
