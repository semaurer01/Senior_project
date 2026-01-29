
boolean NL = true;

void setup() {

Serial.begin(9600);

while (!Serial) ;

Serial1.begin(9600);

}




void loop() {

char Serialdata;




if (Serial.available()) {

Serialdata = Serial.read();

Serial1.print(Serialdata);


if (NL) { Serial.print("\r\n>"); NL = false; }

Serial.write(Serialdata);

if (Serialdata==10) { NL = true; }

}

if (Serial1.available()) {

Serialdata = Serial1.read();

Serial.print(Serialdata);

}

}