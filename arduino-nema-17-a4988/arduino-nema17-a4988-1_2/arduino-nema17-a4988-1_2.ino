int pins[] = {13,12,8,9,10,11}, velocidade;

void setup() {
  
  Serial.begin(9600);
  pinMode(pins[0], OUTPUT);
  pinMode(pins[1], OUTPUT);
  pinMode(pins[2], OUTPUT);
  pinMode(pins[3], OUTPUT);
  pinMode(pins[4], OUTPUT);
  pinMode(pins[5], OUTPUT);
  
}

void loop() {

  //velocidade = 5;
  //digitalWrite(pins[2], LOW); 
  digitalWrite(pins[3], HIGH); 
  digitalWrite(pins[4], LOW); 
  digitalWrite(pins[5], LOW); 

  digitalWrite(pins[0],HIGH); // Defini a direção de rotação
  digitalWrite(pins[1],HIGH); // Envia um pulso de um passo
  delay(5); 
  digitalWrite(pins[1],LOW);
 
}
