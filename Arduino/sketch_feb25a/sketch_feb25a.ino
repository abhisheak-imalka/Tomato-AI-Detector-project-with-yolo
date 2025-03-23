#define LED_PIN0 13// Define the pin for the LED (Change if using a different pin)
#define LED_PIN1 12//
#define LED_PIN2 11//

void setup() {
    pinMode(LED_PIN0, OUTPUT);  // Set LED pin as output
    pinMode(LED_PIN1, OUTPUT);
    pinMode(LED_PIN2, OUTPUT);
    Serial.begin(9600);  // Start serial communication at 9600 baud rate
}

void loop() {
    if (Serial.available() > 0) {  // Check if data is available from Python
        char receivedChar = Serial.read();  // Read the incoming character
        
        if (receivedChar == 'P') {
             
            digitalWrite(LED_PIN0, HIGH); 
            delay(5);

        } else if (receivedChar == 'C') {
             
            digitalWrite(LED_PIN1, HIGH);
             delay(5);  
        }
         else if (receivedChar == 'D') {
             
            digitalWrite(LED_PIN2, HIGH);
             delay(5);  
        }
         else if (receivedChar == 'X' || receivedChar == '0') {
             
            digitalWrite(LED_PIN0, LOW);  // Turn OFF LED
            digitalWrite(LED_PIN1, LOW);
            digitalWrite(LED_PIN2, LOW);
        }
    }
}
