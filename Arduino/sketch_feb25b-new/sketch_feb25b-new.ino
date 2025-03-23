#define LED_PIN0 13  // LED for 'P' (Person)
#define LED_PIN1 12  // LED for 'C' (Car)
#define LED_PIN2 11  // LED for 'D' (Dog)

#define DETECTION_THRESHOLD 3  // Required detections before changing LED

char lastLabel = 'X';  // Stores the last received label
int detectionCount = 0;  // Counter for label consistency

void setup() {
    pinMode(LED_PIN0, OUTPUT);
    pinMode(LED_PIN1, OUTPUT);
    pinMode(LED_PIN2, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    if (Serial.available() > 0) {
        char receivedChar = Serial.read();  // Read the incoming character

        // If the same label is detected again, increase the counter
        if (receivedChar == lastLabel) {
            detectionCount++;
        } else {
            detectionCount = 1;  // Reset counter if new label appears
        }

        lastLabel = receivedChar;  // Update last detected label

        // Change LED only if the same label is detected enough times
        if (detectionCount >= DETECTION_THRESHOLD) {
            setLEDState(receivedChar);
        }
    }
}

// Function to update LED state based on received character
void setLEDState(char label) {
    digitalWrite(LED_PIN0, LOW);
    digitalWrite(LED_PIN1, LOW);
    digitalWrite(LED_PIN2, LOW);

    if (label == 'P') {
        digitalWrite(LED_PIN0, HIGH);  
    } else if (label == 'C') {
        digitalWrite(LED_PIN1, HIGH);  
    } else if (label == 'D') {
        digitalWrite(LED_PIN2, HIGH);  
    }
}
