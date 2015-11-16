

#define RX_D3_PIN     2
#define RX_D2_PIN     3
#define RX_D1_PIN     4
#define LED_PIN       5
#define MC_EN_PIN     6
#define MC_4A_PIN     7
#define MC_3A_PIN     8
#define AUD_VOL_UP_PIN    9
#define AUD_VOL_DN_PIN    10
#define AUD_TRACK_SND01_PIN 11
#define AUD_TRACK_SND02_PIN 12
#define AUD_TRACK_SND03_PIN 13

#define AUD_TRACK_SND01_DURATION 5
#define AUD_TRACK_SND02_DURATION 14
#define AUD_TRACK_SND03_DURATION 3
#define CYCLE_LENGTH    45

bool trainRunning = false;
// Use this to calculate the current speed of the train.
long stateStartedAt = 0;
// because the timer can overflow we avoid that by only watching the time running until we hit fullspeed.
bool fullSpeed = false; 
bool stopped = true;

#define STATE_STOPPED       0
#define STATE_INTRO_WHISTLE 1
#define STATE_ALL_ABOARD    2
#define STATE_MOVING        3
#define STATE_STOPPING      4

long trackStartedAt = 0;
long cycleStartedAt = 0;

int currentMovingState = STATE_STOPPED;

void setup() {
    // Setup the Audio Board
    pinMode(AUD_VOL_UP_PIN, OUTPUT);
    pinMode(AUD_VOL_DN_PIN, OUTPUT);
    pinMode(AUD_TRACK_SND01_PIN, OUTPUT);
    digitalWrite(AUD_TRACK_SND01_PIN, HIGH);
    pinMode(AUD_TRACK_SND02_PIN, OUTPUT);
    digitalWrite(AUD_TRACK_SND02_PIN, HIGH);
    pinMode(AUD_TRACK_SND03_PIN, OUTPUT);
    digitalWrite(AUD_TRACK_SND03_PIN, HIGH);
    
    // Setup the radio
    pinMode(RX_D3_PIN, INPUT);
    pinMode(RX_D2_PIN, INPUT);
    pinMode(RX_D1_PIN, INPUT);
  
    // Setup the LED
    pinMode(LED_PIN, OUTPUT);
  
    // Setup the motor
    pinMode(MC_EN_PIN, OUTPUT);
    digitalWrite(MC_EN_PIN, LOW);
    pinMode(MC_4A_PIN, OUTPUT);
    digitalWrite(MC_4A_PIN, HIGH);
    pinMode(MC_3A_PIN, OUTPUT);
    digitalWrite(MC_3A_PIN, LOW);

    // Listen for the pin corresponding to the 
    // A button on the remote to change
    attachInterrupt(digitalPinToInterrupt(RX_D3_PIN), userToggledTrainState, CHANGE);

    Serial.begin(115200);
    Serial.println("Starting...");
}

void loop() {
    setLightState();
    setMotorState();
}

void userToggledTrainState() {
    if (digitalRead(RX_D3_PIN)) {
        startTrain();
    } else {
        stopTrain();
    }
}

void setLightState() {
    digitalWrite(LED_PIN, trainRunning && !digitalRead(RX_D2_PIN));
}

void setMotorState() {
    if (trainRunning) {
        switch(currentMovingState) {
            case STATE_STOPPED:
            {
                cycleStartedAt = millis();
                trackStartedAt = 0;
                setMovingState(STATE_INTRO_WHISTLE);
            }
            break;
            case STATE_INTRO_WHISTLE:
            {
                if (trackStartedAt == 0) {
                    trackStartedAt = millis();
                    digitalWrite(AUD_TRACK_SND02_PIN, LOW);
                } else {
                    long duration = (millis() - trackStartedAt)/1000;
                    if (duration > AUD_TRACK_SND02_DURATION) {
                        digitalWrite(AUD_TRACK_SND02_PIN, HIGH);
                        // Start the all aboard track
                        trackStartedAt = 0;
                        setMovingState(STATE_ALL_ABOARD);
                    }
                }
            }
            break;
            case STATE_ALL_ABOARD:
            {
                if (trackStartedAt == 0){
                    trackStartedAt = millis();
                    digitalWrite(AUD_TRACK_SND03_PIN, LOW);
                } else {
                    long duration = (millis() - trackStartedAt)/1000;
                    if (duration > AUD_TRACK_SND03_DURATION) {
                        digitalWrite(AUD_TRACK_SND03_PIN, HIGH);
                        // Start the all aboard track
                        trackStartedAt = 0;
                        setMovingState(STATE_MOVING);
                    }
                }
            }
            break;
            case STATE_MOVING:
            {
                accelerateTrain();
            }
            break;
            case STATE_STOPPING:
            {
                deccelerateTrain();
            }
            break;
            default:
            stopTrain();
        }
    } else {
        fullSpeed = false;
        setMovingState(STATE_STOPPED);
        digitalWrite(MC_EN_PIN, LOW);
    }
}

void accelerateTrain() {
    int runningFor = (millis() - stateStartedAt)/200;
    stopped = false;
    if (!fullSpeed) {
        int speed = 200 + runningFor;
        if (speed < 256) {
            analogWrite(MC_EN_PIN, speed);
            Serial.println(speed);
        } else {
            fullSpeed = true;
            digitalWrite(MC_EN_PIN, HIGH);
            Serial.println("Full Speed");
            
            trackStartedAt = millis();
            digitalWrite(AUD_TRACK_SND01_PIN, LOW);
        }
    } else {
        if(trackStartedAt > 0)
        {
            long duration = (millis() - trackStartedAt)/1000;
            if (duration > AUD_TRACK_SND01_DURATION) {
                digitalWrite(AUD_TRACK_SND01_PIN, HIGH);
                // Start the all aboard track
                trackStartedAt = 0;
            }
        }
        long cycleTime = (millis() - cycleStartedAt)/1000;
        if(trackStartedAt == 0 && cycleTime > CYCLE_LENGTH)
        {
            setMovingState(STATE_STOPPING);
        }
    }
}

void deccelerateTrain() {
    int runningFor = (millis() - stateStartedAt)/200;
    fullSpeed = false;
    if (!stopped) {
        int speed = 255 - runningFor;
        if (speed > 200) {
            analogWrite(MC_EN_PIN, speed);
            Serial.println(speed);
        } else {
            stopped = true;
            digitalWrite(MC_EN_PIN, LOW);
            Serial.println("Stopped");
            
            setMovingState(STATE_STOPPED);
        }
    }
}

void setMovingState(int movingState) {
    if(movingState != currentMovingState) {
      currentMovingState = movingState;
      stateStartedAt = millis();
      Serial.print("setMovingState: ");
      Serial.println(movingState);
    }
}

void startTrain() {
    Serial.println("startTrain");
    trainRunning = true;
    fullSpeed = false;
    setLightState();
}

void stopTrain() {
    trainRunning = false;
    setLightState();
    digitalWrite(MC_EN_PIN, LOW);
    digitalWrite(AUD_TRACK_SND01_PIN, HIGH);
    digitalWrite(AUD_TRACK_SND02_PIN, HIGH);
    digitalWrite(AUD_TRACK_SND03_PIN, HIGH);
    trackStartedAt = 0;    
    stopped = true;
    Serial.println("stopTrain");
}

