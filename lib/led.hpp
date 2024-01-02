#ifndef LED_H
#define LED_H

enum ledColor{
    RED,
    GREEN,
    BLUE
};

class LedController{

    private:
        bool redLED = false;
        bool greenLED = false;  
        bool blueLED = false;
    
    public:
        void init(){
            //******************** LED Setup ********************//
            pinMode(LEDR, OUTPUT);
            pinMode(LEDG, OUTPUT);
            pinMode(LEDB, OUTPUT);
            digitalWrite(LEDR, LOW);
            digitalWrite(LEDG, LOW);
            digitalWrite(LEDB, LOW);
            //***************************************************//
        }

        void toggle(ledColor color){
            switch (color){
            case RED:
                if(redLED){digitalWrite(LEDR, LOW); redLED = false;} 
                else{ digitalWrite(LEDR, HIGH); redLED = true;}
                break;
            case GREEN:
                if(greenLED){digitalWrite(LEDG, LOW); greenLED = false;
                } else{ digitalWrite(LEDG, HIGH);greenLED = true;}
                break;
            case BLUE:
                if(blueLED){digitalWrite(LEDB, LOW); blueLED = false;
                } else{ digitalWrite(LEDB, HIGH);blueLED = true;}
                break;
            }
        }
        
        void set(ledColor color){
            switch (color){
            case RED:
                digitalWrite(LEDR, HIGH); redLED = true;
                break;
            case GREEN:
                digitalWrite(LEDG, HIGH); greenLED = true;
                break;
            case BLUE:
                digitalWrite(LEDB, HIGH); blueLED = true;
                break;
            }
        }

        void clear(ledColor color){
            switch (color){
            case RED:
                digitalWrite(LEDR, LOW); redLED = false;
                break;
            case GREEN:
                digitalWrite(LEDG, LOW); greenLED = false;
                break;
            case BLUE:
                digitalWrite(LEDB, LOW); blueLED = false;
                break;
            }
        }
};

#endif