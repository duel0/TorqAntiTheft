# Librerie
import RPi.GPIO as GPIO
import serial
import time
import datetime
from telegram.ext import Updater, CommandHandler
from telegram import Bot

# Funzioni di servizio Telegram

def userHandler(update, topic):
    topic.bot.send_message(chat_id=update.effective_chat.id, text="CIAO")
    print(update.effective_chat.id)

with open("/home/torq/.telegram_tokenBot", "r") as BotToken:
    cmteqBot = BotToken.read().rstrip()
    print(cmteqBot)

bot = Bot(token=cmteqBot)

updater = Updater(token=cmteqBot)
dispatcher = updater.dispatcher
dispatcher.add_handler(CommandHandler('user', userHandler))
 

updater.start_polling()
print("BOT IS WATCHING")

# GPIO, comunicazione con STM32 (2 Way Handshaking)

SENSOR_PIN = 17
CONFIRM_PIN = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN,GPIO.IN)
GPIO.setup(CONFIRM_PIN,GPIO.OUT)

DEBOUNCE_DELAY = 0.05

# GPIO, comunicazione con Flipper (UART)

ser = serial.Serial(port="/dev/ttyS0",baudrate=115200,timeout=1)

now = datetime.datetime.now() # Da aggiornare ad ogni rilevazione

message =f'''
torq. Anti Theft System
Detected Suspicious Movement
{now}

Francesco Balassone
Luca Pisani
'''

try:
    # Handshaking
    movement_detected = False
    last_detection_time = 0
    confirmation_sent = False
    
    # Gestione UART
    
    if ser.is_open:
        print("Seriale aperta correttamente")
    else:
        print("Errore nell'apertura della seriale")
        
    
    while True:
        current_time = time.time()
        if GPIO.input(SENSOR_PIN) == GPIO.HIGH:
            if not movement_detected and (current_time-last_detection_time > DEBOUNCE_DELAY):
                print("ALERT")
                
                now = datetime.datetime.now() # Da aggiornare ad ogni rilevazione

                message =f'''
torq. Anti Theft System
Detected Suspicious Movement

{now}

---------------------------
'''
                ser.write(message.encode("utf-8"))
                bot.send_message(chat_id = "-4206121589", text=message)
                movement_detected = True
                last_detection_time = current_time
                GPIO.output(CONFIRM_PIN,GPIO.HIGH)
                confirmation_sent = True
        if movement_detected and GPIO.input(SENSOR_PIN) == GPIO.LOW:
            if confirmation_sent:
                GPIO.output(CONFIRM_PIN,GPIO.LOW)
                confirmation_sent = False
            movement_detected = False                
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Interruzione da tastiera")
finally:
    GPIO.cleanup()
    ser.close()