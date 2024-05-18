# Librerie
import RPi.GPIO as GPIO
import serial
import time
import datetime
from telegram.ext import Updater, CommandHandler
from telegram import Bot

ALARM = False

# Funzioni di servizio Telegram

def systemCheck(update, topic):
    topic.bot.send_message(chat_id=update.effective_chat.id, text="SYSTEM IS ON")
    print(update.effective_chat.id)

def activateAlarm(update, topic):
    global ALARM
    ALARM = True
    topic.bot.send_message(chat_id=update.effective_chat.id, text="ALARM ACTIVATED!")

def shutdownAlarm(update, topic):
    global ALARM
    ALARM = False
    topic.bot.send_message(chat_id=update.effective_chat.id, text="ALARM DEACTIVATED!")

def HELP(update, topic):
    topic.bot.send_message(chat_id=update.effective_chat.id, text='''
    AVAIABLE COMMANDS:
                            
    /systemcheck
    /activate
    /shutdown
    ''')


with open("/home/torq/.telegram_tokenBot", "r") as BotToken:
    cmteqBot = BotToken.read().rstrip()
    print(cmteqBot)

bot = Bot(token=cmteqBot)

updater = Updater(token=cmteqBot)
dispatcher = updater.dispatcher
dispatcher.add_handler(CommandHandler('systemcheck', systemCheck))
dispatcher.add_handler(CommandHandler('activate', activateAlarm))
dispatcher.add_handler(CommandHandler('shutdown', shutdownAlarm))
dispatcher.add_handler(CommandHandler('help', HELP))
 

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
        if ALARM:

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

                    bot.send_message(chat_id="-4206121589", text="prova: SEI TU? HAI 10 SECONDI PER DIGITARE IL COMANDO /shutdown. ")        
                    time.sleep(10)
                    bot.send_message(chat_id="-4206121589", text="prova: NON HAI DISATTIVATO L' ALLARME! 5 secondi alla riattivazione del sistema ")
                    time.sleep(5) 

            if movement_detected and GPIO.input(SENSOR_PIN) == GPIO.LOW:
                if confirmation_sent:
                    GPIO.output(CONFIRM_PIN,GPIO.LOW)
                    confirmation_sent = False
                movement_detected = False                
            time.sleep(0.1)
        
        else:
            print("ALLARM OFF!")
            time.sleep(2)
except KeyboardInterrupt:
    print("Interruzione da tastiera")
finally:
    GPIO.cleanup()
    ser.close()
