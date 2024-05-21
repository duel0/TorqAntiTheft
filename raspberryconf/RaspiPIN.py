# Librerie
import RPi.GPIO as GPIO
import serial
import time
import datetime
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters
from telegram import Bot

# Variabili globali
ALARM = False
PIN = "1234"  # Imposta il PIN di sicurezza

# Funzioni di servizio Telegram
def systemCheck(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="SYSTEM IS ON")
    print(update.effective_chat.id)

def activateAlarm(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Inserisci il PIN per attivare l'allarme:")

def shutdownAlarm(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text="Inserisci il PIN per disattivare l'allarme:")

def checkPIN(update, context):
    global ALARM
    user_input = update.message.text.split()
    if len(user_input) == 2 and user_input[0].lower() == "/pin":
        input_pin = user_input[1]
        if input_pin == PIN:
            ALARM = not ALARM
            state = "attivato" if ALARM else "disattivato"
            context.bot.send_message(chat_id=update.effective_chat.id, text=f"Allarme {state}!")
        else:
            context.bot.send_message(chat_id=update.effective_chat.id, text="PIN errato!")
    else:
        context.bot.send_message(chat_id=update.effective_chat.id, text="Formato comando errato! Usa /pin <PIN>")

def HELP(update, context):
    context.bot.send_message(chat_id=update.effective_chat.id, text='''
    AVAILABLE COMMANDS:
                            
    /systemcheck
    /activate
    /shutdown
    /pin <PIN>
    ''')

# Lettura del token del bot
with open("/home/torq/.telegram_tokenBot", "r") as BotToken:
    cmteqBot = BotToken.read().rstrip()
    print(cmteqBot)

# Inizializzazione del bot
bot = Bot(token=cmteqBot)
updater = Updater(token=cmteqBot)
dispatcher = updater.dispatcher

# Registrazione dei comandi
dispatcher.add_handler(CommandHandler('systemcheck', systemCheck))
dispatcher.add_handler(CommandHandler('activate', activateAlarm))
dispatcher.add_handler(CommandHandler('shutdown', shutdownAlarm))
dispatcher.add_handler(CommandHandler('help', HELP))
dispatcher.add_handler(MessageHandler(Filters.text & ~Filters.command, checkPIN))

# Avvio del bot
updater.start_polling()
print("BOT IS WATCHING")

# Configurazione GPIO, comunicazione con STM32 (2 Way Handshaking)
SENSOR_PIN = 17
CONFIRM_PIN = 27
GPIO.setmode(GPIO.BCM)
GPIO.setup(SENSOR_PIN, GPIO.IN)
GPIO.setup(CONFIRM_PIN, GPIO.OUT)

DEBOUNCE_DELAY = 0.05

# Configurazione UART per la comunicazione con Flipper
ser = serial.Serial(port="/dev/ttyS0", baudrate=115200, timeout=1)

# Messaggio di allarme
def create_alarm_message():
    now = datetime.datetime.now()
    return f'''
torq. Anti Theft System
Detected Suspicious Movement
{now}
---------------------------
'''

# Gestione comandi via seriale
def process_serial_command(command):
    global ALARM
    if command.startswith("PIN "):
        input_pin = command.split()[1]
        if input_pin == PIN:
            ALARM = not ALARM
            state = "attivato" si ALARM else "disattivato"
            ser.write(f"Allarme {state}!\n".encode("utf-8"))
        else:
            ser.write("PIN errato!\n".encode("utf-8"))

# Funzione principale
def main():
    try:
        movement_detected = False
        last_detection_time = 0
        confirmation_sent = False

        # Verifica dell'apertura della seriale
        if ser.is_open:
            print("Seriale aperta correttamente")
        else:
            print("Errore nell'apertura della seriale")
        
        while True:
            # Lettura dei comandi via seriale
            if ser.in_waiting > 0:
                command = ser.readline().decode("utf-8").strip()
                process_serial_command(command)
            
            if ALARM:
                current_time = time.time()
                if GPIO.input(SENSOR_PIN) == GPIO.HIGH:
                    if not movement_detected and (current_time - last_detection_time > DEBOUNCE_DELAY):
                        print("ALERT")
                        message = create_alarm_message()
                        ser.write(message.encode("utf-8"))
                        bot.send_message(chat_id="-4206121589", text=message)
                        movement_detected = True
                        last_detection_time = current_time
                        GPIO.output(CONFIRM_PIN, GPIO.HIGH)
                        confirmation_sent = True

                        bot.send_message(chat_id="-4206121589", text="SEI TU? HAI 10 SECONDI PER DIGITARE IL COMANDO /shutdown.")
                        time.sleep(10)
                        bot.send_message(chat_id="-4206121589", text="NON HAI DISATTIVATO L'ALLARME! 5 secondi alla riattivazione del sistema")
                        time.sleep(5)

                if movement_detected and GPIO.input(SENSOR_PIN) == GPIO.LOW:
                    if confirmation_sent:
                        GPIO.output(CONFIRM_PIN, GPIO.LOW)
                        confirmation_sent = False
                    movement_detected = False
                time.sleep(0.1)
            else:
                print("ALARM OFF!")
                time.sleep(2)
    except KeyboardInterrupt:
        print("Interruzione da tastiera")
    finally:
        GPIO.cleanup()
        ser.close()

# Esecuzione del programma principale
if __name__ == '__main__':
    main()
