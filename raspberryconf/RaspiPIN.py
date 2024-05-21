# Librerie
import RPi.GPIO as GPIO
import serial
import time
import datetime
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters, ConversationHandler
from telegram import Bot, Update
from telegram.ext.callbackcontext import CallbackContext

# Stati per la conversazione di Telegram
PIN_INPUT, ACTIVATE, DEACTIVATE, SET_PIN = range(4)

# Variabili globali
ALARM = False
PIN = "1234"  # PIN hardcoded iniziale
telegram_users_in_conversation = {}

# Funzioni di servizio Telegram
def start_pin_input(update: Update, context: CallbackContext):
    update.message.reply_text("Inserisci il nuovo PIN a 4 cifre:")
    return SET_PIN

def set_new_pin(update: Update, context: CallbackContext):
    global PIN
    new_pin = update.message.text.strip()
    if len(new_pin) == 4 and new_pin.isdigit():
        PIN = new_pin
        update.message.reply_text(f"Nuovo PIN impostato correttamente.")
        return ConversationHandler.END
    else:
        update.message.reply_text("PIN non valido. Assicurati che sia a 4 cifre.")
        return SET_PIN

def request_pin(update: Update, context: CallbackContext):
    update.message.reply_text("Inserisci il PIN:")
    return PIN_INPUT

def check_pin(update: Update, context: CallbackContext):
    global ALARM
    user_input = update.message.text.strip()
    if user_input == PIN:
        command = telegram_users_in_conversation[update.effective_chat.id]
        if command == 'activate':
            ALARM = True
            update.message.reply_text("Allarme attivato!")
        elif command == 'deactivate':
            ALARM = False
            update.message.reply_text("Allarme disattivato!")
        elif command == 'set_pin':
            return start_pin_input(update, context)
        return ConversationHandler.END
    else:
        update.message.reply_text("PIN errato. Riprova.")
        return PIN_INPUT

def activate_alarm(update: Update, context: CallbackContext):
    telegram_users_in_conversation[update.effective_chat.id] = 'activate'
    return request_pin(update, context)

def shutdown_alarm(update: Update, context: CallbackContext):
    telegram_users_in_conversation[update.effective_chat.id] = 'deactivate'
    return request_pin(update, context)

def help_command(update: Update, context: CallbackContext):
    update.message.reply_text('''
    AVAILABLE COMMANDS:
                            
    /systemcheck
    /activate
    /shutdown
    /setpin
    ''')

def system_check(update: Update, context: CallbackContext):
    context.bot.send_message(chat_id=update.effective_chat.id, text="SYSTEM IS ON")
    print(update.effective_chat.id)

# Lettura del token del bot
with open("/home/torq/.telegram_tokenBot", "r") as BotToken:
    cmteqBot = BotToken.read().rstrip()
    print(cmteqBot)

# Inizializzazione del bot
bot = Bot(token=cmteqBot)
updater = Updater(token=cmteqBot)
dispatcher = updater.dispatcher

# Configurazione delle conversazioni di Telegram
conv_handler = ConversationHandler(
    entry_points=[
        CommandHandler('setpin', start_pin_input),
        CommandHandler('activate', activate_alarm),
        CommandHandler('shutdown', shutdown_alarm)
    ],
    states={
        SET_PIN: [MessageHandler(Filters.text & ~Filters.command, set_new_pin)],
        PIN_INPUT: [MessageHandler(Filters.text & ~Filters.command, check_pin)]
    },
    fallbacks=[CommandHandler('cancel', lambda update, context: update.message.reply_text("Operazione annullata."))]
)

dispatcher.add_handler(conv_handler)
dispatcher.add_handler(CommandHandler('systemcheck', system_check))
dispatcher.add_handler(CommandHandler('help', help_command))

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
    global ALARM, PIN
    if command == "a":
        ser.write("Inserisci il PIN per attivare l'allarme:\n".encode("utf-8"))
        input_pin = ser.readline().decode("utf-8").strip()
        if input_pin == PIN:
            ALARM = True
            ser.write("Allarme attivato!\n".encode("utf-8"))
        else:
            ser.write("PIN errato!\n".encode("utf-8"))
    elif command == "s":
        ser.write("Inserisci il PIN per disattivare l'allarme:\n".encode("utf-8"))
        input_pin = ser.readline().decode("utf-8").strip()
        if input_pin == PIN:
            ALARM = False
            ser.write("Allarme disattivato!\n".encode("utf-8"))
        else:
            ser.write("PIN errato!\n".encode("utf-8"))
    elif command == "p":
        ser.write("Inserisci il nuovo PIN a 4 cifre:\n".encode("utf-8"))
        new_pin = ser.readline().decode("utf-8").strip()
        if len(new_pin) == 4 and new_pin.isdigit():
            PIN = new_pin
            ser.write("Nuovo PIN impostato correttamente.\n".encode("utf-8"))
        else:
            ser.write("PIN non valido. Assicurati che sia a 4 cifre.\n".encode("utf-8"))

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
