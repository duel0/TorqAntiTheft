#!/usr/bin/env python3
from telegram.ext import Updater, CommandHandler
from telegram import Bot
import RPi.GPIO as GPIO
import time

SIGNAL_PIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(SIGNAL_PIN, GPIO.IN)

USER_ID = -4206121589

def userHandler(update, topic):
    topic.bot.send_message(chat_id=update.effective_chat.id, text="CIAO")
    print(update.effective_chat.id)

with open("/home/luca/.local/share/.telegram_tokenBot", "r") as BotToken:
    cmteqBot = BotToken.read().rstrip()
    #print(cmteqBot)

bot = Bot(token=cmteqBot)

updater = Updater(token=cmteqBot)
dispatcher = updater.dispatcher
dispatcher.add_handler(CommandHandler('user', userHandler))
 

updater.start_polling()
print("BOT IS WATCHING")
#updater.idle()


try:

    while True:
        signal = GPIO.input(SIGNAL_PIN)
        if signal == 1:
            print("Invio messaggio su telegram")
            bot.send_message(chat_id = USER_ID, text="ATTENZIONE! MOVIMENTO SOSPETTO RILEVATO")
        time.sleep(1)

except KeyboardInterrupt:
    print("Program exiting...")
    updater.stop()