import serial
import time

# Script pour mettre l'Arduino Nano R4 en mode DFU (bootloader)
# La séquence correcte est : ouvrir à 1200 bauds, puis fermer
print("Mise en mode DFU de l'Arduino Nano R4...")

try:
    # Ouvrir le port à 1200 bauds (cela déclenche le bootloader)
    ser = serial.Serial('/dev/ttyACM0', 1200, timeout=1)
    time.sleep(0.1)
    ser.close()
    print("Signal DFU envoyé avec succès!")
    print("Attente de 2 secondes pour que la carte passe en mode bootloader...")
    time.sleep(2)
    print("La carte devrait maintenant être en mode DFU.")
    print("Vous pouvez maintenant lancer: platformio run --target upload")
except serial.SerialException as e:
    print(f"Erreur: {e}")
    print("Vérifiez que le port /dev/ttyACM0 existe et que vous avez les permissions.")
except Exception as e:
    print(f"Erreur inattendue: {e}")