Import("env")
import serial
import time

def reset_dfu_before_upload(source, target, env):
    """Script pour mettre l'Arduino Nano R4 en mode DFU avant l'upload"""
    print("Mise en mode DFU de l'Arduino Nano R4...")

    try:
        # Récupérer le port série de l'environnement PlatformIO
        upload_port = env.get("UPLOAD_PORT", "/dev/motor")

        # Ouvrir le port à 1200 bauds (cela déclenche le bootloader)
        ser = serial.Serial(upload_port, 1200, timeout=1)
        time.sleep(0.1)
        ser.close()
        print("Signal DFU envoyé avec succès!")
        print("Attente de 2 secondes pour que la carte passe en mode bootloader...")
        time.sleep(2)
        print("La carte est maintenant en mode DFU.")
    except serial.SerialException as e:
        print(f"Erreur: {e}")
        print(f"Vérifiez que le port {upload_port} existe et que vous avez les permissions.")
    except Exception as e:
        print(f"Erreur inattendue: {e}")

# Enregistrer le callback avant l'upload
env.AddPreAction("upload", reset_dfu_before_upload)