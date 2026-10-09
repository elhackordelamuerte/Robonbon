# /// script
# requires-python = ">=3.11"
# dependencies = ["pyserial==3.5"]
# ///
"""Pont port série USB (bras SO-ARM101) <-> TCP, pour le conteneur `control`.

Docker Desktop (macOS) ne donne pas accès aux ports USB depuis les conteneurs : ce script
tourne sur le Mac, ouvre le port série du bras et le relaie en TCP brut sur 127.0.0.1:8765.
Dans le conteneur, socat recrée un port série virtuel (/dev/ttyBRIDGE) relié à ce pont
(SERIAL_BRIDGE=tcp:host.docker.internal:8765, cf. docker/control_entrypoint.sh).

    uv run scripts/serial_bridge.py --list                 # ports série disponibles
    uv run scripts/serial_bridge.py                        # port détecté automatiquement
    uv run scripts/serial_bridge.py --port /dev/cu.usbmodem5A7A0185311
    uv run scripts/serial_bridge.py --port loop://         # test sans bras (écho)
"""
import argparse
import functools
import socket
import sys
import termios
import threading
import time

import serial
from serial.tools import list_ports

print = functools.partial(print, flush=True)  # noqa: A001 - logs visibles même redirigés

BAUDRATE_FEETECH = 1_000_000  # débit par défaut des servos STS3215 (LeRobot)
# Pas 7000 : sur macOS, le récepteur AirPlay (ControlCenter) écoute déjà sur 5000 et 7000.
PORT_TCP = 8765
TAILLE_BLOC = 4096
DELAI_REOUVERTURE_S = 1.0
# Erreurs d'un port USB qui disparaît (câble, chute de tension) : on rouvre au lieu de planter.
ERREURS_SERIE = (OSError, serial.SerialException, termios.error)


def ports_candidats() -> list[str]:
    """Ports USB-série plausibles pour la carte du bras (macOS : /dev/cu.usbmodem*)."""
    return [p.device for p in list_ports.comports() if "usbmodem" in p.device or "usbserial" in p.device
            or "ttyACM" in p.device or "ttyUSB" in p.device]


def relayer(source_lire, destination_ecrire, arret: threading.Event) -> None:
    try:
        while not arret.is_set():
            donnees = source_lire()
            if donnees:
                destination_ecrire(donnees)
    except ERREURS_SERIE:
        pass
    finally:
        arret.set()


def ouvrir_port(port: str, baud: int) -> serial.SerialBase:
    """Ouvre le port série, en réessayant tant que le périphérique est absent."""
    signale = False
    while True:
        try:
            return serial.serial_for_url(port, baudrate=baud, timeout=0.01)
        except ERREURS_SERIE as erreur:
            if not signale:
                print(f"⏳ {port} indisponible ({erreur}) : nouvelle tentative chaque seconde…")
                signale = True
            time.sleep(DELAI_REOUVERTURE_S)


def port_valide(port_serie: serial.SerialBase) -> bool:
    try:
        port_serie.in_waiting  # noqa: B018 - lève une erreur si le périphérique a disparu
        return True
    except ERREURS_SERIE:
        return False


def servir_client(client: socket.socket, port_serie: serial.SerialBase) -> None:
    """Relaie dans les deux sens jusqu'à la déconnexion du client."""
    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # paquets servo courts : pas de délai Nagle
    client.settimeout(0.05)
    arret = threading.Event()

    def lire_tcp() -> bytes:
        try:
            donnees = client.recv(TAILLE_BLOC)
        except TimeoutError:
            return b""
        if not donnees:
            raise OSError("client déconnecté")
        return donnees

    def lire_serie() -> bytes:
        return port_serie.read(port_serie.in_waiting or 1)

    port_serie.reset_input_buffer()
    vers_serie = threading.Thread(target=relayer, args=(lire_tcp, port_serie.write, arret), daemon=True)
    vers_tcp = threading.Thread(target=relayer, args=(lire_serie, client.sendall, arret), daemon=True)
    vers_serie.start()
    vers_tcp.start()
    vers_serie.join()
    vers_tcp.join()
    client.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", help="port série du bras (défaut : détection automatique)")
    parser.add_argument("--baud", type=int, default=BAUDRATE_FEETECH)
    parser.add_argument("--tcp", type=int, default=PORT_TCP, help="port TCP local")
    parser.add_argument("--list", action="store_true", help="liste les ports série puis quitte")
    args = parser.parse_args()

    if args.list:
        for p in list_ports.comports():
            print(f"  {p.device}  ({p.description})")
        return
    port = args.port
    if port is None:
        candidats = ports_candidats()
        if len(candidats) != 1:
            sys.exit(f"❌ {len(candidats)} port(s) USB-série trouvé(s) {candidats} : préciser --port (voir --list)")
        port = candidats[0]

    serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serveur.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        # 127.0.0.1 : joignable depuis les conteneurs (host.docker.internal), pas depuis le réseau.
        serveur.bind(("127.0.0.1", args.tcp))
    except OSError:
        sys.exit(f"❌ Port TCP {args.tcp} déjà utilisé : un serial_bridge tourne déjà ? (pkill -f serial_bridge.py)")
    serveur.listen(1)
    port_serie = ouvrir_port(port, args.baud)
    print(f"🔌 {port} @ {args.baud} bauds relayé sur tcp://127.0.0.1:{args.tcp} (Ctrl-C pour arrêter)")
    try:
        while True:
            client, adresse = serveur.accept()
            print(f"  ↔ client connecté ({adresse[0]})")
            try:
                servir_client(client, port_serie)  # un seul client à la fois : le bus série n'est pas partageable
            except ERREURS_SERIE as erreur:
                print(f"  ⚠️ port série perdu ({erreur})")
                client.close()
            print("  ✕ client déconnecté")
            if not port_valide(port_serie):
                print("  🔁 réouverture du port série…")
                port_serie.close()
                port_serie = ouvrir_port(port, args.baud)
                print(f"  🔌 {port} rouvert")
    except KeyboardInterrupt:
        pass
    finally:
        port_serie.close()
        serveur.close()


if __name__ == "__main__":
    main()
