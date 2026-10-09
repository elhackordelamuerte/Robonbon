# /// script
# requires-python = ">=3.11"
# dependencies = ["opencv-python==4.10.0.84", "numpy<2.3"]
# ///
"""Diffuse une caméra de l'hôte en MJPEG, pour le conteneur `perception`.

Docker Desktop (macOS) ne donne pas accès aux webcams depuis les conteneurs :
ce script tourne sur le Mac et le nœud `camera_node` lit le flux via
http://host.docker.internal:8081/stream.mjpg

    uv run scripts/camera_stream.py --list          # caméras disponibles (noms + index)
    uv run scripts/camera_stream.py --nom UGREEN    # RECOMMANDÉ sur macOS : caméra choisie par son nom (ffmpeg)
    uv run scripts/camera_stream.py --index 0       # par index OpenCV (l'ordre peut changer, ex. iPhone à côté)
    open http://localhost:8081                      # aperçu dans le navigateur

Au premier lancement, macOS demande l'autorisation d'accès à la caméra pour le terminal.
"""
import argparse
import re
import shutil
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

FRONTIERE = "image"
QUALITE_JPEG = 85
NB_INDEX_TESTES = 6
DELAI_RECONNEXION_S = 1.0
# Mode capteur demandé à AVFoundation : sans taille explicite, ffmpeg peut attendre indéfiniment
# un mode que la caméra ne sert pas. 1280x720 @ 30 i/s est supporté par la UGREEN ; réduit ensuite.
TAILLE_CAPTURE_FFMPEG = "1280x720"
# Sur macOS, le backend par défaut peut tomber sur OBSENSOR et échouer : forcer AVFoundation.
BACKEND = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY


def cameras_ffmpeg() -> list[str]:
    """Noms des caméras vus par ffmpeg (AVFoundation, macOS), dans l'ordre de ffmpeg."""
    if sys.platform != "darwin" or not shutil.which("ffmpeg"):
        return []
    sortie = subprocess.run(["ffmpeg", "-hide_banner", "-f", "avfoundation", "-list_devices", "true", "-i", ""],
                            capture_output=True, text=True).stderr
    video = sortie.split("audio devices")[0]
    return re.findall(r"\] \[\d+\] (.+)", video)


def lister_cameras() -> None:
    noms = cameras_ffmpeg()
    if noms:
        print("Par nom (--nom, recommandé) :")
        for nom in noms:
            print(f"  {nom}")
        print("Par index OpenCV (--index) :")
    for index in range(NB_INDEX_TESTES):
        capture = cv2.VideoCapture(index, BACKEND)
        ok, image = capture.read()
        if ok:
            print(f"  index {index} : {image.shape[1]}x{image.shape[0]}")
        capture.release()


class Capture:
    """Lit la caméra en continu dans un thread et garde la dernière image encodée en JPEG."""

    def __init__(self, index: int, largeur: int, hauteur: int, fps: int):
        self._index, self._largeur, self._hauteur, self._fps = index, largeur, hauteur, fps
        self._jpeg: bytes | None = None
        self._condition = threading.Condition()
        threading.Thread(target=self._boucle, daemon=True).start()

    def _ouvrir(self) -> cv2.VideoCapture:
        capture = cv2.VideoCapture(self._index, BACKEND)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self._largeur)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self._hauteur)
        capture.set(cv2.CAP_PROP_FPS, self._fps)
        return capture

    def _boucle(self) -> None:
        capture = self._ouvrir()
        while True:
            ok, image = capture.read()
            if not ok:
                print(f"⚠️  caméra {self._index} : lecture impossible, nouvelle tentative…")
                capture.release()
                time.sleep(DELAI_RECONNEXION_S)
                capture = self._ouvrir()
                continue
            ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, QUALITE_JPEG])
            if ok:
                with self._condition:
                    self._jpeg = jpeg.tobytes()
                    self._condition.notify_all()

    def attendre_image(self) -> bytes:
        with self._condition:
            self._condition.wait(timeout=2.0)
            return self._jpeg


class CaptureFfmpeg(Capture):
    """Capture par NOM de caméra via ffmpeg (macOS) : indépendante de l'ordre des index.

    ffmpeg produit directement un flux MJPEG ; on découpe les images JPEG (marqueurs FFD8 … FFD9).
    """

    def __init__(self, nom: str, largeur: int, hauteur: int, fps: int):
        trouves = [n for n in cameras_ffmpeg() if nom.lower() in n.lower()]
        if len(trouves) != 1:
            sys.exit(f"❌ Caméra « {nom} » : {len(trouves)} correspondance(s) {trouves} (voir --list)")
        self._nom = trouves[0]
        super().__init__(-1, largeur, hauteur, fps)

    def _boucle(self) -> None:
        commande = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-f", "avfoundation",
                    "-framerate", "30", "-video_size", TAILLE_CAPTURE_FFMPEG, "-i", f"{self._nom}:none",
                    # Largeur imposée, hauteur déduite : on GARDE le format du capteur (16:9) pour des pixels
                    # carrés (fx = fy), hypothèse de la calibration. 1280x720 -> 640x360.
                    "-vf", f"scale={self._largeur}:-2", "-r", str(self._fps),
                    "-f", "mjpeg", "-q:v", "4", "pipe:1"]
        print(f"🎥 Capture par nom : « {self._nom} »", flush=True)
        while True:
            processus = subprocess.Popen(commande, stdout=subprocess.PIPE)
            tampon = b""
            while True:
                bloc = processus.stdout.read(65536)
                if not bloc:
                    break
                tampon += bloc
                debut, fin = tampon.find(b"\xff\xd8"), tampon.find(b"\xff\xd9")
                while debut != -1 and fin > debut:
                    with self._condition:
                        self._jpeg = tampon[debut: fin + 2]
                        self._condition.notify_all()
                    tampon = tampon[fin + 2:]
                    debut, fin = tampon.find(b"\xff\xd8"), tampon.find(b"\xff\xd9")
            print(f"⚠️  « {self._nom} » : flux interrompu, nouvelle tentative…")
            processus.kill()
            time.sleep(DELAI_RECONNEXION_S)


def fabriquer_handler(capture: Capture):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                page = b'<html><body style="margin:0;background:#111"><img src="/stream.mjpg" style="max-width:100%"></body></html>'
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(page)
                return
            if self.path != "/stream.mjpg":
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={FRONTIERE}")
            self.end_headers()
            try:
                while True:
                    jpeg = capture.attendre_image()
                    if jpeg is None:
                        continue
                    self.wfile.write(f"--{FRONTIERE}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass  # client déconnecté

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--index", type=int, default=0, help="index OpenCV de la caméra")
    parser.add_argument("--nom", help="nom (ou partie du nom) de la caméra, ex. UGREEN : capture via ffmpeg (macOS)")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--largeur", type=int, default=640)
    parser.add_argument("--hauteur", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--list", action="store_true", help="liste les caméras puis quitte")
    args = parser.parse_args()

    if args.list:
        lister_cameras()
        return
    # 127.0.0.1 : accessible depuis les conteneurs via host.docker.internal (Docker Desktop),
    # mais pas depuis le réseau local. Port réservé avant d'ouvrir la caméra.
    try:
        serveur = ThreadingHTTPServer(("127.0.0.1", args.port), None)
    except OSError:
        sys.exit(f"❌ Port {args.port} déjà utilisé : un camera_stream tourne sans doute déjà "
                 f"(http://localhost:{args.port}). Arrêter avec : pkill -f camera_stream.py")
    if args.nom:
        if not shutil.which("ffmpeg"):
            sys.exit("❌ --nom nécessite ffmpeg (brew install ffmpeg), sinon utiliser --index")
        capture = CaptureFfmpeg(args.nom, args.largeur, args.hauteur, args.fps)
    else:
        capture = Capture(args.index, args.largeur, args.hauteur, args.fps)
    serveur.RequestHandlerClass = fabriquer_handler(capture)
    print(f"📷 Caméra {args.nom or args.index} diffusée sur http://localhost:{args.port}/stream.mjpg (Ctrl-C pour arrêter)")
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
