#!/usr/bin/env python3
"""Montage de la vidéo de démo à partir des rushes de scripts/video_sim.sh.

Entrées (dans <dossier>) :
  rviz.mp4            capture de la vue 3D de RViz (ffmpeg x11grab)
  debut.txt           heure murale du début de la capture (s)
  evenements.jsonl    logs du cerveau / de la perception + positions du cube (enregistreur.py)
  camera/<t>.jpg      images annotées par la perception (/perception/debug)
  scene/<t>.jpg       vue d'ensemble MuJoCo (/scene_cam/image_raw), facultative
  parametres.json     erreur de calibration simulée (mm), pour le texte affiché

Sortie : <dossier>/pick_place_sim.mp4

Tous les textes de l'overlay sont tirés des logs réels, à l'heure où ils ont été publiés :
rien n'est recalé à la main. Seul l'« offset appris » est déduit (pas loggé par le cerveau) :
c'est le décalage de l'essai validé (brain_node.py, l. 264 et 462).

Usage (dans le conteneur, ffmpeg avec libass) :  python3 montage.py <dossier>
"""
import ast
import json
import re
import subprocess
import sys
from pathlib import Path

import cv2

LARGEUR, HAUTEUR = 1920, 1080
LARGEUR_CAMERA = 460
ECART_INCRUSTATIONS = 52   # place pour la légende de l'incrustation du dessus
MARGE = 30
DUREE_TITRE_S = 5.0
DUREE_VERDICT_S = 3.5
DUREE_FIN_S = 6.0
MAX_ESSAIS = 6
POLICE = "DejaVu Sans"
POLICE_MONO = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf"

ETAPES = {
    "planned": "Planification de la trajectoire (cinématique inverse)",
    "move above object": "Déplacement au-dessus du cube",
    "approach": "Approche",
    "open gripper": "Ouverture de la pince",
    "align": "Alignement",
    "descend": "Descente vers le cube",
    "seat the jaw": "Mise en place de la mâchoire",
    "close gripper": "Fermeture de la pince",
    "hold": "Serrage",
    "lift": "Levée",
    "verify grasp": "Vérification de la prise par la caméra",
    "reopen gripper": "Réouverture de la pince",
    "retry": "Relance : nouvel essai à côté",
    "move above box": "Transport vers la boîte",
    "lower": "Descente au-dessus de la boîte",
    "release": "Lâcher",
    "let it fall": "Lâcher",
    "up": "Remontée",
    "back to rest": "Retour au repos",
}

RE_ETAPE = re.compile(r"^pick & place: ([a-z ]+?)(?:,|$)")
RE_VERDICT = re.compile(r"^pick & place: grasp (OK|FAILED) \(cube ([\d.]+) px")
RE_RELANCE = re.compile(r"^pick & place: retry (\d+), target offset \(([+-]\d+), ([+-]\d+)\) mm")
RE_ABANDON = re.compile(r"giving up")
RE_SEUIL = re.compile(r"'seuil_px': ([\d.]+)")
RE_RAISON_SOL = re.compile(r"'raison': 'cube encore sur la table à ([\d.]+) cm")
SEUIL_PAR_DEFAUT_PX = 35.0  # SEUIL_PRISE_PX de perception_node.py, si le log ne le donne pas

LEGENDE = (r"{\c&H002619E6&}■{\c&H00FFFFFF&} cube rouge : cube vu par la caméra\N"
           r"{\c&H004040FF&}✛{\c&H00FFFFFF&} TF : world, base_link, pince, caméra\N"
           r"{\c&H002020B0&}□{\c&H00FFFFFF&} bloc translucide : boîte de dépôt")


# ------------------------------------------------------------------ ASS

def temps_ass(t: float) -> str:
    t = max(0.0, t)
    h, reste = divmod(t, 3600)
    m, s = divmod(reste, 60)
    return f"{int(h)}:{int(m):02d}:{s:05.2f}"


def ligne(debut: float, fin: float, style: str, texte: str, couche: int = 1) -> str:
    return f"Dialogue: {couche},{temps_ass(debut)},{temps_ass(fin)},{style},,0,0,0,,{texte}"


def bande(debut: float, fin: float, haut: int, bas: int) -> str:
    """Bandeau sombre sur toute la largeur (sous les textes, couche 0)."""
    return ligne(debut, fin, "Bande", r"{\fad(300,400)\pos(0,0)\alpha&H38&\p1}"
                 f"m 0 {haut} l {LARGEUR} {haut} {LARGEUR} {bas} 0 {bas}" r"{\p0}", couche=0)


def px(v: float) -> str:
    texte = f"{v:.0f}" if float(v).is_integer() else f"{v:.1f}".replace(".", ",")
    return texte + " px"


ENTETE_ASS = f"""[Script Info]
ScriptType: v4.00+
PlayResX: {LARGEUR}
PlayResY: {HAUTEUR}
WrapStyle: 2

[V4+ Styles]
Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, Alignment, MarginL, MarginR, MarginV, Encoding
Style: Titre,{POLICE},64,&H00FFFFFF,&H00FFFFFF,&H00000000,&H00000000,1,0,0,0,100,100,0,0,1,2,0,5,0,0,0,1
Style: SousTitre,{POLICE},34,&H00E0E0E0,&H00FFFFFF,&H00000000,&H00000000,0,0,0,0,100,100,0,0,1,1,0,5,0,0,0,1
Style: Entete,{POLICE},30,&H0000C8FF,&H00FFFFFF,&H00000000,&H64000000,1,0,0,0,100,100,0,0,3,12,0,7,{MARGE},0,{MARGE},1
Style: Etape,{POLICE},46,&H00FFFFFF,&H00FFFFFF,&H00000000,&H64000000,1,0,0,0,100,100,0,0,3,14,0,7,{MARGE},0,{MARGE + 62},1
Style: Valeurs,DejaVu Sans Mono,26,&H00FFFFFF,&H00FFFFFF,&H00000000,&H64000000,0,0,0,0,100,100,0,0,3,12,0,1,{MARGE},0,{MARGE + 40},1
Style: Note,{POLICE},22,&H00B0B0B0,&H00FFFFFF,&H00000000,&H64000000,0,1,0,0,100,100,0,0,3,8,0,1,{MARGE},0,{MARGE - 18},1
Style: Camera,{POLICE},22,&H00FFFFFF,&H00FFFFFF,&H00000000,&H64000000,0,0,0,0,100,100,0,0,3,8,0,9,0,{MARGE},0,1
Style: Succes,{POLICE},44,&H0050E050,&H00FFFFFF,&H00000000,&H78000000,1,0,0,0,100,100,0,0,3,18,0,5,0,0,0,1
Style: Echec,{POLICE},44,&H005050FF,&H00FFFFFF,&H00000000,&H78000000,1,0,0,0,100,100,0,0,3,18,0,5,0,0,0,1
Style: Legende,{POLICE},24,&H00FFFFFF,&H00FFFFFF,&H00000000,&H64000000,0,0,0,0,100,100,0,0,3,10,0,3,0,{MARGE},{MARGE + 62},1
Style: Bande,{POLICE},20,&H00000000,&H00000000,&H00000000,&H00000000,0,0,0,0,100,100,0,0,1,0,0,7,0,0,0,1
Style: Bilan,{POLICE},40,&H00FFFFFF,&H00FFFFFF,&H00000000,&H00000000,0,0,0,0,100,100,0,0,1,0,0,5,0,0,0,1

[Events]
Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text
"""


# ------------------------------------------------------------------ chronologie

def mm(v: float) -> str:
    return f"{v:+.0f}"


def construire_overlay(evenements: list[dict], t0: float, duree: float, biais_mm: list[float],
                       hauteur_camera: int, hauteur_scene: int | None) -> tuple[str, list[str]]:
    """Retourne (contenu ASS, résumé texte de ce qui s'est passé)."""
    lignes: list[str] = []
    resume: list[str] = []

    seuil = next((float(m.group(1)) for e in evenements if e["type"] == "log"
                  and (m := RE_SEUIL.search(e["msg"]))), SEUIL_PAR_DEFAUT_PX)
    bilan: dict[int, list[str]] = {}

    # Titre, sur un bandeau sombre sous l'incrustation caméra
    haut = MARGE + hauteur_camera + 60  # bandeaux sous l'incrustation caméra
    lignes.append(bande(0, DUREE_TITRE_S, haut, haut + 340))
    lignes.append(ligne(0, DUREE_TITRE_S, "Titre",
                        f"{{\\fad(300,400)\\pos(960,{haut + 95})}}SO-ARM101 · Phase 1 · Pick & place"))
    lignes.append(ligne(0, DUREE_TITRE_S, "SousTitre",
                        f"{{\\fad(300,400)\\pos(960,{haut + 195})}}Simulation MuJoCo + ROS 2 · vue RViz · temps réel, sans coupe"))
    lignes.append(ligne(0.8, DUREE_TITRE_S, "SousTitre",
                        f"{{\\fad(300,400)\\pos(960,{haut + 265})}}Erreur de calibration caméra simulée : "
                        f"{mm(biais_mm[0])} mm en x, {mm(biais_mm[1])} mm en y"))
    # Légende des éléments de RViz
    lignes.append(ligne(DUREE_TITRE_S, duree, "Legende", r"{\fad(300,0)}" + LEGENDE))
    # Note permanente sur l'erreur volontaire
    lignes.append(ligne(DUREE_TITRE_S, duree, "Note",
                        f"Erreur de calibration volontaire ({mm(biais_mm[0])} ; {mm(biais_mm[1])}) mm ajoutée "
                        "à la position vue par la caméra, pour montrer la relance et l'offset appris."))
    # Légende de la caméra
    lignes.append(ligne(0, duree, "Camera",
                        f"{{\\pos({LARGEUR - MARGE},{MARGE + hauteur_camera + 12})\\an9}}"
                        "Caméra · détection du cube (/perception/debug)"))
    if hauteur_scene:
        haut_scene = MARGE + hauteur_camera + ECART_INCRUSTATIONS
        lignes.append(ligne(0, duree, "Camera",
                            f"{{\\pos({LARGEUR - MARGE},{haut_scene + hauteur_scene + 12})\\an9}}"
                            "Scène · simulation MuJoCo"))

    # États successifs, reconstruits depuis les logs
    cube, essai = 0, 1
    offset_essai = [0.0, 0.0]   # décalage visé de l'essai courant (mm)
    offset_appris = [0.0, 0.0]  # décalage du dernier essai validé (mm)
    en_cours = False
    etape, t_etape = None, None
    entete, t_entete = None, None
    objet = None
    valeurs, t_valeurs = None, None

    def fermer_etape(t: float):
        nonlocal etape, t_etape
        if etape is not None:
            lignes.append(ligne(t_etape, t, "Etape", etape))
        etape, t_etape = None, None

    def poser_entete(t: float, texte: str | None):
        nonlocal entete, t_entete
        if entete is not None:
            lignes.append(ligne(t_entete, t, "Entete", entete))
        entete, t_entete = texte, t

    def texte_valeurs() -> str:
        if objet is None:
            return ""
        x, y, z = objet
        return (f"Cube vu par la caméra : x = {x:.3f} m   y = {y:.3f} m   z = {z:.3f} m\\N"
                f"Décalage visé (essai)  : ({mm(offset_essai[0])} ; {mm(offset_essai[1])}) mm\\N"
                f"Offset appris          : ({mm(offset_appris[0])} ; {mm(offset_appris[1])}) mm")

    def rafraichir_valeurs(t: float):
        nonlocal valeurs, t_valeurs
        nouveau = texte_valeurs()
        if nouveau == valeurs:
            return
        if valeurs:
            lignes.append(ligne(t_valeurs, t, "Valeurs", valeurs))
        valeurs, t_valeurs = nouveau, t

    raison_sol = None  # « cube encore sur la table », donné par la dernière vérification de la perception
    for ev in evenements:
        t = ev["t"] - t0
        if t > duree:
            break
        if ev["type"] == "objet":
            objet = (ev["x"], ev["y"], ev["z"])
            rafraichir_valeurs(max(t, DUREE_TITRE_S))
            continue
        if ev["noeud"] == "so101_perception" and "vérification de prise" in ev["msg"]:
            m = RE_RAISON_SOL.search(ev["msg"])
            raison_sol = float(m.group(1)) if m else None
            continue
        if ev["noeud"] != "so101_brain":
            continue
        msg = ev["msg"]
        tt = max(t, DUREE_TITRE_S)

        if (m := RE_VERDICT.match(msg)):
            ok, distance = m.group(1) == "OK", float(m.group(2))
            mesure = f"cube à {px(distance)} de la pince (seuil {px(seuil)})"
            if ok:
                offset_appris = list(offset_essai)
                texte = r"✓ Prise réussie\N{\fs30}" + mesure
                if essai == 1 and any(offset_appris):
                    texte += r"\Ndu premier coup, grâce à l'offset appris"
                lignes.append(ligne(tt, tt + DUREE_VERDICT_S, "Succes", r"{\fad(150,300)\pos(960,540)}" + texte))
                resume.append(f"cube {cube}, essai {essai} : réussi ({px(distance)})")
                bilan.setdefault(cube, []).append(f"{{\\c&H0050E050&}}✓ essai {essai}{{\\c&H00FFFFFF&}} ({px(distance)})")
            else:
                if raison_sol is not None:  # proche dans l'image, mais resté sur la table
                    mesure = f"le cube est resté sur la table ({raison_sol:.1f} cm sous la pince)".replace(".", ",")
                lignes.append(ligne(tt, tt + DUREE_VERDICT_S, "Echec",
                                    r"{\fad(150,300)\pos(960,540)}✗ Prise ratée\N{\fs30}" + mesure
                                    + r"\Nle robot rouvre la pince et réessaie 5 mm à côté"))
                cause = "resté sur la table" if raison_sol is not None else px(distance)
                resume.append(f"cube {cube}, essai {essai} : raté ({cause})")
                bilan.setdefault(cube, []).append(f"{{\\c&H005050FF&}}✗ essai {essai}{{\\c&H00FFFFFF&}} ({cause})")
            rafraichir_valeurs(tt)
            continue
        if (m := RE_RELANCE.match(msg)):
            essai = int(m.group(1))
            offset_essai = [float(m.group(2)), float(m.group(3))]
            poser_entete(tt, f"Cube n°{cube} · essai {essai}/{MAX_ESSAIS}")
            rafraichir_valeurs(tt)
            continue
        if RE_ABANDON.search(msg):
            resume.append(f"cube {cube} : abandon après {essai} essais")
            continue
        if (m := RE_ETAPE.match(msg)):
            cle = m.group(1)
            if cle == "done":
                fermer_etape(tt)
                if en_cours:
                    poser_entete(tt, None)
                en_cours = False
                continue
            if cle == "planned" and not en_cours:  # nouveau pick & place
                en_cours, cube, essai = True, cube + 1, 1
                offset_essai = list(offset_appris)  # cible = vue caméra + offset appris (l. 264)
                poser_entete(tt, f"Cube n°{cube} · essai {essai}/{MAX_ESSAIS}")
                rafraichir_valeurs(tt)
            if cle in ETAPES and en_cours:
                fermer_etape(tt)
                etape, t_etape = ETAPES[cle], tt

    fermer_etape(duree)
    poser_entete(duree, None)
    if valeurs:
        lignes.append(ligne(t_valeurs, duree, "Valeurs", valeurs))

    # Carton de fin : bilan par cube, sur un bandeau sombre
    if bilan:
        debut_fin = max(DUREE_TITRE_S, duree - DUREE_FIN_S)
        texte = r"{\b1\fs48}Bilan{\b0}\N" + r"\N".join(
            f"Cube n°{n} :  " + "  →  ".join(essais) for n, essais in bilan.items())
        texte += r"\N{\fs28\c&H00C8C8C8&}" + f"Vérification par la caméra : prise validée sous {px(seuil)}"
        lignes.append(bande(debut_fin, duree, haut, haut + 340))
        lignes.append(ligne(debut_fin, duree, "Bilan", f"{{\\fad(400,0)\\pos(960,{haut + 170})}}" + texte, couche=2))
    return ENTETE_ASS + "\n".join(lignes) + "\n", resume


# ------------------------------------------------------------------ vidéo caméra

def video_images(dossier: Path, sous_dossier: str, t0: float, duree: float) -> tuple[Path, int] | None:
    """Vidéo (largeur LARGEUR_CAMERA) des images <sous_dossier>/<t>.jpg, calée sur la capture RViz."""
    images = sorted(dossier.glob(f"{sous_dossier}/*.jpg"), key=lambda p: float(p.stem))
    if not images:
        return None
    temps = [float(p.stem) - t0 for p in images]
    # dernière image avant le début, puis toutes celles pendant la capture
    i0 = max(0, sum(1 for t in temps if t <= 0) - 1)
    images, temps = images[i0:], [max(0.0, t) for t in temps[i0:]]
    temps[0] = 0.0
    liste = dossier / f"{sous_dossier}.txt"
    with liste.open("w") as f:
        for i, image in enumerate(images):
            fin = temps[i + 1] if i + 1 < len(images) else duree
            f.write(f"file '{image.resolve()}'\nduration {max(fin - temps[i], 0.001):.3f}\n")
        f.write(f"file '{images[-1].resolve()}'\n")
    sortie = dossier / f"{sous_dossier}.mp4"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", str(liste),
                    "-vf", f"scale={LARGEUR_CAMERA}:-2,fps=30", "-pix_fmt", "yuv420p",
                    "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", str(sortie)], check=True)
    h, w = cv2.imread(str(images[0])).shape[:2]
    return sortie, round(LARGEUR_CAMERA * h / w / 2) * 2


def duree_video(chemin: Path) -> float:
    sortie = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0",
                             str(chemin)], capture_output=True, text=True, check=True)
    return float(sortie.stdout.strip())


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage : montage.py <dossier_rushes>")
    dossier = Path(sys.argv[1])
    t0 = float((dossier / "debut.txt").read_text().strip())
    parametres = json.loads((dossier / "parametres.json").read_text())
    evenements = [json.loads(l) for l in (dossier / "evenements.jsonl").read_text().splitlines() if l.strip()]
    evenements.sort(key=lambda e: e["t"])
    duree = duree_video(dossier / "rviz.mp4")

    video_cam = video_images(dossier, "camera", t0, duree)
    if video_cam is None:
        sys.exit("aucune image caméra dans camera/")
    camera, hauteur_camera = video_cam
    video_scene = video_images(dossier, "scene", t0, duree)
    hauteur_scene = video_scene[1] if video_scene else None
    ass, resume = construire_overlay(evenements, t0, duree, parametres["biais_mm"], hauteur_camera,
                                     hauteur_scene)
    (dossier / "overlay.ass").write_text(ass, encoding="utf-8")

    chrono = (f"drawtext=fontfile={POLICE_MONO}:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.45:"
              f"boxborderw=10:x=w-tw-{MARGE}:y=h-th-{MARGE}:text='t = %{{pts\\:hms}}'")
    entrees = ["-i", str(dossier / "rviz.mp4"), "-i", str(camera)]
    filtre = (f"[1:v]pad=iw+6:ih+6:3:3:color=white[cam];"
              f"[0:v][cam]overlay=W-w-{MARGE}:{MARGE}:eof_action=repeat[v1];")
    if video_scene:
        entrees += ["-i", str(video_scene[0])]
        haut_scene = MARGE + hauteur_camera + ECART_INCRUSTATIONS
        filtre += (f"[2:v]pad=iw+6:ih+6:3:3:color=white[scn];"
                   f"[v1][scn]overlay=W-w-{MARGE}:{haut_scene}:eof_action=repeat[v1b];")
    else:
        filtre += "[v1]null[v1b];"
    filtre += f"[v1b]ass={dossier / 'overlay.ass'}[v2];[v2]{chrono}[v]"
    sortie = dossier / "pick_place_sim.mp4"
    subprocess.run(["ffmpeg", "-loglevel", "error", "-y", *entrees,
                    "-filter_complex", filtre, "-map", "[v]", "-c:v", "libx264", "-preset", "medium",
                    "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(sortie)], check=True)
    print(f"Vidéo : {sortie} ({duree:.1f} s)")
    for r in resume:
        print("  -", r)


if __name__ == "__main__":
    main()
